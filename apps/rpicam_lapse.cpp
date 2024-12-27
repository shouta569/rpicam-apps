/*
 * rpicam_lapse.cpp - libcamera still timelapse record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include "core/frame_info.hpp"
#include "core/rpicam_lapse_encoder.hpp"
#include "core/lapse_options.hpp"
#include "output/output.hpp"

#include <mutex>
#include <condition_variable>

using namespace std::placeholders;

// Some keypress/signal handling.
static int signal_received;

static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}

static int get_key_or_signal(LapseOptions const *options, pollfd p[1])
{
	int key = 0;
	if (signal_received == SIGINT)
		return 'x';
	if (options->keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if ((signal_received == SIGUSR2) || (signal_received == SIGPIPE))
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static void captureImage(RPiCamLapseEncoder &app) {
	app.StartCamera();
	RPiCamLapseEncoder::Msg msg = app.Wait();
	if (msg.type == RPiCamApp::MsgType::Timeout)
	{
		LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
		app.StopCamera();
		app.StartCamera();
		return;
	}
	CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
	app.StopCamera();
	app.EncodeBuffer(completed_request, app.StillStream());
}

// The main even loop for the application.
static void event_loop(RPiCamLapseEncoder &app)
{
	uint64_t frame_count = 0, delayed_frame = 0;
	try {
		LapseOptions const *options = app.GetOptions();
		std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
		app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
		app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

		// Monitoring for keypresses and signals.
		signal(SIGUSR1, default_signal_handler);
		signal(SIGUSR2, default_signal_handler);
		signal(SIGINT, default_signal_handler);
		pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

		app.OpenCamera();

		// Working now, refactoring now...
		if (options->af_on_capture) {
			app.ConfigureViewfinder();
			app.StartCamera();

			libcamera::ControlList cl;
			cl.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
			cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
			app.SetControls(cl);

			LOG(1, "Running autofocus before capturing timelapse...");
			while (true){
				RPiCamApp::Msg msg = app.Wait();
				if (msg.type == RPiCamApp::MsgType::Timeout)
				{
					LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
					app.StopCamera();
					app.StartCamera();
					continue;
				}
				if (msg.type == RPiCamApp::MsgType::Quit)
					return;
				else if (msg.type != RPiCamApp::MsgType::RequestComplete)
					throw std::runtime_error("unrecognised message!");

				CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
				int key = get_key_or_signal(options, p);
				if (key == 'x' || key == 'X') {
					app.StopCamera();
					return;
				}

				FrameInfo fi(completed_request);
				int scanning = fi.af_state;

				if (scanning == libcamera::controls::AfStateIdle)
					LOG(2, "Current AF Scan Status: Idle.");
				else if  (scanning == libcamera::controls::AfStateScanning)
					LOG(2, "Current AF Scan Status: Scanning. Lens position = " << fi.lens_position);
				else {
					LOG(1, "Autofocus completed. Lens position set to " << fi.lens_position);
					break;
				}
			}
			app.StopCamera();
			app.Teardown();
		}

		app.ConfigureStill(RPiCamApp::FLAG_STILL_NONE);
		if (options->af_on_capture)
		{
			libcamera::ControlList cl;
			cl.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
			cl.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerCancel);
			app.SetControls(cl);
		} 
		app.StartEncoder();

		std::mutex main_mtx;
		std::condition_variable main_cv;
		std::unique_lock<std::mutex> lock(main_mtx);

		auto next_capture_time = std::chrono::high_resolution_clock::now();
		const auto end_capture_time = next_capture_time + options->timeout.value;
		const std::time_t start = std::chrono::system_clock::to_time_t(next_capture_time);
		const std::time_t end = std::chrono::system_clock::to_time_t(end_capture_time);

		LOG(1, "Start Time: " << std::ctime(&start) << "End Time: " << std::ctime(&end));

		while (next_capture_time <= end_capture_time) {
			int key = get_key_or_signal(options, p);
			if (key == 'x' || key == 'X') {
				LOG(1, "Captured frame = " << frame_count);
				LOG(1, "Delayed frame = " << delayed_frame);
				app.StopCamera();
				app.StopEncoder();
				return;
			}
			
			if (main_cv.wait_until(lock, next_capture_time) == std::cv_status::timeout){
				captureImage(app);
				frame_count++;
				next_capture_time += options->interval.value;
				auto current_time = std::chrono::high_resolution_clock::now();
				if (current_time > next_capture_time) {
					auto delay_time_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - next_capture_time);
					auto delay_time = delay_time_in_ms.count();
					LOG(1, "Next frame capture delayed by " << delay_time << "us");
					next_capture_time = current_time;
					delayed_frame++;
				}
			}
			else {
				break;
			}
		}
		app.StopCamera();
		app.StopEncoder();
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		// Attempt to stop camera and encoder
		app.StopCamera();
		app.StopEncoder();
	}
	
	LOG(1, "Captured frame = " << frame_count);
	LOG(1, "Delayed frame = " << delayed_frame);
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamLapseEncoder app;
		LapseOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
