/*
 * lapse_options.hpp - still timelapse capture program options
 */

#pragma once

#include <cstdio>

#include <string>

#include "video_options.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

struct LapseOptions : public VideoOptions
{
	LapseOptions() : VideoOptions()
	{
		using namespace boost::program_options;
		using namespace boost;
		// clang-format off
		options_.add_options()
			("interval", value<std::string>(&interval_)->default_value("1000ms"),
			 "Time interval between timelapse captures. If no units are provided default to ms.")
			("autofocus-on-capture", value<bool>(&af_on_capture)->default_value(false)->implicit_value(true),
			 "Switch to AfModeAuto and trigger a scan just before capturing timelapse. Autofocus will only run once.");

		// Override no-raw option default value to true to prevent unintended image cropping
		option_description const& no_raw_opt = options_.find("no-raw", false);
		shared_ptr<const value_semantic> no_raw_const_value_semantic = no_raw_opt.semantic();
		shared_ptr<value_semantic> no_raw_value_semantic = const_pointer_cast<value_semantic>(no_raw_const_value_semantic);
		shared_ptr<typed_value<bool>> no_raw_typed_value = dynamic_pointer_cast<typed_value<bool>>(no_raw_value_semantic);
		no_raw_typed_value->default_value(true);
	}

	TimeVal<std::chrono::milliseconds> interval;

	virtual bool Parse(int argc, char *argv[]) override
	{
		if (VideoOptions::Parse(argc, argv) == false)
			return false;

		interval.set(interval_);

		return true;
	}
	virtual void Print() const override
	{
		VideoOptions::Print();
		std::cerr << "    timelapse interval: " << interval.get() << "ms" << std::endl;
		std::cerr << "    AF on capture: " << af_on_capture << std::endl;
	}

private:
	std::string interval_;
};
