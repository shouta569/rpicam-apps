/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * negate_stage.cpp - image negate effect
 */

#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include <queue>

#include <thread>
#include <chrono>

using Stream = libcamera::Stream;

class MotionExtractionStage : public PostProcessingStage
{
public:
	MotionExtractionStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	void Stop() override;

private:
	Stream *stream_;
	size_t bufferSize, ySize, uvSize;
	std::queue<uint8_t*> frameBuffers;
	uint8_t frameOffset;
	uint8_t bufferCount, prerecordFrameCount, bufferFrameStart;
	float scale;
	float fadeFactor;
	bool fixedFrame, skip_uv_diff;
};

#define NAME "motion_extraction"

char const *MotionExtractionStage::Name() const
{
	return NAME;
}

void MotionExtractionStage::Read(boost::property_tree::ptree const &params)
{
	frameOffset = params.get<uint8_t>("frame_offset", 1);
	fixedFrame = params.get<bool>("fixed_frame", false);
	uint8_t prerecordFrames = params.get<uint8_t>("prerecord_frames", 0);
	scale = params.get<float>("scale", 1.0);
	skip_uv_diff = params.get<bool>("skip_uv_diff", false);

	if (fixedFrame)
		frameOffset = 1;

	if (prerecordFrames == 0) {
		bufferFrameStart = 0;
		fadeFactor = 1.0;
	}
	else {
		bufferFrameStart = prerecordFrames - frameOffset;
		fadeFactor = 0.0;
	}
}

void MotionExtractionStage::Configure()
{
	stream_ = app_->GetMainStream();

	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("AnnotateCvStage: only YUV420 format supported");
	StreamInfo info_ = app_->GetStreamInfo(stream_);
	bufferSize = info_.width * info_.height;
	if (!skip_uv_diff)
		bufferSize = bufferSize * 3 / 2;
	ySize = info_.width * info_.height;
	uvSize = info_.width * info_.height / 2;
	bufferCount = 0;
	prerecordFrameCount = 0;
	std::cout << "Buffer size = " << bufferSize << std::endl;
}

bool MotionExtractionStage::Process(CompletedRequestPtr &completed_request)
{
	if (prerecordFrameCount < bufferFrameStart) {
		prerecordFrameCount++;
		return false;
	}
	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint8_t *ptr = (uint8_t *)buffer.data();

	if ((!fixedFrame) || (fixedFrame && bufferCount == 0)) {
		uint8_t *newFrame = new uint8_t[bufferSize];
		memcpy(newFrame, ptr, bufferSize);
		frameBuffers.push(newFrame);
	}

	if (bufferCount < frameOffset) {
		bufferCount++;
		return false;
	}

	float scaleFactor = scale;

	uint8_t *oldPtr = frameBuffers.front();
	if (!fixedFrame)
		frameBuffers.pop();
	if (fadeFactor < 1.0) {
		fadeFactor += 0.05;
		scaleFactor = 1.0;
	}

	// Y components
	for (size_t i = 0; i < ySize; i++, ptr++, oldPtr++) {
		*ptr = std::abs(*ptr - (*oldPtr * fadeFactor)) * scaleFactor;
	}
	// U and V components
	if (!skip_uv_diff) {
		for (size_t i = 0; i < uvSize; i++, ptr++, oldPtr++) {
			*ptr = static_cast<uint8_t>((int)*ptr - (int)*oldPtr + 128);
		}
	}

	if (!fixedFrame) {
		oldPtr -= bufferSize;
		delete[] oldPtr;
	}
	return false;
}

void MotionExtractionStage::Stop() {
	// Set 5-second delay, more than enough time to avoid unintended segmentation fault.
    std::this_thread::sleep_for(std::chrono::seconds(5));  
	while (!frameBuffers.empty()){
		delete[] frameBuffers.front();
        frameBuffers.pop();
    }
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new MotionExtractionStage(app);
}

static RegisterStage reg(NAME, &Create);
