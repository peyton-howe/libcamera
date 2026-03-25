/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * IPA Frame context queue
 */

#pragma once

#include <stdint.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/controls.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(FCQueue)

namespace ipa {

template<typename FrameContext>
class FCQueue;

struct FrameContext {
	uint32_t frame() const { return frame_; }

private:
	template<typename T> friend class FCQueue;
	uint32_t frame_;
};

template<typename FC>
class FCQueue
{
public:
	using InitCallback = std::function<void(FC &, const ControlList &)>;

	FCQueue(unsigned int size)
		: contexts_(size)
	{
	}

	void setInitCallback(const InitCallback &cb)
	{
		initCallback_ = cb;
	}

	void clear()
	{
		for (FC &ctx : contexts_) {
			ctx.frame_ = 0;
		}
		initialized_ = false;
	}

	FC &getOrInitContext(unsigned int frame, const ControlList &controls = {})
	{
		FC &fc = contexts_[frame % contexts_.size()];
		FrameContext &frameContext = fc;

		/*
		 * If the IPA algorithms try to access a frame context slot which
		 * has been already overwritten by a newer context, it means the
		 * frame context queue has overflowed and the desired context
		 * has been forever lost. The pipeline handler shall avoid
		 * queueing more requests to the IPA than the frame context
		 * queue size.
		 */
		if (frame < frameContext.frame_)
			LOG(FCQueue, Fatal) << "Frame context for " << frame
					    << " has been overwritten by "
					    << frameContext.frame_;

		if (initialized_ && frame == frameContext.frame_) {
			if (!controls.empty()) {
				/* Too late to apply the controls. Store them for later. */
				LOG(FCQueue, Warning)
					<< "Request underrun. Controls for frame "
					<< frame << " are delayed ";
				controlsToApply_.merge(controls,
						       ControlList::MergePolicy::OverwriteExisting);
			}
			LOG(FCQueue, Debug) << "Got " << frame;
			return fc;
		}

		const ControlList *controls2 = &controls;
		if (!controlsToApply_.empty()) {
			LOG(FCQueue, Debug) << "Applied late controls on frame" << frame;
			controlsToApply_.merge(controls, ControlList::MergePolicy::OverwriteExisting);
			controls2 = &controlsToApply_;
		}

		LOG(FCQueue, Debug) << "Init " << frame;

		fc = {};
		frameContext.frame_ = frame;
		initCallback_(fc, *controls2);
		initialized_ = true;
		controlsToApply_.clear();

		return fc;
	}

private:
	std::vector<FC> contexts_;
	InitCallback initCallback_;
	ControlList controlsToApply_;
	bool initialized_;
};

} /* namespace ipa */

} /* namespace libcamera */
