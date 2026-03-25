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

namespace libcamera {

LOG_DECLARE_CATEGORY(FCQueue)

namespace ipa {

template<typename FrameContext>
class FCQueue;

struct FrameContext {
private:
	template<typename T> friend class FCQueue;
	uint32_t frame_;
	bool initialised_ = false;
};

template<typename FC>
class FCQueue
{
public:
	FCQueue(unsigned int size)
		: contexts_(size)
	{
	}

	void clear()
	{
		for (FC &ctx : contexts_) {
			ctx.initialised_ = false;
			ctx.frame_ = 0;
		}
	}

	FC &alloc(const uint32_t frame)
	{
		FC &fc = contexts_[frame % contexts_.size()];
		FrameContext &frameContext = fc;

		/*
		 * Do not re-initialise if a get() call has already fetched this
		 * frame context to preseve the context.
		 *
		 * \todo If the the sequence number of the context to initialise
		 * is smaller than the sequence number of the queue slot to use,
		 * it means that we had a serious request underrun and more
		 * frames than the queue size has been produced since the last
		 * time the application has queued a request. Does this deserve
		 * an error condition ?
		 */
		if (frame != 0 && frame <= frameContext.frame_)
			LOG(FCQueue, Warning)
				<< "Frame " << frame << " already initialised";
		else
			init(fc, frame);

		return fc;
	}

	FC &get(uint32_t frame)
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

		if (frame == 0 && !frameContext.initialised_) {
			/*
			 * If the IPA calls get() at start() time it will get an
			 * un-intialized FrameContext as the below "frame ==
			 * frameContext.frame_" check will return success
			 * because FrameContexts are zeroed at creation time.
			 *
			 * Make sure the FrameContext gets initialised if get()
			 * is called before alloc() by the IPA for frame#0.
			 */
			init(fc, frame);

			return fc;
		}

		if (frame == frameContext.frame_)
			return fc;

		/*
		 * The frame context has been retrieved before it was
		 * initialised through the initialise() call. This indicates an
		 * algorithm attempted to access a Frame context before it was
		 * queued to the IPA. Controls applied for this request may be
		 * left unhandled.
		 *
		 * \todo Set an error flag for per-frame control errors.
		 */
		LOG(FCQueue, Warning)
			<< "Obtained an uninitialised FrameContext for " << frame;

		init(fc, frame);

		return fc;
	}

private:
	void init(FC &fc, const uint32_t frame)
	{
		fc = {};
		FrameContext &frameContext = fc;
		frameContext.frame_ = frame;
		frameContext.initialised_ = true;
	}

	std::vector<FC> contexts_;
};

} /* namespace ipa */

} /* namespace libcamera */
