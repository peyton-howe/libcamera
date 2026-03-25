/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board
 *
 * Sequence sync helper
 */

#pragma once

#include <map>
#include <memory>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>

#include <libcamera/framebuffer.h>

#include "sequence_sync_helper.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RkISP1Schedule)

struct BufferQueueDelegateBase {
	virtual ~BufferQueueDelegateBase() = default;
	virtual int allocateBuffers(unsigned int count,
				    std::vector<std::unique_ptr<FrameBuffer>> *buffers) = 0;
	virtual int importBuffers(unsigned int count) = 0;
	virtual int releaseBuffers() = 0;

	virtual int queueBuffer(FrameBuffer *buffer) = 0;

	Signal<FrameBuffer *> bufferReady;
};

template<typename T>
struct BufferQueueDelegate : public BufferQueueDelegateBase {
	BufferQueueDelegate(T *video) : video_(video)
	{
		video_->bufferReady.connect(this, [this](FrameBuffer *buffer) {
			this->bufferReady.emit(buffer);
		});
	}

	int allocateBuffers(unsigned int count,
			    std::vector<std::unique_ptr<FrameBuffer>> *buffers) override
	{
		return video_->allocateBuffers(count, buffers);
	}

	int importBuffers(unsigned int count) override
	{
		return video_->importBuffers(count);
	}

	int queueBuffer(FrameBuffer *buffer) override
	{
		return video_->queueBuffer(buffer);
	}

	int releaseBuffers() override
	{
		return video_->releaseBuffers();
	}

private:
	T *video_;
};

class BufferQueue
{
public:
	enum State {
		Idle = 0,
		Preparing,
		Capturing,
		Postprocessing
	};

	enum Flags {
		PrepareStage = 1,
		PostprocessStage = 2
	};

	BufferQueue(std::unique_ptr<BufferQueueDelegateBase> &&delegate, int flags = 0, std::string name = {});

	int allocateBuffers(unsigned int count);
	int importBuffers(unsigned int count);
	int releaseBuffers();

	int sequenceCorrection();
	uint32_t nextSequence();

	int prepareBuffer(uint32_t *sequence = nullptr);
	int prepareBuffer(FrameBuffer *buffer, uint32_t *sequence = nullptr);
	int preparedBuffer();

	int queueBuffer(uint32_t *sequence = nullptr);
	int queueBuffer(FrameBuffer *buffer, uint32_t *sequence = nullptr);

	void postprocessedBuffer();

	bool empty(State state);

	FrameBuffer *front(State state);

	unsigned int expectedSequence(FrameBuffer *buffer) const;
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers() const;

	Signal<FrameBuffer *> bufferReady;

protected:
	void onBufferReady(FrameBuffer *buffer);

	int internalPrepareBuffer(FrameBuffer *buffer, uint32_t *sequence = nullptr);
	int internalPreparedBuffer();
	void internalPostprocessedBuffer();

	std::map<State, std::list<FrameBuffer *>> bufferState_;
	std::map<FrameBuffer *, unsigned int> expectedSequence_;
	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
	SequenceSyncHelper syncHelper_;
	uint32_t nextSequence_;
	std::string name_;
	bool ownsBuffers_;
	bool hasBuffers_;
	int flags_;
	std::unique_ptr<BufferQueueDelegateBase> delegate_;
};

} /* namespace libcamera */
