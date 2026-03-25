/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board
 *
 * Sequence sync helper
 */

#pragma once

#include <queue>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(SequenceSyncHelper)

class SequenceSyncHelper
{
public:
	int gotFrame(size_t expectedSequence, size_t actualSequence)
	{
		ASSERT(!corrections_.empty());
		int diff = actualSequence - expectedSequence;
		int corr = corrections_.front();
		corrections_.pop();
		expectedOffset_ -= corr;
		int necessaryCorrection = diff - expectedOffset_;
		correctionToApply_ += necessaryCorrection;

		LOG(SequenceSyncHelper, Debug) << "Sync frame "
					       << "expected: " << expectedSequence
					       << " actual: " << actualSequence
					       << " correction: " << corr
					       << " expectedOffset: " << expectedOffset_
					       << " correctionToApply " << correctionToApply_;

		expectedOffset_ += necessaryCorrection;
		return necessaryCorrection;
	}

	void cancelFrame()
	{
		int corr = corrections_.front();
		corrections_.pop();
		expectedOffset_ -= corr;
	}

	/* Value to be added to the source sequence */
	int correction()
	{
		return correctionToApply_;
	}

	void pushCorrection(int correction)
	{
		corrections_.push(correction);
		correctionToApply_ -= correction;
		LOG(SequenceSyncHelper, Debug)
			<< "Push correction " << correction
			<< " correctionToApply " << correctionToApply_;
	}

	void reset()
	{
		corrections_ = {};
		correctionToApply_ = 0;
		expectedOffset_ = 0;
	}

	std::queue<int> corrections_;
	int correctionToApply_ = 0;
	int expectedOffset_ = 0;
};

} /* namespace libcamera */
