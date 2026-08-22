/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 ISP Statistics
 */

#pragma once

#include <stdint.h>

#include <linux/media/rockchip/rkisp2-config.h>
#include <linux/videodev2.h>

#include <libipa/v4l2_stats.h>

namespace libcamera {

namespace ipa::rkisp2 {

enum class RkISP2StatsBlocks : uint16_t {
	AeLite,
	HistLite,
	HistBig0,
	HistBig1,
	HistBig2,
	Awb,
};

template<RkISP2StatsBlocks B>
struct block_type {
};

#define RKISP2_DEFINE_STATS_BLOCK_TYPE(id, cfgType, blkType)		\
template<>								\
struct block_type<RkISP2StatsBlocks::id> {				\
	using type = struct rkisp2_stats_##cfgType;			\
	static constexpr rkisp2_stats_block_type blockType = 		\
		rkisp2_stats_block_type::RKISP2_STATS_BLOCK_##blkType;	\
}

RKISP2_DEFINE_STATS_BLOCK_TYPE(AeLite,		ae_lite,	AE_LITE);
RKISP2_DEFINE_STATS_BLOCK_TYPE(HistLite,	hist,		HIST_LITE);
RKISP2_DEFINE_STATS_BLOCK_TYPE(HistBig0,	hist,		HIST_BIG0);
RKISP2_DEFINE_STATS_BLOCK_TYPE(HistBig1,	hist,		HIST_BIG1);
RKISP2_DEFINE_STATS_BLOCK_TYPE(HistBig2,	hist,		HIST_BIG2);
RKISP2_DEFINE_STATS_BLOCK_TYPE(Awb,		awb,		AWB);

struct stats_traits {
	using id_type = RkISP2StatsBlocks;
	template<id_type Id> using id_to_details = block_type<Id>;
};

class RkISP2Stats : public V4L2Stats<stats_traits>
{
public:
	RkISP2Stats(Span<uint8_t> data)
		: V4L2Stats(data, V4L2_ISP_VERSION_V1)
	{
	}
};

} /* namespace ipa::rkisp2 */

} /* namespace libcamera */
