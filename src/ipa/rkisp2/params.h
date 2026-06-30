/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 ISP Parameters
 */

#pragma once

#include <stdint.h>

#include <linux/media/rockchip/rkisp2-config.h>
#include <linux/videodev2.h>

#include <libipa/v4l2_params.h>

namespace libcamera {

namespace ipa::rkisp2 {

enum class RkISP2ParamsBlocks : uint16_t {
	Bls,
	AwbGains,
	Csm,
	Ccm,
	Goc,
	Lsc,
	Crop,
	AeLite,
	HistLite,
	HistBig0,
	HistBig1,
	HistBig2,
	AwbMeas,
};

namespace details {

template<RkISP2ParamsBlocks B>
struct block_type {
};

#define RkISP2_DEFINE_PARAMS_BLOCK_TYPE(id, cfgType, blkType)		\
template<>								\
struct block_type<RkISP2ParamsBlocks::id> {				\
	using type = struct rkisp2_params_##cfgType;			\
	static constexpr rkisp2_params_block_type blockType = 		\
		rkisp2_params_block_type::RKISP2_PARAMS_BLOCK_##blkType;\
}

RkISP2_DEFINE_PARAMS_BLOCK_TYPE(Bls,		bls,		BLS);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(AwbGains,	awb_gains,	AWB_GAINS);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(Csm,		csm,		CSM);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(Ccm,		ccm,		CCM);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(Goc,		goc,		GOC);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(Lsc,		lsc,		LSC);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(Crop,		crop,		CROP);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(AeLite,		ae_lite,	AE_LITE);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(HistLite,	hist_lite,	HIST_LITE);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(HistBig0,	hist_big,	HIST_BIG0);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(HistBig1,	hist_big,	HIST_BIG1);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(HistBig2,	hist_big,	HIST_BIG2);
RkISP2_DEFINE_PARAMS_BLOCK_TYPE(AwbMeas,	awb_meas,	AWB_MEAS);

struct param_traits {
	using id_type = RkISP2ParamsBlocks;

	template<id_type Id>
	using id_to_details = block_type<Id>;
};

} /* namespace details */

class RkISP2Params : public V4L2Params<details::param_traits>
{
public:
	RkISP2Params(Span<uint8_t> data)
		: V4L2Params(data, V4L2_ISP_VERSION_V1)
	{
	}
};

} /* namespace ipa::rkisp2 */

} /* namespace libcamera */
