/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas on Board Oy.
 *
 * RkISP2 Image Processing Algorithms
 */

#include <algorithm>
#include <functional>
#include <stdint.h>
#include <string.h>
#include <tuple>

#include <linux/media/rockchip/rkisp2-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp2_ipa_interface.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"

#include "ipa_context.h"
#include "params.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPARkISP2)

namespace ipa::rkisp2 {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPARkISP2 : public IPARkISP2Interface, public Module
{
public:
	IPARkISP2();

	int init(const IPASettings &settings,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;

	int configure(const IPAConfigInfo &ipaConfig,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void computeParams(const uint32_t frame, const uint32_t bufferId) override;
	void initializeFrameContext(IPAFrameContext &frameContext,
				    const ControlList &controls);
	void processStats(const uint32_t frame, const uint32_t bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateControls(ControlInfoMap *ipaControls);

	void setControls(unsigned int frame, const IPAFrameContext &frameContext);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

/* List of controls handled by the RkISP2 IPA */
const ControlInfoMap::Map rkisp2Controls{
	{ &controls::DebugMetadataEnable, ControlInfo(false, true, false) },
};

} /* namespace */

IPARkISP2::IPARkISP2()
	: context_(kMaxFrameContexts)
{
	context_.frameContexts.setInitCallback(
		[this](IPAFrameContext &fc, const ControlList &c) {
			this->initializeFrameContext(fc, c);
		});
}

std::string IPARkISP2::logPrefix() const
{
	return "rkisp2";
}

int IPARkISP2::init(const IPASettings &settings,
		    const IPACameraSensorInfo &sensorInfo,
		    const ControlInfoMap &sensorControls,
		    ControlInfoMap *ipaControls)
{
	context_.sensorInfo = sensorInfo;
	context_.sensorControls = sensorControls;

	context_.camHelper = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!context_.camHelper) {
		LOG(IPARkISP2, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	/* Load the tuning data file. */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPARkISP2, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::ValueNode> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	unsigned int version = (*data)["version"].get<uint32_t>(0);
	if (version != 1) {
		LOG(IPARkISP2, Error)
			<< "Invalid tuning file version " << version;
		return -EINVAL;
	}

	if (!data->contains("algorithms")) {
		LOG(IPARkISP2, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	/* Initialize controls. */
	updateControls(ipaControls);

	return 0;
}

int IPARkISP2::start()
{
	/* \todo Properly handle startup controls. */
	return 0;
}

void IPARkISP2::stop()
{
	context_.frameContexts.clear();
}

int IPARkISP2::configure(const IPAConfigInfo &ipaConfig,
			 ControlInfoMap *ipaControls)
{
	context_.sensorInfo = ipaConfig.sensorInfo;
	context_.sensorControls = ipaConfig.sensorControls;

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	context_.configuration.sensor.size = context_.sensorInfo.outputSize;

	context_.configuration.csm.colorSpaceEncoding = ipaConfig.colorSpaceEncoding;
	context_.configuration.csm.colorSpaceRange = ipaConfig.colorSpaceRange;

	for (const auto &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		int ret = algo->configure(context_, context_.sensorInfo);
		if (ret)
			return ret;
	}

	updateControls(ipaControls);

	return 0;
}

void IPARkISP2::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		MappedFrameBuffer mappedBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite);
		if (!mappedBuffer.isValid()) {
			LOG(IPARkISP2, Fatal) << "Failed to mmap buffer: "
					      << strerror(mappedBuffer.error());
		}

		mappedBuffers_.emplace(buffer.id, std::move(mappedBuffer));
	}
}

void IPARkISP2::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		const auto fb = buffers_.find(id);
		if (fb == buffers_.end())
			continue;

		mappedBuffers_.erase(id);
		buffers_.erase(id);
	}
}

void IPARkISP2::queueRequest(const uint32_t frame, const ControlList &controls)
{
	context_.debugMetadata.enableByControl(controls);

	context_.frameContexts.getOrInitContext(frame, controls);
}

void IPARkISP2::initializeFrameContext(IPAFrameContext &frameContext,
				       const ControlList &controls)
{
	for (const auto &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());
		algo->queueRequest(context_, frameContext.frame(), frameContext, controls);
	}
}

void IPARkISP2::computeParams(const uint32_t frame, const uint32_t bufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.getOrInitContext(frame);

	RkISP2Params params(mappedBuffers_.at(bufferId).planes()[0]);

	for (const auto &algo : algorithms())
		algo->prepare(context_, frame, frameContext, &params);

	paramsComputed.emit(frame, bufferId, params.bytesused());
}

void IPARkISP2::processStats(const uint32_t frame, const uint32_t bufferId,
			     const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.getOrInitContext(frame);

	RkISP2Stats stats(mappedBuffers_.at(bufferId).planes()[0]);

	std::tie(frameContext.sensor.exposure, frameContext.sensor.gain) =
		agc::extractControls(sensorControls, context_.camHelper.get());

	ControlList metadata(controls::controls);

	for (const auto &algo : algorithms())
		algo->process(context_, frame, frameContext, &stats, metadata);

	setControls(frame, frameContext);

	metadataReady.emit(metadata);
}

void IPARkISP2::setControls(unsigned int frame, const IPAFrameContext &frameContext)
{
	/*
	 * \todo The frame number is most likely wrong here, we need to take
	 * internal sensor delays and other timing parameters into account.
	 */

	uint32_t exposure = frameContext.agc.exposure;
	uint32_t vblank = frameContext.agc.vblank;

	LOG(IPARkISP2, Debug)
		<< "Set controls for frame " << frame << ": exposure " << exposure
		<< ", gain " << frameContext.agc.gain << ", vblank " << vblank;

	ControlList ctrls(context_.sensorControls);
	if (frameContext.agc.exposure * frameContext.agc.gain > 0)
		agc::prepareControls(ctrls, context_.camHelper.get(),
				     exposure, frameContext.agc.gain);
	ctrls.set(V4L2_CID_VBLANK, static_cast<int32_t>(vblank));

	setSensorControls.emit(frame, ctrls);
}

void IPARkISP2::updateControls(ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap = rkisp2Controls;

	const IPACameraSensorInfo &sensorInfo = context_.sensorInfo;

	Rectangle ispMinCrop{ 0, 0, 32, 32 };
	/*
	 * No need to clamp this as the hardware will hang anyway if sensor
	 * size > isp max size
	 */
	Rectangle ispMaxCrop{ 0, 0, sensorInfo.outputSize };
	/*
	 * \todo Either always enable Crop algo or make this conditional on
	 * when the Crop algo is present
	 */
	context_.ctrlMap[&controls::ScalerCrop] =
		ControlInfo(ispMinCrop, ispMaxCrop, ispMaxCrop);

	ctrlMap.insert(context_.ctrlMap.begin(), context_.ctrlMap.end());
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

} /* namespace ipa::rkisp2 */

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"rkisp2",
};

IPAInterface *ipaCreate()
{
	return new ipa::rkisp2::IPARkISP2();
}
}

} /* namespace libcamera */
