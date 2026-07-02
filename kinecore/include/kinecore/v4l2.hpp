#pragma once

// V4L2 control IDs used by the Pi's hardware H.264 video encoder.
// kinecore itself does not call these — they live here because they are
// platform-level constants shared by future encode/analytics modules.
// kineencode::H264Encoder consumes them when migrated in task #3.

#include <cstdint>

namespace kinecore::v4l2 {

enum class VideoEncoder : uint32_t {
    GOP_SIZE          = 0x009909cb,  // V4L2_CID_MPEG_VIDEO_GOP_SIZE
    FORCE_KEY_FRAME   = 0x009909e5,  // V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME
    REPEAT_SEQ_HEADER = 0x009909e2,  // V4L2_CID_MPEG_VIDEO_H264_I_PERIOD
    BITRATE_MODE      = 0x009909ce,  // V4L2_CID_MPEG_VIDEO_BITRATE_MODE
    BITRATE           = 0x009909cf,  // V4L2_CID_MPEG_VIDEO_BITRATE
};

}  // namespace kinecore::v4l2
