#include <../../build/include/libcamera/libcamera.h>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <turbojpeg.h>
#include <vector>
#include <sys/mman.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <curl/curl.h>
#include <map>
#include <string>
#include <queue>
#include <condition_variable>
#include <atomic>

// ARM NEON intrinsics for SIMD optimization
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

using namespace libcamera;

// V4L2 Control IDs for video encoder
namespace V4L2Controls {
    enum class VideoEncoder : uint32_t {
        GOP_SIZE = 0x009909cb,           // V4L2_CID_MPEG_VIDEO_GOP_SIZE
        FORCE_KEY_FRAME = 0x009909e5,    // V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME
        REPEAT_SEQ_HEADER = 0x009909e2,  // V4L2_CID_MPEG_VIDEO_H264_I_PERIOD (SPS/PPS repeat)
        BITRATE_MODE = 0x009909ce,       // V4L2_CID_MPEG_VIDEO_BITRATE_MODE
        BITRATE = 0x009909cf             // V4L2_ID_MPEG_VIDEO_BITRATE
    };
}

// HTTP Client - wrapper around libcurl for multipart/form-data POST requests
class HttpClient {
public:
    struct Response {
        long status_code = 0;
        std::string body;
        bool success = false;
    };

    HttpClient() {
        curl_global_init(CURL_GLOBAL_ALL);
    }

    ~HttpClient() {
        curl_global_cleanup();
    }

    // Callback for writing response data
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        size_t total_size = size * nmemb;
        std::string* response = static_cast<std::string*>(userp);
        response->append(static_cast<char*>(contents), total_size);
        return total_size;
    }

    // POST multipart/form-data with file
    Response postMultipartFile(
        const std::string& url,
        const std::map<std::string, std::string>& fields,
        const std::string& file_field_name,
        const std::vector<unsigned char>& file_data,
        const std::string& filename,
        int timeout_ms = 10000
    ) {
        Response response;
        CURL* curl = curl_easy_init();

        if (!curl) {
            std::cerr << "Failed to initialize CURL" << std::endl;
            return response;
        }

        curl_mime* mime = curl_mime_init(curl);

        // Add text fields
        for (const auto& [key, value] : fields) {
            curl_mimepart* part = curl_mime_addpart(mime);
            curl_mime_name(part, key.c_str());
            curl_mime_data(part, value.c_str(), CURL_ZERO_TERMINATED);
        }

        // Add file data
        curl_mimepart* file_part = curl_mime_addpart(mime);
        curl_mime_name(file_part, file_field_name.c_str());
        curl_mime_data(file_part, reinterpret_cast<const char*>(file_data.data()), file_data.size());
        curl_mime_filename(file_part, filename.c_str());
        curl_mime_type(file_part, "application/octet-stream");

        // Configure CURL
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        // Perform request
        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK) {
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
            response.success = (response.status_code >= 200 && response.status_code < 300);
        } else {
            std::cerr << "CURL error: " << curl_easy_strerror(res) << std::endl;
            response.success = false;
        }

        // Cleanup
        curl_mime_free(mime);
        curl_easy_cleanup(curl);

        return response;
    }
};

// Telegram Bot API client
class TelegramBotApi {
private:
    std::string bot_token_;
    std::string base_url_;
    HttpClient http_client_;

    int max_retries_ = 3;
    int retry_delay_ms_ = 1000;

    // Send with retry logic
    bool sendWithRetry(
        const std::string& method,
        const std::map<std::string, std::string>& fields,
        const std::string& file_field,
        const std::vector<unsigned char>& file_data,
        const std::string& filename
    ) {
        for (int attempt = 0; attempt < max_retries_; ++attempt) {
            std::string url = base_url_ + method;

            auto response = http_client_.postMultipartFile(
                url, fields, file_field, file_data, filename
            );

            if (response.success) {
                std::cout << "Telegram API: " << method << " success (attempt "
                         << (attempt + 1) << ")" << std::endl;
                return true;
            }

            std::cerr << "Telegram API: " << method << " failed (attempt "
                     << (attempt + 1) << "/" << max_retries_ << "): "
                     << "HTTP " << response.status_code << std::endl;

            if (attempt < max_retries_ - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
            }
        }

        std::cerr << "Telegram API: " << method << " failed after "
                 << max_retries_ << " attempts" << std::endl;
        return false;
    }

public:
    TelegramBotApi(const std::string& bot_token)
        : bot_token_(bot_token),
          base_url_("https://api.telegram.org/bot" + bot_token + "/") {}

    // Send photo (JPEG)
    bool sendPhoto(
        const std::string& chat_id,
        const std::vector<unsigned char>& jpeg_data,
        const std::string& caption = std::string()
    ) {
        std::map<std::string, std::string> fields;
        fields["chat_id"] = chat_id;
        if (!caption.empty()) {
            fields["caption"] = caption;
        }

        return sendWithRetry("sendPhoto", fields, "photo", jpeg_data, "photo.jpg");
    }

    // Send video (MP4)
    bool sendVideo(
        const std::string& chat_id,
        const std::vector<unsigned char>& mp4_data,
        int duration = 0,
        const std::string& caption = std::string()
    ) {
        std::map<std::string, std::string> fields;
        fields["chat_id"] = chat_id;
        if (duration > 0) {
            fields["duration"] = std::to_string(duration);
        }
        if (!caption.empty()) {
            fields["caption"] = caption;
        }

        return sendWithRetry("sendVideo", fields, "video", mp4_data, "video.mp4");
    }

    // Send animation (GIF)
    bool sendAnimation(
        const std::string& chat_id,
        const std::vector<unsigned char>& gif_data,
        const std::string& caption = std::string()
    ) {
        std::map<std::string, std::string> fields;
        fields["chat_id"] = chat_id;
        if (!caption.empty()) {
            fields["caption"] = caption;
        }

        return sendWithRetry("sendAnimation", fields, "animation", gif_data, "animation.gif");
    }
};

class MotionDetector {
private:
    // Previous Y frame (downsampled)
    std::vector<unsigned char> prev_y_frame_;

    // Frame dimensions
    int full_width_, full_height_;      // 1920x1080
    int sample_width_, sample_height_;  // 480x270 (each 4th line/column)

    // Detection parameters
    float threshold_ = 25.0f;           // Pixel difference threshold
    float motion_area_threshold_ = 0.05f; // 5% area for motion
    int downsample_factor_ = 4;         // Skip every 4th pixel

    // Statistics
    float last_motion_level_ = 0.0f;
    int motion_pixel_count_ = 0;
    bool motion_detected_ = false;
    bool first_frame_ = true;

public:
    MotionDetector(int width, int height, int downsample = 4)
        : full_width_(width), full_height_(height), downsample_factor_(downsample) {

        sample_width_ = full_width_ / downsample_factor_;
        sample_height_ = full_height_ / downsample_factor_;
        prev_y_frame_.resize(sample_width_ * sample_height_);

        std::cout << "MotionDetector initialized: " << full_width_ << "x" << full_height_
                  << " -> " << sample_width_ << "x" << sample_height_
                  << " (downsample: " << downsample_factor_ << ")" << std::endl;
    }

    bool detectMotion(unsigned char* y_data, size_t y_size) {
        // Reset statistics
        motion_pixel_count_ = 0;
        motion_detected_ = false;

        // Skip first frame (no previous frame to compare)
        if (first_frame_) {
            downsampleYFrame(y_data, prev_y_frame_.data());
            first_frame_ = false;
            return false;
        }

        // Downsample current frame
        std::vector<unsigned char> current_frame(sample_width_ * sample_height_);
        downsampleYFrame(y_data, current_frame.data());

        // Compare with previous frame using NEON if available
        int total_pixels = sample_width_ * sample_height_;

#ifdef __ARM_NEON
        motion_pixel_count_ = compareFramesNEON(current_frame.data(), prev_y_frame_.data(), total_pixels);
#else
        motion_pixel_count_ = compareFramesScalar(current_frame.data(), prev_y_frame_.data(), total_pixels);
#endif

        // Calculate motion level as percentage
        last_motion_level_ = (float)motion_pixel_count_ / total_pixels * 100.0f;

        // Detect motion if threshold exceeded
        motion_detected_ = last_motion_level_ > (motion_area_threshold_ * 100.0f);

        // Store current frame as previous for next iteration
        prev_y_frame_ = std::move(current_frame);

        return motion_detected_;
    }

    // Configuration methods
    void setThreshold(float threshold) { threshold_ = threshold; }
    void setMotionAreaThreshold(float area_threshold) { motion_area_threshold_ = area_threshold; }
    void setDownsampleFactor(int factor) {
        downsample_factor_ = factor;
        sample_width_ = full_width_ / downsample_factor_;
        sample_height_ = full_height_ / downsample_factor_;
        prev_y_frame_.resize(sample_width_ * sample_height_);
        first_frame_ = true; // Reset detection
    }

    // Statistics methods
    float getMotionLevel() const { return last_motion_level_; }
    int getMotionPixelCount() const { return motion_pixel_count_; }
    bool isMotionDetected() const { return motion_detected_; }

private:
    void downsampleYFrame(unsigned char* full_frame, unsigned char* sampled_frame) {
        const int block_size = 16;  // Process 16 rows at a time for better cache locality

        for (int block_y = 0; block_y < sample_height_; block_y += block_size) {
            int end_y = std::min(block_y + block_size, sample_height_);

            // Prefetch next block data while processing current block
            if (block_y + block_size < sample_height_) {
                int next_block_y = (block_y + block_size) * downsample_factor_;
                __builtin_prefetch(&full_frame[next_block_y * full_width_], 0, 1);
            }

            for (int y = block_y; y < end_y; y++) {
                for (int x = 0; x < sample_width_; x++) {
                    // Sample every downsample_factor_-th pixel
                    int full_idx = y * downsample_factor_ * full_width_ + x * downsample_factor_;
                    int sample_idx = y * sample_width_ + x;

                    sampled_frame[sample_idx] = full_frame[full_idx];
                }
            }
        }
    }

#ifdef __ARM_NEON

    int compareFramesNEON_C(unsigned char* current, unsigned char* previous, int total_pixels) {
        const uint8_t threshold = (uint8_t)threshold_;
        uint8x16_t thresh_vec = vdupq_n_u8(threshold);
        int32x4_t motion_count_vec = vdupq_n_s32(0);

        // Process 16 pixels at a time with NEON - integrated diff+count
        int simd_size = total_pixels & ~15;
        for (int i = 0; i < simd_size; i += 16) {
            // Load 16 pixels from both frames
            uint8x16_t f1 = vld1q_u8(&current[i]);
            uint8x16_t f2 = vld1q_u8(&previous[i]);

            // Calculate absolute difference and compare with threshold
            uint8x16_t abs_diff = vabdq_u8(f1, f2);
            uint8x16_t mask = vcgtq_u8(abs_diff, thresh_vec);

            // Convert 0xFF to 1, count motion pixels
            uint8x16_t ones = vandq_u8(mask, vdupq_n_u8(1));

            // Sum up the motion pixels
            uint16x8_t sum8 = vpaddlq_u8(ones);
            uint32x4_t sum4 = vpaddlq_u16(sum8);
            motion_count_vec = vaddq_s32(motion_count_vec, vreinterpretq_s32_u32(sum4));
        }

        // Sum the 4 lanes to get total motion count
        int motion_count = vgetq_lane_s32(motion_count_vec, 0) +
                          vgetq_lane_s32(motion_count_vec, 1) +
                          vgetq_lane_s32(motion_count_vec, 2) +
                          vgetq_lane_s32(motion_count_vec, 3);

        // Handle remaining pixels (if any)
        for (int i = simd_size; i < total_pixels; i++) {
            int diff = abs(current[i] - previous[i]);
            if (diff > threshold) {
                motion_count++;
            }
        }

        return motion_count;
    }

    // Assembly inline version (maximum performance)
    int compareFramesNEON_ASM(unsigned char* current, unsigned char* previous, int total_pixels) {
        const uint8_t threshold = (uint8_t)threshold_;
        int motion_count = 0;
        int simd_size = total_pixels & ~15;

        if (simd_size > 0) {
            asm volatile (
                "dup v0.16b, %w[thresh]      \n"  // v0 = threshold vector (16 copies of threshold)
                "movi v1.4s, #0              \n"  // v1 = motion count accumulator (4x32bit zeros)
                "movi v10.16b, #1            \n"  // v10 = vector of 1s (create once outside loop)
                "mov x0, #0                  \n"  // x0 = loop counter (byte offset)

                "1:                          \n"  // loop start label
                "ldr q2, [%[curr], x0]       \n"  // load 16 current pixels into v2
                "ldr q3, [%[prev], x0]       \n"  // load 16 previous pixels into v3

                "uabd v4.16b, v2.16b, v3.16b \n"  // absolute difference: |curr - prev|
                "cmhi v5.16b, v4.16b, v0.16b \n"  // compare: diff > threshold (0xFF if true, 0x00 if false)
                "and v6.16b, v5.16b, v10.16b \n"  // convert 0xFF to 1, 0x00 to 0 (using pre-created v10)

                "uaddlp v7.8h, v6.16b       \n"  // sum adjacent pairs: 16x8bit -> 8x16bit
                "uaddlp v8.4s, v7.8h        \n"  // sum adjacent pairs: 8x16bit -> 4x32bit
                "add v1.4s, v1.4s, v8.4s    \n"  // accumulate into motion counter

                "add x0, x0, #16             \n"  // increment byte offset by 16
                "cmp x0, %[size]             \n"  // compare with simd_size
                "b.lt 1b                     \n"  // branch if less than (continue loop)

                "addv s9, v1.4s              \n"  // horizontal add all 4 lanes of v1
                "mov %w[result], v9.s[0]     \n"  // extract 32-bit result to motion_count

                : [result] "=r" (motion_count)                                    // output
                : [curr] "r" (current), [prev] "r" (previous),                   // input pointers
                  [thresh] "r" (threshold), [size] "r" ((long)simd_size)         // input values
                : "x0", "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "memory"  // clobbered
            );
        }

        // Handle remaining pixels (if any) with scalar code
        for (int i = simd_size; i < total_pixels; i++) {
            int diff = abs(current[i] - previous[i]);
            if (diff > threshold) {
                motion_count++;
            }
        }

        return motion_count;
    }

    int compareFramesNEON(unsigned char* current, unsigned char* previous, int total_pixels) {
        #define USE_ASM_VERSION 1

        #if USE_ASM_VERSION
            return compareFramesNEON_ASM(current, previous, total_pixels);
        #else
            return compareFramesNEON_C(current, previous, total_pixels);
        #endif
    }
#endif

    int compareFramesScalar(unsigned char* current, unsigned char* previous, int total_pixels) {
        int motion_count = 0;
        for (int i = 0; i < total_pixels; i++) {
            int diff = abs(current[i] - previous[i]);
            if (diff > threshold_) {
                motion_count++;
            }
        }
        return motion_count;
    }
};

class TcpClient {
public:
    TcpClient(const std::string &server_ip, int server_port)
        : server_ip_(server_ip), server_port_(server_port), sockfd_(-1) {}
    ~TcpClient() { disconnect(); }

    bool connect() {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);
        if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid server address" << std::endl;
            close(sockfd_);
            return false;
        }

        if (::connect(sockfd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to connect to server: " << strerror(errno) << std::endl;
            close(sockfd_);
            return false;
        }

        std::cout << "Connected to server " << server_ip_ << ":" << server_port_ << std::endl;
        return true;
    }
    void disconnect() {
        if (sockfd_ >= 0) {
            close(sockfd_);
            sockfd_ = -1;
        }
    }
    bool sendData(const std::vector<unsigned char> &data) {
        if (sockfd_ < 0) {
            std::cerr << "Socket not connected" << std::endl;
            return false;
        }

        ssize_t total_sent = 0;
        ssize_t data_size = data.size();
        while (total_sent < data_size) {
            ssize_t sent = ::send(sockfd_, data.data() + total_sent, data_size - total_sent, 0);
            if (sent < 0) {
                std::cerr << "Failed to send data" << std::endl;
                return false;
            }
            total_sent += sent;
        }

        return true;
    }
private:
    std::string server_ip_;
    int server_port_;
    int sockfd_;
};

class JpegEncoder {
private:
    tjhandle tj_;
    int quality_;
    std::vector<unsigned char> raw_data_;
    int width_;
    int height_;
    int jpeg_worst_size_;
    unsigned char* jpeg_data_;
    unsigned long jpeg_size_;
public:
    JpegEncoder(int width, int height, int quality = 85)
        : tj_(nullptr), quality_(quality), raw_data_(width * height * 3 / 2),
          jpeg_worst_size_(0), jpeg_data_(nullptr), jpeg_size_(0) {
        width_ = width;
        height_ = height;

        jpeg_worst_size_ = tjBufSize(width, height, TJSAMP_420);
        if (jpeg_worst_size_ == 0) {
            throw std::runtime_error("Error calculating worst case JPEG size");
        }

        jpeg_data_ = (unsigned char*)tjAlloc(jpeg_worst_size_);
        if (!jpeg_data_) {
            throw std::runtime_error("Failed to allocate JPEG buffer");
        }

        tj_ = tjInitCompress();
        if (!tj_) {
            tjFree(jpeg_data_);
            throw std::runtime_error("Error initializing TurboJPEG");
        }
    }

    ~JpegEncoder() {
        if (jpeg_data_) {
            tjFree(jpeg_data_);
        }
        if (tj_) {
            tjDestroy(tj_);
        }
    }

    bool encode(const unsigned char* yuv_data, size_t yuv_size, std::vector<unsigned char>& out_jpeg) {
        size_t expected_size = width_ * height_ * 3 / 2;
        if (yuv_size < expected_size) {
            std::cerr << "Input YUV data is too small for " << width_ << "x" << height_ << std::endl;
            std::cerr << "Expected: " << expected_size << " bytes, got: " << yuv_size << " bytes" << std::endl;
            return false;
        }

        jpeg_size_ = jpeg_worst_size_;

        int ret = tjCompressFromYUV(tj_, 
                    yuv_data, 
                    width_, 
                    1, 
                    height_, 
                    TJSAMP_420,     // YUV420 format
                    &jpeg_data_, // Pass the address of buffer allocated for JPEG data
                    &jpeg_size_,  // Will be updated with actual size
                    quality_,           // Quality
                    TJFLAG_FASTDCT | TJFLAG_NOREALLOC);

        if (ret != 0) {
            std::cerr << "TurboJPEG compression error: " << tjGetErrorStr() << std::endl;
            return false;
        }

        out_jpeg.resize(jpeg_size_);
        std::memcpy(out_jpeg.data(), jpeg_data_, jpeg_size_);
        return true;
    }
};

class H264Encoder {
private:
    int encoder_fd_;
    int width_, height_;
    int gop_size_;  // GOP size (keyframe interval)
    int bitrate_;   // Bitrate in bps
    bool initialized_;

    // V4L2 M2M structures
    struct v4l2_format input_format_;
    struct v4l2_format output_format_;
    struct v4l2_requestbuffers input_reqbufs_;
    struct v4l2_requestbuffers output_reqbufs_;
    bool formats_set_;
    bool streaming_started_;
    bool first_frame_;

    // Output buffer management
    void* output_mem_;
    size_t output_buffer_size_;

public:
    H264Encoder(int width, int height, int gop_size = 30, int bitrate = 10000000)
        : encoder_fd_(-1), width_(width), height_(height), gop_size_(gop_size), bitrate_(bitrate),
          initialized_(false), formats_set_(false),
          streaming_started_(false), first_frame_(true), output_mem_(nullptr), output_buffer_size_(0) {
        memset(&input_format_, 0, sizeof(input_format_));
        memset(&output_format_, 0, sizeof(output_format_));
        memset(&input_reqbufs_, 0, sizeof(input_reqbufs_));
        memset(&output_reqbufs_, 0, sizeof(output_reqbufs_));
    }

    ~H264Encoder() {
        if (streaming_started_) {
            // Stop streaming
            int input_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            int output_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &input_type);
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &output_type);
        }

        if (output_mem_) {
            munmap(output_mem_, output_buffer_size_);
        }

        if (encoder_fd_ >= 0) {
            close(encoder_fd_);
        }
    }

    bool initialize() {
        std::cout << "H264Encoder::initialize() called" << std::endl;
        // Try to find V4L2 H.264 encoder device
        for (int i = 0; i < 40; i++) {
            std::string device = "/dev/video" + std::to_string(i);
            int fd = open(device.c_str(), O_RDWR);
            if (fd < 0) continue;

            struct v4l2_capability cap;
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
                // Check if this is a memory-to-memory encoder
                if ((cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) ||
                    (cap.capabilities & V4L2_CAP_VIDEO_M2M)) {

                    // Check if it supports H.264 encoding
                    struct v4l2_fmtdesc fmt;
                    memset(&fmt, 0, sizeof(fmt));
                    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

                    bool h264_found = false;
                    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
                        if (fmt.pixelformat == V4L2_PIX_FMT_H264) {
                            h264_found = true;
                            break;
                        }
                        fmt.index++;
                    }

                    if (h264_found) {
                        encoder_fd_ = fd;
                        std::cout << "Found H.264 encoder: " << device << std::endl;
                        break;
                    }
                }
            }
            close(fd);
        }

        if (encoder_fd_ < 0) {
            std::cerr << "No H.264 encoder device found" << std::endl;
            return false;
        }

        initialized_ = true;
        setupEncoder();
        return forceKeyFrame();
    }

private:
    bool setupEncoder() {
        std::cout << "Setting up V4L2 M2M H.264 encoder..." << std::endl;

        // Set input format (YU12 single plane)
        input_format_.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        input_format_.fmt.pix_mp.width = width_;
        input_format_.fmt.pix_mp.height = height_;
        input_format_.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;  // YU12
        input_format_.fmt.pix_mp.field = V4L2_FIELD_NONE;
        input_format_.fmt.pix_mp.num_planes = 1;  // Single plane for YU12

        // Single plane size for Y+U+V data
        input_format_.fmt.pix_mp.plane_fmt[0].sizeimage = width_ * height_ * 3 / 2;  // Y + U + V
        input_format_.fmt.pix_mp.plane_fmt[0].bytesperline = width_;  // Y plane width

        if (ioctl(encoder_fd_, VIDIOC_S_FMT, &input_format_) < 0) {
            std::cerr << "Failed to set input format: " << strerror(errno) << std::endl;
            return false;
        }
        std::cout << "Input format set successfully" << std::endl;

        // Set output format (H.264)
        output_format_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_format_.fmt.pix_mp.width = width_;
        output_format_.fmt.pix_mp.height = height_;
        output_format_.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
        output_format_.fmt.pix_mp.field = V4L2_FIELD_NONE;
        output_format_.fmt.pix_mp.num_planes = 1;
        output_format_.fmt.pix_mp.plane_fmt[0].sizeimage = width_ * height_;

        if (ioctl(encoder_fd_, VIDIOC_S_FMT, &output_format_) < 0) {
            std::cerr << "Failed to set output format: " << strerror(errno) << std::endl;
            return false;
        }
        std::cout << "Output format set successfully" << std::endl;

        // Set GOP size (keyframe interval)
        if (!setGopSize(gop_size_)) {
            std::cerr << "Failed to set GOP size, continuing with default" << std::endl;
        }

        // Set bitrate (default VBR mode)
        if (!setBitrate(bitrate_)) {
            std::cerr << "Failed to set bitrate, continuing with default" << std::endl;
        }

        formats_set_ = true;
        std::cout << "V4L2 M2M H.264 encoder setup completed (GOP size: " << gop_size_ << ")" << std::endl;
        return true;
    }

    bool startStreaming() {
        std::cout << "H264Encoder::startStreaming() called" << std::endl;
        if (streaming_started_) {
            std::cout << "Streaming already started" << std::endl;
            return true;  // Already started
        }

        // Request input buffers (DMABUF type for zero-copy)
        input_reqbufs_.count = 1;
        input_reqbufs_.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        input_reqbufs_.memory = V4L2_MEMORY_DMABUF;

        if (ioctl(encoder_fd_, VIDIOC_REQBUFS, &input_reqbufs_) < 0) {
            std::cerr << "Failed to request input buffers: " << strerror(errno) << std::endl;
            return false;
        }
        std::cout << "Input DMABUF buffers requested successfully" << std::endl;

        // Request and setup output buffers (MMAP type for H.264 output)
        output_reqbufs_.count = 1;
        output_reqbufs_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_reqbufs_.memory = V4L2_MEMORY_MMAP;

        if (ioctl(encoder_fd_, VIDIOC_REQBUFS, &output_reqbufs_) < 0) {
            std::cerr << "Failed to request output buffers: " << strerror(errno) << std::endl;
            return false;
        }

        // Query and map output buffer
        struct v4l2_buffer output_buf;
        struct v4l2_plane output_planes[1];
        memset(&output_buf, 0, sizeof(output_buf));
        memset(output_planes, 0, sizeof(output_planes));

        output_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_buf.memory = V4L2_MEMORY_MMAP;
        output_buf.index = 0;
        output_buf.m.planes = output_planes;
        output_buf.length = 1;

        if (ioctl(encoder_fd_, VIDIOC_QUERYBUF, &output_buf) < 0) {
            std::cerr << "Failed to query output buffer: " << strerror(errno) << std::endl;
            return false;
        }

        output_buffer_size_ = output_planes[0].length;
        output_mem_ = mmap(NULL, output_buffer_size_, PROT_READ | PROT_WRITE, MAP_SHARED, encoder_fd_, output_planes[0].m.mem_offset);
        if (output_mem_ == MAP_FAILED) {
            std::cerr << "Failed to mmap output buffer: " << strerror(errno) << std::endl;
            return false;
        }

        // Queue output buffer
        if (ioctl(encoder_fd_, VIDIOC_QBUF, &output_buf) < 0) {
            std::cerr << "Failed to queue output buffer: " << strerror(errno) << std::endl;
            munmap(output_mem_, output_buffer_size_);
            return false;
        }

        // Start streaming
        int input_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        int output_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        if (ioctl(encoder_fd_, VIDIOC_STREAMON, &input_type) < 0) {
            std::cerr << "Failed to start input streaming: " << strerror(errno) << std::endl;
            munmap(output_mem_, output_buffer_size_);
            return false;
        }

        if (ioctl(encoder_fd_, VIDIOC_STREAMON, &output_type) < 0) {
            std::cerr << "Failed to start output streaming: " << strerror(errno) << std::endl;
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &input_type);
            munmap(output_mem_, output_buffer_size_);
            return false;
        }

        streaming_started_ = true;
        std::cout << "V4L2 streaming started successfully" << std::endl;

        // Set GOP size after streaming is started
        if (!setGopSize(gop_size_)) {
            std::cerr << "Failed to set GOP size after streaming start" << std::endl;
        }

        // Enable SPS/PPS repeat with each I-frame for better streaming compatibility
        if (!enableSPSPPSRepeat(true)) {
            std::cerr << "Failed to enable SPS/PPS repeat after streaming start" << std::endl;
        }

        return true;
    }

public:
    // Zero-copy encoding using DMA file descriptor
    bool encodeDMA(int dma_fd, std::vector<unsigned char>& h264_data, bool& is_keyframe) {
        if (!initialized_) {
            std::cerr << "H.264 encoder not initialized" << std::endl;
            return false;
        }

        // Start streaming on first call (lazy initialization)
        if (!streaming_started_) {
            std::cout << "Starting streaming on first encodeDMA call..." << std::endl;
            if (!startStreaming()) {
                std::cerr << "Failed to start V4L2 streaming" << std::endl;
                return false;
            }
        }

        // Queue input buffer with DMABUF fd (single plane YU12)
        struct v4l2_buffer input_buf;
        struct v4l2_plane input_planes[1];
        memset(&input_buf, 0, sizeof(input_buf));
        memset(input_planes, 0, sizeof(input_planes));

        input_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        input_buf.memory = V4L2_MEMORY_DMABUF;
        input_buf.index = 0;
        input_buf.m.planes = input_planes;
        input_buf.length = 1;  // Single plane

        // Set DMABUF fd for single plane YU12 (Y+U+V data in sequence)
        size_t expected_size = width_ * height_ * 3 / 2;
        input_planes[0].m.fd = dma_fd;
        input_planes[0].data_offset = 0;
        input_planes[0].bytesused = expected_size;  // YU12 size
        input_planes[0].length = expected_size;     // Match format setup

        if (ioctl(encoder_fd_, VIDIOC_QBUF, &input_buf) < 0) {
            std::cerr << "Failed to queue input buffer: " << strerror(errno) << std::endl;
            return false;
        }

        // Wait for encoding completion using poll
        struct pollfd pfd;
        pfd.fd = encoder_fd_;
        pfd.events = POLLIN;

        int poll_result = poll(&pfd, 1, 1000); // 1 second timeout
        if (poll_result <= 0) {
            std::cerr << "Encoding timeout or error" << std::endl;
            return false;
        }

        // Dequeue input buffer to free it for reuse
        if (ioctl(encoder_fd_, VIDIOC_DQBUF, &input_buf) < 0) {
            std::cerr << "Failed to dequeue input buffer: " << strerror(errno) << std::endl;
            return false;
        }

        // Dequeue output buffer to get encoded H.264 data
        struct v4l2_buffer output_buf;
        struct v4l2_plane output_planes[1];
        memset(&output_buf, 0, sizeof(output_buf));
        memset(output_planes, 0, sizeof(output_planes));

        output_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_buf.memory = V4L2_MEMORY_MMAP;
        output_buf.m.planes = output_planes;
        output_buf.length = 1;

        if (ioctl(encoder_fd_, VIDIOC_DQBUF, &output_buf) < 0) {
            std::cerr << "Failed to dequeue output buffer: " << strerror(errno) << std::endl;
            return false;
        }

        // Detect keyframe (IDR frame)
        is_keyframe = !!(output_buf.flags & V4L2_BUF_FLAG_KEYFRAME);

        // Copy H.264 data from pre-mapped output buffer
        size_t h264_size = output_planes[0].bytesused;
        h264_data.resize(h264_size);
        memcpy(h264_data.data(), output_mem_, h264_size);

        // Re-queue output buffer for next frame
        if (ioctl(encoder_fd_, VIDIOC_QBUF, &output_buf) < 0) {
            std::cerr << "Failed to re-queue output buffer: " << strerror(errno) << std::endl;
            return false;
        }

        return true;
    }

    bool setGopSize(int gop_size) {
        gop_size_ = gop_size;

        if (!initialized_) {
            std::cout << "GOP size will be set to " << gop_size << " during initialization" << std::endl;
            return true;
        }

        struct v4l2_ext_controls ext_ctrls;
        struct v4l2_ext_control ext_ctrl;

        memset(&ext_ctrls, 0, sizeof(ext_ctrls));
        memset(&ext_ctrl, 0, sizeof(ext_ctrl));

        ext_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::GOP_SIZE);
        ext_ctrl.value = gop_size;

        ext_ctrls.count = 1;
        ext_ctrls.controls = &ext_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_S_EXT_CTRLS, &ext_ctrls) < 0) {
            std::cerr << "Failed to set GOP size: " << strerror(errno) << std::endl;
            return false;
        }

        // Verify the setting was applied
        struct v4l2_ext_controls verify_ctrls;
        struct v4l2_ext_control verify_ctrl;
        memset(&verify_ctrls, 0, sizeof(verify_ctrls));
        memset(&verify_ctrl, 0, sizeof(verify_ctrl));

        verify_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::GOP_SIZE);
        verify_ctrls.count = 1;
        verify_ctrls.controls = &verify_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_G_EXT_CTRLS, &verify_ctrls) == 0) {
            std::cout << "GOP size set to " << gop_size << " frames, verified value: " << verify_ctrl.value << std::endl;
        } else {
            std::cout << "GOP size set to " << gop_size << " frames (verification failed)" << std::endl;
        }

        return true;
    }

    bool forceKeyFrame() {
        if (!initialized_) {
            std::cerr << "H.264 encoder not initialized" << std::endl;
            return false;
        }

        struct v4l2_ext_controls force_ctrls;
        struct v4l2_ext_control force_ctrl;
        memset(&force_ctrls, 0, sizeof(force_ctrls));
        memset(&force_ctrl, 0, sizeof(force_ctrl));

        force_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::FORCE_KEY_FRAME);
        force_ctrl.value = 1;        // Any value triggers the action

        force_ctrls.count = 1;
        force_ctrls.controls = &force_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_S_EXT_CTRLS, &force_ctrls) < 0) {
            std::cerr << "Failed to force key frame: " << strerror(errno) << std::endl;
            return false;
        }

        std::cout << "Forced I-frame generation" << std::endl;
        return true;
    }

    bool enableSPSPPSRepeat(bool enable = true) {
        if (!initialized_) {
            std::cout << "SPS/PPS repeat will be " << (enable ? "enabled" : "disabled") << " during initialization" << std::endl;
            return true;
        }

        struct v4l2_ext_controls ext_ctrls;
        struct v4l2_ext_control ext_ctrl;

        memset(&ext_ctrls, 0, sizeof(ext_ctrls));
        memset(&ext_ctrl, 0, sizeof(ext_ctrl));

        ext_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::REPEAT_SEQ_HEADER);
        ext_ctrl.value = enable ? 1 : 0;

        ext_ctrls.count = 1;
        ext_ctrls.controls = &ext_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_S_EXT_CTRLS, &ext_ctrls) < 0) {
            std::cerr << "Failed to set SPS/PPS repeat: " << strerror(errno) << std::endl;
            return false;
        }

        // Verify the setting was applied
        struct v4l2_ext_controls verify_ctrls;
        struct v4l2_ext_control verify_ctrl;
        memset(&verify_ctrls, 0, sizeof(verify_ctrls));
        memset(&verify_ctrl, 0, sizeof(verify_ctrl));

        verify_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::REPEAT_SEQ_HEADER);
        verify_ctrls.count = 1;
        verify_ctrls.controls = &verify_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_G_EXT_CTRLS, &verify_ctrls) == 0) {
            std::cout << "SPS/PPS repeat " << (enable ? "enabled" : "disabled") << ", verified value: " << verify_ctrl.value << std::endl;
        } else {
            std::cout << "SPS/PPS repeat " << (enable ? "enabled" : "disabled") << " (verification failed)" << std::endl;
        }

        return true;
    }

    bool setBitrate(int bitrate, bool cbr = false) {
        bitrate_ = bitrate;

        if (!initialized_) {
            std::cout << "Bitrate will be set to " << bitrate << " bps during initialization" << std::endl;
            return true;
        }

        // Set bitrate mode first (VBR=0, CBR=1)
        struct v4l2_ext_controls mode_ctrls;
        struct v4l2_ext_control mode_ctrl;
        memset(&mode_ctrls, 0, sizeof(mode_ctrls));
        memset(&mode_ctrl, 0, sizeof(mode_ctrl));

        mode_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::BITRATE_MODE);
        mode_ctrl.value = cbr ? 1 : 0;  // 0=VBR, 1=CBR 

        mode_ctrls.count = 1;
        mode_ctrls.controls = &mode_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_S_EXT_CTRLS, &mode_ctrls) < 0) {
            std::cerr << "Failed to set bitrate mode: " << strerror(errno) << std::endl;
            return false;
        }

        // Set bitrate value
        struct v4l2_ext_controls bitrate_ctrls;
        struct v4l2_ext_control bitrate_ctrl;
        memset(&bitrate_ctrls, 0, sizeof(bitrate_ctrls));
        memset(&bitrate_ctrl, 0, sizeof(bitrate_ctrl));

        bitrate_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::BITRATE);
        bitrate_ctrl.value = bitrate;

        bitrate_ctrls.count = 1;
        bitrate_ctrls.controls = &bitrate_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_S_EXT_CTRLS, &bitrate_ctrls) < 0) {
            std::cerr << "Failed to set bitrate: " << strerror(errno) << std::endl;
            return false;
        }

        // Verify the setting was applied
        struct v4l2_ext_controls verify_ctrls;
        struct v4l2_ext_control verify_ctrl;
        memset(&verify_ctrls, 0, sizeof(verify_ctrls));
        memset(&verify_ctrl, 0, sizeof(verify_ctrl));

        verify_ctrl.id = static_cast<uint32_t>(V4L2Controls::VideoEncoder::BITRATE);
        verify_ctrls.count = 1;
        verify_ctrls.controls = &verify_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_G_EXT_CTRLS, &verify_ctrls) == 0) {
            std::cout << "Bitrate set to " << bitrate << " bps (" << (cbr ? "CBR" : "VBR")
                      << "), verified value: " << verify_ctrl.value << std::endl;
        } else {
            std::cout << "Bitrate set to " << bitrate << " bps (verification failed)" << std::endl;
        }

        return true;
    }
};

// Transport layer classes

// TCP Broadcaster - accepts up to N clients and broadcasts data to all connected clients
class TcpBroadcaster {
private:
    int server_fd_;                    // Server socket file descriptor
    int server_port_;                  // Server port to listen on
    std::vector<int> client_sockets_;  // List of connected client sockets
    mutable std::mutex clients_mutex_; // Mutex to protect client_sockets_ from race conditions (mutable for const methods)
    int max_clients_;                  // Maximum number of simultaneous clients
    bool running_;                     // Flag to control accept thread
    std::thread accept_thread_;        // Thread for accepting new client connections
    bool first_frame_;                 // Track first frame for initial burst transmission

    // Keyframe caching for new clients
    std::vector<unsigned char> cached_keyframe_;  // Cached IDR frame (includes SPS/PPS if inline_headers enabled)
    mutable std::mutex keyframe_mutex_;           // Mutex to protect keyframe cache
    bool has_keyframe_;                           // Do we have a cached keyframe?

public:
    TcpBroadcaster(int server_port, int max_clients = 5)
        : server_fd_(-1), server_port_(server_port), max_clients_(max_clients),
          running_(false), first_frame_(true), has_keyframe_(false) {

        // Create server socket
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            std::cerr << "Failed to create server socket" << std::endl;
            return;
        }

        // Set socket options: allow address reuse (useful for quick restart)
        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "Failed to set SO_REUSEADDR" << std::endl;
        }

        // Bind socket to port
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);
        server_addr.sin_addr.s_addr = INADDR_ANY;  // Accept connections on any interface

        if (bind(server_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to bind socket to port " << server_port_
                     << ": " << strerror(errno) << std::endl;
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        // Start listening for connections
        if (listen(server_fd_, max_clients_) < 0) {
            std::cerr << "Failed to listen on socket: " << strerror(errno) << std::endl;
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        // Start accept thread
        running_ = true;
        accept_thread_ = std::thread(&TcpBroadcaster::acceptClients, this);

        std::cout << "TCP broadcaster started on port " << server_port_
                  << " (max clients: " << max_clients_ << ")" << std::endl;
    }

    ~TcpBroadcaster() {
        // Stop accept thread
        running_ = false;
        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }

        // Close all client connections
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int client_fd : client_sockets_) {
            close(client_fd);
        }
        client_sockets_.clear();

        // Close server socket
        if (server_fd_ >= 0) {
            close(server_fd_);
        }

        std::cout << "TCP broadcaster stopped" << std::endl;
    }

    // Send data to all connected clients
    void send(const std::vector<unsigned char>& data, bool is_keyframe = false) {
        // Cache keyframe for new clients (contains SPS/PPS + IDR if inline_headers enabled)
        if (is_keyframe) {
            std::lock_guard<std::mutex> kf_lock(keyframe_mutex_);
            cached_keyframe_ = data;
            has_keyframe_ = true;
        }

        std::lock_guard<std::mutex> lock(clients_mutex_);

        // No clients connected - skip transmission
        if (client_sockets_.empty()) {
            return;
        }

        // Determine how many times to send (first frame gets sent multiple times)
        // int send_count = first_frame_ ? 3 : 1;
        int send_count = 1; // Disabled repeated sending for first frame to reduce bandwidth

        // Broadcast data to all connected clients
        for (int repeat = 0; repeat < send_count; ++repeat) {
            auto it = client_sockets_.begin();
            while (it != client_sockets_.end()) {
                int client_fd = *it;

                // Send data to this client
                ssize_t total_sent = 0;
                ssize_t data_size = data.size();
                bool failed = false;

                while (total_sent < data_size) {
                    // MSG_NOSIGNAL prevents SIGPIPE when client disconnects
                    ssize_t sent = ::send(client_fd, data.data() + total_sent,
                                         data_size - total_sent, MSG_NOSIGNAL);
                    if (sent < 0) {
                        std::cerr << "Failed to send to client fd=" << client_fd
                                 << ", disconnecting: " << strerror(errno) << std::endl;
                        close(client_fd);
                        it = client_sockets_.erase(it);
                        failed = true;
                        break;
                    }
                    total_sent += sent;
                }

                // Move to next client only if send succeeded
                if (!failed) {
                    ++it;
                }
            }
        }

        // Mark first frame as sent
        if (first_frame_) {
            std::cout << "First frame sent " << send_count
                     << " times to " << client_sockets_.size() << " client(s)" << std::endl;
            first_frame_ = false;
        }
    }

    // Get number of currently connected clients
    int getClientCount() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return client_sockets_.size();
    }

private:
    // Thread function to accept new client connections
    void acceptClients() {
        if (server_fd_ < 0) {
            std::cerr << "Accept thread: invalid server socket" << std::endl;
            return;
        }

        while (running_) {
            // Use poll to check for incoming connections with timeout
            struct pollfd pfd;
            pfd.fd = server_fd_;
            pfd.events = POLLIN;

            int poll_ret = poll(&pfd, 1, 1000); // 1 second timeout
            if (poll_ret <= 0) {
                // Timeout or error - continue loop to check running_ flag
                continue;
            }

            // Accept new client connection
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);

            if (client_fd < 0) {
                if (running_) {  // Only log if we're still running (not shutting down)
                    std::cerr << "Failed to accept connection: " << strerror(errno) << std::endl;
                }
                continue;
            }

            // Get client IP address for logging
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);

            // Check if we've reached max clients
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);

                if (client_sockets_.size() >= static_cast<size_t>(max_clients_)) {
                    std::cerr << "Max clients (" << max_clients_
                             << ") reached, rejecting connection from " << client_ip << std::endl;
                    close(client_fd);
                    continue;
                }

                // Add client to list
                client_sockets_.push_back(client_fd);
                std::cout << "Client connected from " << client_ip
                         << " (fd=" << client_fd << ", total clients: "
                         << client_sockets_.size() << "/" << max_clients_ << ")" << std::endl;

                // Send cached keyframe to new client IMMEDIATELY (3 times for reliability)
                // This allows the client to start decoding right away
                {
                    std::lock_guard<std::mutex> kf_lock(keyframe_mutex_);
                    if (has_keyframe_) {
                        for (int i = 0; i < 3; i++) {
                            ssize_t total_sent = 0;
                            ssize_t data_size = cached_keyframe_.size();

                            while (total_sent < data_size) {
                                ssize_t sent = ::send(client_fd, cached_keyframe_.data() + total_sent,
                                                     data_size - total_sent, MSG_NOSIGNAL);
                                if (sent < 0) {
                                    std::cerr << "Failed to send cached keyframe to new client (attempt "
                                             << (i+1) << ")" << std::endl;
                                    break;
                                }
                                total_sent += sent;
                            }

                            if (total_sent == data_size) {
                                std::cout << "Sent cached keyframe to new client (attempt " << (i+1)
                                         << ", " << cached_keyframe_.size() << " bytes)" << std::endl;
                            }
                        }
                    }
                }
            }
        }

        std::cout << "Accept thread terminated" << std::endl;
    }
};

class TcpBroadcastSender {
private:
    std::unique_ptr<TcpBroadcaster> broadcaster_;

public:
    TcpBroadcastSender(int server_port, int max_clients = 5) {
        broadcaster_ = std::make_unique<TcpBroadcaster>(server_port, max_clients);
    }

    void send(const std::vector<unsigned char>& data, bool is_keyframe = false) {
        if (broadcaster_) {
            broadcaster_->send(data, is_keyframe);
        }
    }
};

class TelegramSender {
private:
    std::unique_ptr<TelegramBotApi> bot_api_;
    std::string chat_id_;

    // Async queue with frame dropping
    std::queue<std::vector<unsigned char>> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};

    size_t max_queue_size_ = 5;  // Drop oldest frames if queue grows beyond this
    int frame_counter_ = 0;

    // Worker thread loop - processes frames from queue
    void workerLoop() {
        while (running_) {
            std::vector<unsigned char> frame_data;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                    return !frame_queue_.empty() || !running_;
                });

                if (!running_ && frame_queue_.empty()) {
                    break;
                }

                if (!frame_queue_.empty()) {
                    frame_data = std::move(frame_queue_.front());
                    frame_queue_.pop();
                }
            }

            if (!frame_data.empty()) {
                // Send to Telegram with retry logic
                bool success = bot_api_->sendPhoto(chat_id_, frame_data);
                if (success) {
                    std::cout << "Telegram: Frame " << frame_counter_++ << " sent successfully" << std::endl;
                } else {
                    std::cerr << "Telegram: Failed to send frame " << frame_counter_ << std::endl;
                }
            }
        }

        std::cout << "Telegram worker thread stopped" << std::endl;
    }

public:
    TelegramSender(const std::string& chat_id) : chat_id_(chat_id) {
        // Read bot token from environment
        const char* token = std::getenv("TELEGRAM_BOT_TOKEN");
        if (!token) {
            throw std::runtime_error("TELEGRAM_BOT_TOKEN environment variable not set");
        }

        bot_api_ = std::make_unique<TelegramBotApi>(token);
        std::cout << "TelegramSender initialized (chat_id: " << chat_id_ << ")" << std::endl;
    }

    ~TelegramSender() {
        stop();
    }

    void start() {
        if (!running_) {
            running_ = true;
            worker_thread_ = std::thread(&TelegramSender::workerLoop, this);
            std::cout << "Telegram worker thread started" << std::endl;
        }
    }

    void stop() {
        if (running_) {
            running_ = false;
            queue_cv_.notify_all();
            if (worker_thread_.joinable()) {
                worker_thread_.join();
            }
            std::cout << "Telegram sender stopped" << std::endl;
        }
    }

    void send(const std::vector<unsigned char>& data, bool is_keyframe = false) {
        // Throttling is now handled in CapturerV2::processingFrameBuffer (before JPEG encoding)

        std::lock_guard<std::mutex> lock(queue_mutex_);

        // Drop oldest frame if queue is full
        if (frame_queue_.size() >= max_queue_size_) {
            frame_queue_.pop();
            std::cout << "Telegram: Queue full, dropped oldest frame" << std::endl;
        }

        frame_queue_.push(data);
        queue_cv_.notify_one();
    }
};

class TcpSender {
private:
    std::unique_ptr<TcpClient> tcp_client_;
    bool first_frame_ = true;

public:
    TcpSender(const std::string& server_ip, int server_port) {
        tcp_client_ = std::make_unique<TcpClient>(server_ip, server_port);
        if (!tcp_client_->connect()) {
            std::cerr << "Warning: Failed to connect to TCP server" << std::endl;
        }
        std::cout << "TCP sender initialized successfully" << std::endl;
    }

    void send(const std::vector<unsigned char>& data, bool is_keyframe = false) {
        // Note: is_keyframe is ignored for TcpSender (client mode)
        // Keyframe caching is only used in TcpBroadcaster (server mode)
        bool success = tcp_client_->sendData(data);
        if (!success) {
            std::cerr << "Failed to send data via TCP" << std::endl;
            return;
        }
        if (first_frame_) {
            // Send first frame multiple times for better streaming start
            for (int i = 0; i < 3; ++i) {
                success = tcp_client_->sendData(data);
            }
            if (!success) {
                std::cerr << "Failed to send first frame multiple times via TCP" << std::endl;
            } else {
                std::cout << "First frame sent multiple times via TCP for better start" << std::endl;
            }
            first_frame_ = false;
        }
    }
};

class FileSender {
private:
    std::string base_path_;
    std::string extension_;
    int frame_counter_ = 0;

public:
    FileSender(const std::string& path, const std::string& ext = ".dat")
        : base_path_(path), extension_(ext) {}

    void send(const std::vector<unsigned char>& data, bool is_keyframe = false) {
        // Note: is_keyframe can be used to save only keyframes if desired
        std::string filename = base_path_ + "/frame_" + std::to_string(frame_counter_++) + extension_;
        std::ofstream file(filename, std::ios::binary);
        file.write(reinterpret_cast<const char*>(data.data()), data.size());
    }
};

// Frame handler layer classes
template<typename Sender>
class MjpegFrameHandler {
private:
    std::shared_ptr<Sender> sender_;

public:
    explicit MjpegFrameHandler(std::shared_ptr<Sender> sender) : sender_(sender) {}

    void operator()(const std::vector<unsigned char>& mjpeg_data) {
        sender_->send(mjpeg_data);
    }
};

template<typename Sender>
class H264FrameHandler {
private:
    std::shared_ptr<Sender> sender_;

public:
    explicit H264FrameHandler(std::shared_ptr<Sender> sender) : sender_(sender) {}

    void operator()(const std::vector<unsigned char>& h264_data, bool is_keyframe = false) {
        sender_->send(h264_data, is_keyframe);
    }
};

template<typename MjpegHandler, typename H264Handler>
class CapturerV2 {
private:
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;

    // Frame handlers
    MjpegHandler mjpeg_handler_;
    H264Handler h264_handler_;

    // Encoders
    std::unique_ptr<JpegEncoder> jpeg_encoder_;
    std::unique_ptr<H264Encoder> h264_encoder_;
    bool use_h264_ = false;

    // Output format control
    bool use_mjpeg_ = true;
    bool use_h264_output_ = false;

    // Motion detection
    std::unique_ptr<MotionDetector> motion_detector_;

    // Motion tail recording (continue for 5 seconds after motion stops)
    std::chrono::steady_clock::time_point last_motion_time_;
    std::chrono::seconds motion_tail_duration_{5}; // Continue recording for 5 seconds after motion stops

    // Statistics
    int frames_captured_ = 0;
    bool capture_running_ = false;

    // Frame rate settings
    int current_fps_ = 30;     // Current FPS setting
    std::chrono::steady_clock::time_point last_frame_time_;

    // Motion detection frame skip control
    int motion_frame_skip_ = 1;        // Process every N-th frame (1 = every frame)
    int motion_frame_counter_ = 0;     // Current frame counter

    // JPEG encoding throttle (for Telegram sender)
    std::chrono::steady_clock::time_point last_jpeg_encode_time_;
    std::chrono::milliseconds jpeg_encode_interval_{300};  // Minimum 300ms between JPEG encodes

    // Camera resolution settings
    int width_ = 1920;
    int height_ = 1080;

    // Autofocus settings
    struct AutofocusSettings {
        int mode = 2;           // 0=Auto, 1=Manual, 2=Continuous
        float lens_position = 0.0f;  // For manual mode
        bool trigger_scan = false;   // For triggering AF scan
        bool enable = true;          // Enable/disable AF
        int speed = 1;          // 0=Normal, 1=Fast
        int range = 0;          // 0=Normal, 1=Macro, 2=Full
        bool use_windows = false;    // Use specific AF windows
    } af_settings_;

public:
    CapturerV2(MjpegHandler mjpeg_handler, H264Handler h264_handler, int width = 1920, int height = 1080)
        : cm_(std::make_unique<CameraManager>()),
          mjpeg_handler_(std::move(mjpeg_handler)),
          h264_handler_(std::move(h264_handler)),
          last_motion_time_(std::chrono::steady_clock::now()),
          last_jpeg_encode_time_(std::chrono::steady_clock::now()),
          width_(width),
          height_(height) {

        // Initialize encoders
        jpeg_encoder_ = std::make_unique<JpegEncoder>(width_, height_, 90); // Quality=90
        h264_encoder_ = std::make_unique<H264Encoder>(width_, height_, 60, 5000000); // GOP=60, Bitrate=5Mbps

        // Initialize motion detector
        motion_detector_ = std::make_unique<MotionDetector>(width_, height_, 4); // Downsample factor 4
    }
    
    bool initialize() {
        std::cout << "Initializing camera..." << std::endl;
        
        // Start camera manager
        int ret = cm_->start();
        if (ret) {
            std::cerr << "Failed to start camera manager: " << ret << std::endl;
            return false;
        }
        
        // Check for available cameras
        if (cm_->cameras().empty()) {
            std::cerr << "No cameras found!" << std::endl;
            return false;
        }
        
        // Get first camera
        camera_ = cm_->cameras()[0];
        std::cout << "Found camera: " << camera_->id() << std::endl;
        
        // Acquire camera access
        if (camera_->acquire()) {
            std::cerr << "Failed to acquire camera" << std::endl;
            return false;
        }

        // Try to initialize H.264 encoder
        if (h264_encoder_->initialize()) {
            std::cout << "H.264 hardware encoder available" << std::endl;
        } else {
            std::cout << "H.264 hardware encoder not available, using JPEG only" << std::endl;
        }

        std::cout << "Camera initialized successfully" << std::endl;
        return true;
    }

    bool configure() {
        std::cout << "Configuring camera..." << std::endl;
        
        config_ = camera_->generateConfiguration({StreamRole::VideoRecording});
        
        if (!config_) {
            std::cerr << "Failed to create configuration" << std::endl;
            return false;
        }
        
        // Configure parameters for Zero 2W + Camera v3
        StreamConfiguration &stream_config = config_->at(0);
        stream_config.size = Size(width_, height_);  // Use configured resolution
        stream_config.pixelFormat = formats::YUV420; // Efficient format
        
        // Validate configuration
        CameraConfiguration::Status validation = config_->validate();
        if (validation == CameraConfiguration::Invalid) {
            std::cerr << "Configuration is invalid" << std::endl;
            return false;
        } else if (validation == CameraConfiguration::Adjusted) {
            std::cout << "Configuration was adjusted" << std::endl;
        }
        
        // Apply configuration
        if (camera_->configure(config_.get())) {
            std::cerr << "Failed to apply configuration" << std::endl;
            return false;
        }

        std::cout << "Configuration applied successfully" << std::endl;
        return true;
    }
    
    bool setupBuffers() {
        std::cout << "Setting up buffers..." << std::endl;
        
        // Create buffer allocator
        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
        
        // Get stream
        Stream *stream = config_->at(0).stream();
        
        // Allocate buffers
        int ret = allocator_->allocate(stream);
        if (ret < 0) {
            std::cerr << "Failed to allocate buffers" << std::endl;
            return false;
        }
        
        for (unsigned int i = 0; i < allocator_->buffers(stream).size(); ++i) {
            auto request = camera_->createRequest();
            if (!request) {
                std::cerr << "Failed to create request " << i << std::endl;
                return false;
            }
            
            const std::unique_ptr<FrameBuffer> &buffer = allocator_->buffers(stream)[i];
            int ret = request->addBuffer(stream, buffer.get());
            if (ret < 0) {
                std::cerr << "Failed to add buffer to request " << i << std::endl;
                return false;
            }

            ControlList controls = buildControls();
            request->controls() = controls;

            requests_.push_back(std::move(request));
        }
        
        std::cout << "Buffers set up successfully" << std::endl;
        return true;
    }
    
    bool startCapture() {
        std::cout << "Starting capture..." << std::endl;
        
        camera_->requestCompleted.connect(this, &CapturerV2::requestComplete);
        
        if (camera_->start()) {
            std::cerr << "Failed to start camera" << std::endl;
            return false;
        }
        
        capture_running_ = true;
        
        for (auto &request : requests_) {
            if (camera_->queueRequest(request.get())) {
                std::cerr << "Failed to queue request" << std::endl;
                return false;
            }
        }
        
        std::cout << "Capture started at " << current_fps_ << " FPS!" << std::endl;
        return true;
    }
    
    void stopCapture() {
        std::cout << "Stopping capture..." << std::endl;
        
        capture_running_ = false;
        
        if (camera_) {
            camera_->stop();
        }
        
        std::cout << "Capture stopped" << std::endl;
    }
    
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    virtual void processingFrameBuffer(const FrameBuffer *buffer) {
        size_t total_yuv_size = width_ * height_ * 3 / 2;
        void *mapped_data = mmap(nullptr, total_yuv_size, PROT_READ, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
        if (mapped_data == MAP_FAILED) {
            std::cerr << "Failed to mmap frame buffer" << std::endl;
            return;
        }
        const FrameBuffer::Plane &first_plane = buffer->planes()[0];

        // Motion detection BEFORE encoding (using Y plane only)
        bool should_record = true;
        bool motion = false;
        bool should_detect = false;
        if (motion_detector_ && first_plane.length >= 0) {
            // Frame skipping optimization - only process every N-th frame
            should_detect = (motion_frame_counter_ % motion_frame_skip_ == 0);
            motion_frame_counter_ = (motion_frame_counter_ + 1) % motion_frame_skip_;

            if (should_detect) {
                motion = motion_detector_->detectMotion(static_cast<unsigned char*>(mapped_data), first_plane.length);
            } else {
                // Use last known motion state when skipping frames
                motion = motion_detector_->isMotionDetected();
            }

            if (motion) {
                // Update last motion time
                last_motion_time_ = std::chrono::steady_clock::now();

                std::cout << "[MOTION] Detected! Level: " << std::fixed << std::setprecision(2)
                          << motion_detector_->getMotionLevel() << "%, Pixels: "
                          << motion_detector_->getMotionPixelCount() << std::endl;
            } else {
                // Check if we're still in the "tail" recording period
                auto now = std::chrono::steady_clock::now();
                auto time_since_motion = std::chrono::duration_cast<std::chrono::seconds>(now - last_motion_time_);

                if (time_since_motion > motion_tail_duration_) {
                    should_record = false;
                }
            }
        }

        // Try H.264 encoding and output if enabled
        if (use_h264_output_ && use_h264_ && h264_encoder_) {
            std::vector<unsigned char> h264_data;
            bool is_keyframe = false;

            if (h264_encoder_->encodeDMA(first_plane.fd.get(), h264_data, is_keyframe)) {
                h264_handler_(h264_data, is_keyframe);
            } else {
                std::cout << "H.264 DMA encoding failed" << std::endl;
            }
        }

        // JPEG encoding and output if enabled (with 300ms throttle to save CPU)
        // if (use_mjpeg_ && should_record) {
        if (motion && should_detect) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_jpeg_encode_time_);

            if (elapsed >= jpeg_encode_interval_) {
                last_jpeg_encode_time_ = now;

                std::vector<unsigned char> jpeg_data;
                if (jpeg_encoder_->encode(static_cast<unsigned char*>(mapped_data), total_yuv_size, jpeg_data)) {
                    mjpeg_handler_(jpeg_data);
                }
            }
        }
        munmap(mapped_data, total_yuv_size);
    }

    virtual void requestComplete(Request *req) {
        if (req->status() == Request::RequestComplete) {
            frames_captured_++;

            const Request::BufferMap &buffers = req->buffers();
            for (auto bufferPair : buffers) {
                FrameBuffer *buffer = bufferPair.second;
                processingFrameBuffer(buffer);
                break;
            }
            
            if (capture_running_) {
                req->reuse(Request::ReuseBuffers);

                // Rebuild controls for next request (autofocus, etc.)
                ControlList controls = buildControls();
                req->controls() = controls;

                camera_->queueRequest(req);
            }
        } else {
            std::cerr << "Frame capture error" << std::endl;
        }
    }
    
    void showStats() {
        std::cout << "\nStatistics:" << std::endl;
        std::cout << "   Frames captured: " << frames_captured_ << std::endl;
        std::cout << "   Status: " << (capture_running_ ? "Active" : "Stopped") << std::endl;
    }

    void setAutofocusMode(int mode) {
        af_settings_.mode = mode;
    }

    void setLensPosition(float position) {
        af_settings_.lens_position = position;
    }

    // Trigger autofocus scan (for Auto mode)
    void triggerAutofocus() {
        af_settings_.trigger_scan = true;
    }

    void enableAutofocus(bool enable) {
        af_settings_.enable = enable;
    }

    void setAutofocusSpeed(int speed) {
        af_settings_.speed = speed;
    }

    void setAutofocusRange(int range) {
        af_settings_.range = range;
    }

    void enableH264Encoding(bool enable) {
        use_h264_ = enable && h264_encoder_;
    }

    bool isH264Available() const {
        return h264_encoder_ != nullptr;
    }

    void enableMjpegOutput(bool enable) {
        use_mjpeg_ = enable;
        std::cout << "MJPEG output " << (enable ? "enabled" : "disabled") << std::endl;
    }

    void enableH264Output(bool enable) {
        use_h264_output_ = enable;
        std::cout << "H.264 output " << (enable ? "enabled" : "disabled") << std::endl;
    }

    bool isMjpegOutputEnabled() const {
        return use_mjpeg_;
    }

    bool isH264OutputEnabled() const {
        return use_h264_output_;
    }

    // Motion detection methods
    void enableMotionDetection(bool enable) {
        if (enable && !motion_detector_) {
            motion_detector_ = std::make_unique<MotionDetector>(width_, height_, 4);
        } else if (!enable) {
            motion_detector_.reset();
        }
        std::cout << "Motion detection " << (enable ? "enabled" : "disabled") << std::endl;
    }

    bool isMotionDetectionEnabled() const {
        return motion_detector_ != nullptr;
    }

    bool isMotionDetected() const {
        return motion_detector_ ? motion_detector_->isMotionDetected() : false;
    }

    float getMotionLevel() const {
        return motion_detector_ ? motion_detector_->getMotionLevel() : 0.0f;
    }

    void setMotionThreshold(float threshold) {
        if (motion_detector_) {
            motion_detector_->setThreshold(threshold);
            std::cout << "Motion threshold set to " << threshold << std::endl;
        }
    }

    void setMotionAreaThreshold(float area_threshold) {
        if (motion_detector_) {
            motion_detector_->setMotionAreaThreshold(area_threshold);
            std::cout << "Motion area threshold set to " << (area_threshold * 100) << "%" << std::endl;
        }
    }

    void setMotionFrameSkip(int skip) {
        motion_frame_skip_ = std::max(1, skip); // Minimum skip = 1 (process every frame)
        motion_frame_counter_ = 0; // Reset counter
        std::cout << "Motion detection frame skip set to " << skip
                  << " (processing every " << skip << " frame" << (skip > 1 ? "s" : "") << ")" << std::endl;
    }

    void setMotionTailDuration(int seconds) {
        motion_tail_duration_ = std::chrono::seconds(seconds);
        std::cout << "Motion tail recording duration set to " << seconds << " seconds" << std::endl;
    }

    int getMotionTailDuration() const {
        return static_cast<int>(motion_tail_duration_.count());
    }

    void setJpegEncodeInterval(int milliseconds) {
        jpeg_encode_interval_ = std::chrono::milliseconds(milliseconds);
        std::cout << "JPEG encode interval set to " << milliseconds << "ms" << std::endl;
    }

    int getJpegEncodeInterval() const {
        return static_cast<int>(jpeg_encode_interval_.count());
    }

    bool setH264GopSize(int gop_size) {
        if (!h264_encoder_) {
            std::cerr << "Set gop: H.264 encoder not available" << std::endl;
            return false;
        }
        return h264_encoder_->setGopSize(gop_size);
    }

    bool setH264Bitrate(int bitrate, bool cbr = false) {
        if (!h264_encoder_) {
            std::cerr << "Set bitrate: H.264 encoder not available" << std::endl;
            return false;
        }
        return h264_encoder_->setBitrate(bitrate, cbr);
    }

    bool setFrameRate(int fps) {
        if (fps <= 0 || fps > 120) {
            std::cerr << "Invalid FPS value: " << fps << ". Must be between 1 and 120." << std::endl;
            return false;
        }
        current_fps_ = fps;

        return true;
    }

    int getFrameRate() const {
        return current_fps_;
    }

private:
    ControlList buildControls() {
        ControlList controls;

        if (!af_settings_.enable) {
            return controls;
        }

        switch (af_settings_.mode) {
            case 0:
                controls.set(controls::AfMode, controls::AfModeAuto);
                if (af_settings_.trigger_scan) {
                    controls.set(controls::AfTrigger, controls::AfTriggerStart);
                    af_settings_.trigger_scan = false;
                }
                break;
            case 1:
                controls.set(controls::AfMode, controls::AfModeManual);
                controls.set(controls::LensPosition, af_settings_.lens_position);
                break;
            case 2:
                controls.set(controls::AfMode, controls::AfModeContinuous);
                break;
        }

        switch (af_settings_.speed) {
            case 0:
                controls.set(controls::AfSpeed, controls::AfSpeedNormal);
                break;
            case 1:
                controls.set(controls::AfSpeed, controls::AfSpeedFast);
                break;
        }

        switch (af_settings_.range) {
            case 0:
                controls.set(controls::AfRange, controls::AfRangeNormal);
                break;
            case 1:
                controls.set(controls::AfRange, controls::AfRangeMacro);
                break;
            case 2:
                controls.set(controls::AfRange, controls::AfRangeFull);
                break;
        }

        int64_t frame_time_us = 1000000 / current_fps_;
        controls.set(controls::FrameDurationLimits,
                    libcamera::Span<const int64_t, 2>({ frame_time_us, frame_time_us }));

        return controls;
    }

public:
    
    ~CapturerV2() {
        std::cout << "Capturer destructor: Cleaning up resources..." << std::endl;
        
        if (camera_) {
            if (capture_running_) {
                camera_->stop();
            }
            camera_->release();
        }
        
        if (cm_) {
            cm_->stop();
        }
        
        std::cout << "Cleanup completed" << std::endl;
    }
};

int main(int argc, char *argv[]) {
    std::cout << "Camera Module v3 + Pi Zero 2W + libcamera" << std::endl;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << "connect <server_ip> <server_port>" << std::endl;
        std::cerr << "       " << argv[0] << "accept <server_port> [max_clients]" << std::endl;
        return -1;
    }

    try {
        // Read Telegram chat ID from environment or use default
        const char* chat_id_env = std::getenv("TELEGRAM_CHAT_ID");
        std::string chat_id = chat_id_env ? chat_id_env : "YOUR_CHAT_ID";

        auto sender_mjpeg = std::make_shared<TelegramSender>(chat_id);
        sender_mjpeg->start();  // Start worker thread

        MjpegFrameHandler<TelegramSender> mjpeg_handler(sender_mjpeg);

        // if (std::string(argv[1]) != "connect" && std::string(argv[1]) != "accept") {
        //     std::cerr << "Invalid mode. Use 'connect' or 'accept'." << std::endl;
        //     return -1;
        // }
        // if (std::string(argv[1]) == "client") {
        //     TcpSender tcp_sender_h264(std::string(argv[1]), std::stoi(argv[2])); // Different port
        //     H264FrameHandler<TcpSender> h264_handler(std::move(tcp_sender_h264));
        // } else {
            int max_clients = 5;
            if (argc >= 4) {
                max_clients = std::stoi(argv[3]);
            }
            auto tcp_broadcaster = std::make_shared<TcpBroadcastSender>(std::stoi(argv[2]), max_clients);
            H264FrameHandler<TcpBroadcastSender> h264_handler(tcp_broadcaster);
        // }

        // Regular capturer (always encoding)
        CapturerV2 capturer(std::move(mjpeg_handler), std::move(h264_handler), 1920, 1080);

        if (!capturer.initialize()) {
            return -1;
        }

        if (!capturer.configure()) {
            return -1;
        }

        if (!capturer.setupBuffers()) {
            return -1;
        }

        capturer.setAutofocusMode(2);  // 0=Auto, 1=Manual, 2=Continuous
        capturer.setAutofocusSpeed(1); // 0=Normal, 1=Fast
        capturer.setAutofocusRange(2); // 0=Normal, 1=Macro, 2=Full
        capturer.enableAutofocus(true);
        capturer.setFrameRate(30);  // 30 FPS
        capturer.enableMotionDetection(true); // Enable motion detection by default
        capturer.setMotionFrameSkip(7); // Process every 7th frame
        capturer.setMotionTailDuration(0); // Record for 0 seconds after motion stops
        capturer.setJpegEncodeInterval(10);  // 10ms between JPEG encodes

        if (capturer.isH264Available()) {
            std::cout << "H.264 hardware encoder detected!" << std::endl;
            capturer.enableH264Encoding(true);
            capturer.setH264GopSize(60);  // 1 I-frame every 60 frames
            capturer.setH264Bitrate(5000000, false);  // 5Mbps VBR = false, CBR = true
        } else {
            std::cout << "H.264 hardware encoder not available, using JPEG only" << std::endl;
        }

        capturer.enableMjpegOutput(true);
        capturer.enableH264Output(true);

        if (!capturer.startCapture()) {
            return -1;
        }

        std::cout << "Press Ctrl+C to stop" << std::endl;

        // For demo purposes, run indefinitely
        for (;;) {
            sleep(1);
        }

        capturer.showStats();
        capturer.stopCapture();

    } catch (const std::exception& e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "\nProgram completed successfully!" << std::endl;
    return 0;
}
