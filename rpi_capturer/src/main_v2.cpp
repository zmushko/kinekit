#include <../../build/include/libcamera/libcamera.h>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <turbojpeg.h>
#include <vector>
#include <map>
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

// ARM NEON intrinsics for SIMD optimization
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

using namespace libcamera;

// Shared buffer pool for memory mappings with LRU eviction
class SharedBufferPool {
private:
    struct BufferMapping {
        void* mapped_data = nullptr;
        size_t size = 0;
        uint64_t last_access_time = 0;  // For LRU eviction
    };

    std::map<int, BufferMapping> buffer_mappings_;  // fd -> mapping
    static const size_t MAX_BUFFER_MAPPINGS = 12;   // Increased for shared usage
    uint64_t access_counter_ = 0;  // Monotonic counter for LRU

    // LRU eviction helper
    void evictOldestMapping() {
        if (buffer_mappings_.empty()) return;

        auto oldest_it = buffer_mappings_.begin();
        for (auto it = buffer_mappings_.begin(); it != buffer_mappings_.end(); ++it) {
            if (it->second.last_access_time < oldest_it->second.last_access_time) {
                oldest_it = it;
            }
        }

        // Cleanup the mapping
        if (oldest_it->second.mapped_data && oldest_it->second.mapped_data != MAP_FAILED) {
            munmap(oldest_it->second.mapped_data, oldest_it->second.size);
        }

        std::cout << "SharedBufferPool: Evicted mapping for fd=" << oldest_it->first
                  << " (LRU, access_time=" << oldest_it->second.last_access_time << ")" << std::endl;

        buffer_mappings_.erase(oldest_it);
    }

public:
    ~SharedBufferPool() {
        // Cleanup all mappings
        for (auto& pair : buffer_mappings_) {
            if (pair.second.mapped_data && pair.second.mapped_data != MAP_FAILED) {
                munmap(pair.second.mapped_data, pair.second.size);
            }
        }
        buffer_mappings_.clear();
    }

    // Get or create mapping for file descriptor
    void* getMapping(int dma_fd, size_t total_size, const std::string& client_name = "unknown") {
        if (dma_fd < 0 || total_size == 0) {
            return nullptr;
        }

        // Get or create mapping for this fd with LRU eviction
        auto it = buffer_mappings_.find(dma_fd);
        void* mapped_data = nullptr;

        if (it == buffer_mappings_.end() || it->second.size != total_size) {
            // Need to create new mapping
            if (it != buffer_mappings_.end() && it->second.mapped_data) {
                // Cleanup old mapping with different size
                munmap(it->second.mapped_data, it->second.size);
                buffer_mappings_.erase(it);
            }

            // Check if we need to evict oldest mapping before adding new one
            if (buffer_mappings_.size() >= MAX_BUFFER_MAPPINGS) {
                evictOldestMapping();
            }

            mapped_data = mmap(nullptr, total_size, PROT_READ, MAP_SHARED, dma_fd, 0);
            if (mapped_data == MAP_FAILED) {
                std::cerr << "SharedBufferPool: Failed to mmap for " << client_name
                          << ", fd=" << dma_fd << ": " << strerror(errno) << std::endl;
                return nullptr;
            }

            // Add new mapping with current access time
            buffer_mappings_[dma_fd] = {mapped_data, total_size, ++access_counter_};
            std::cout << "SharedBufferPool: New mapping for " << client_name
                      << ", fd=" << dma_fd << ", size=" << total_size
                      << " (total: " << buffer_mappings_.size() << ")" << std::endl;
        } else {
            // Use existing mapping and update access time
            mapped_data = it->second.mapped_data;
            it->second.last_access_time = ++access_counter_;
        }

        return mapped_data;
    }

    // Debug utility
    void printStats() const {
        std::cout << "SharedBufferPool: " << buffer_mappings_.size()
                  << "/" << MAX_BUFFER_MAPPINGS << " mappings:" << std::endl;
        for (const auto& pair : buffer_mappings_) {
            std::cout << "  fd=" << pair.first
                      << " size=" << pair.second.size
                      << " access_time=" << pair.second.last_access_time << std::endl;
        }
    }
};

// Global shared buffer pool instance
static SharedBufferPool g_buffer_pool;

// Smart Streaming Policies
enum class StreamingState {
    STARTUP,        // Initial phase - always encode
    MOTION_ACTIVE,  // Motion detected - active encoding
    STABILIZING,    // Motion stopped - waiting for stabilization
    STATIC_CACHED,  // Static scene - send cached frames
    MOTION_RESUME   // Motion resumed - quick transition
};

struct SceneAnalysis {
    bool motion_detected = false;
    bool af_stable = false;
    float motion_level = 0.0f;
    std::chrono::milliseconds since_last_motion{0};
    std::chrono::milliseconds since_af_stable{0};
};

// Always stream policy (current behavior)
class AlwaysStreamPolicy {
public:
    bool shouldEncodeH264(const SceneAnalysis&) { return true; }
    bool shouldEncodeMJPEG(const SceneAnalysis&) { return true; }
    void onFrameEncoded(const std::vector<unsigned char>&, bool) {}
    template<typename H264Handler, typename MJPEGHandler>
    void sendCachedFrame(H264Handler&, MJPEGHandler&) {}
};

// Motion-triggered stream policy
class MotionStreamPolicy {
private:
    bool motion_active_ = false;
    std::chrono::steady_clock::time_point last_motion_time_;

public:
    bool shouldEncodeH264(const SceneAnalysis& scene) {
        updateMotionState(scene);
        return motion_active_;
    }

    bool shouldEncodeMJPEG(const SceneAnalysis& scene) {
        updateMotionState(scene);
        return motion_active_;
    }

    void onFrameEncoded(const std::vector<unsigned char>&, bool) {}
    template<typename H264Handler, typename MJPEGHandler>
    void sendCachedFrame(H264Handler&, MJPEGHandler&) {}

private:
    void updateMotionState(const SceneAnalysis& scene) {
        if (scene.motion_detected) {
            motion_active_ = true;
            last_motion_time_ = std::chrono::steady_clock::now();
        } else {
            auto now = std::chrono::steady_clock::now();
            auto time_since_motion = std::chrono::duration_cast<std::chrono::seconds>(now - last_motion_time_);
            if (time_since_motion > std::chrono::seconds(3)) {
                motion_active_ = false;
            }
        }
    }
};

// Hybrid smart streaming policy
class HybridStreamPolicy {
private:
    StreamingState current_state_ = StreamingState::STARTUP;
    std::chrono::steady_clock::time_point startup_time_;
    std::chrono::steady_clock::time_point stabilize_start_;
    std::chrono::steady_clock::time_point cache_timestamp_;
    std::chrono::steady_clock::time_point last_cache_send_;

    // Cached frames
    std::vector<unsigned char> cached_h264_frame_;
    std::vector<unsigned char> cached_mjpeg_frame_;
    bool has_cached_frames_ = false;

    // Cache transmission throttling
    std::chrono::milliseconds cache_send_interval_{1000}; // Send cached frame every 1 second instead of 30 FPS

public:
    HybridStreamPolicy() : startup_time_(std::chrono::steady_clock::now()),
                          last_cache_send_(std::chrono::steady_clock::now()) {}

    bool shouldEncodeH264(const SceneAnalysis& scene) {
        return shouldEncode(scene);
    }

    bool shouldEncodeMJPEG(const SceneAnalysis& scene) {
        return shouldEncode(scene);
    }

    void onFrameEncoded(const std::vector<unsigned char>& frame_data, bool is_h264) {
        // Cache the frame when transitioning to static mode
        if (current_state_ == StreamingState::STABILIZING) {
            if (is_h264) {
                // Only cache I-frames for H.264 to avoid decoder issues
                if (isIFrame(frame_data)) {
                    cached_h264_frame_ = frame_data;
                    std::cout << "[CACHE] Cached H.264 I-frame (" << frame_data.size() << " bytes)" << std::endl;
                } else {
                    std::cout << "[CACHE] Skipping H.264 P-frame (waiting for I-frame)" << std::endl;
                    return; // Don't set has_cached_frames_ yet
                }
            } else {
                // Always cache MJPEG frames (they're always complete)
                cached_mjpeg_frame_ = frame_data;
                std::cout << "[CACHE] Cached MJPEG frame (" << frame_data.size() << " bytes)" << std::endl;
            }
            has_cached_frames_ = true;
            cache_timestamp_ = std::chrono::steady_clock::now();
        }
    }

    template<typename H264Handler, typename MJPEGHandler>
    void sendCachedFrame(H264Handler& h264_handler, MJPEGHandler& mjpeg_handler) {
        if (!has_cached_frames_) return;

        auto now = std::chrono::steady_clock::now();
        auto time_since_last_send = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cache_send_);

        // Only send cached frame if enough time has passed (throttling)
        if (time_since_last_send >= cache_send_interval_) {
            if (!cached_h264_frame_.empty()) {
                h264_handler(cached_h264_frame_);
            }
            if (!cached_mjpeg_frame_.empty()) {
                mjpeg_handler(cached_mjpeg_frame_);
            }
            last_cache_send_ = now;

            // Debug: Log cache transmission
            static int cache_send_count = 0;
            std::cout << "[CACHE] Sent cached frame #" << ++cache_send_count
                      << " (interval: " << time_since_last_send.count() << "ms)" << std::endl;
        }
    }

private:
    bool shouldEncode(const SceneAnalysis& scene) {
        auto now = std::chrono::steady_clock::now();

        // Only log state changes, not every frame
        static int last_logged_state = -1;
        if (static_cast<int>(current_state_) != last_logged_state) {
            std::cout << "[DEBUG] State=" << static_cast<int>(current_state_)
                      << ", motion=" << scene.motion_detected
                      << ", af_stable=" << scene.af_stable
                      << ", motion_level=" << scene.motion_level << std::endl;
            last_logged_state = static_cast<int>(current_state_);
        }

        switch (current_state_) {
            case StreamingState::STARTUP: {
                auto startup_duration = std::chrono::duration_cast<std::chrono::seconds>(now - startup_time_);
                if (startup_duration > std::chrono::seconds(10)) {
                    transitionTo(scene.motion_detected ? StreamingState::MOTION_ACTIVE : StreamingState::STABILIZING);
                }
                return true;
            }

            case StreamingState::MOTION_ACTIVE: {
                if (!scene.motion_detected && scene.motion_level < 1.0f) {
                    transitionTo(StreamingState::STABILIZING);
                    stabilize_start_ = now;
                }
                return true;
            }

            case StreamingState::STABILIZING: {
                auto stabilize_duration = std::chrono::duration_cast<std::chrono::seconds>(now - stabilize_start_);
                if (scene.af_stable && scene.since_last_motion > std::chrono::milliseconds(2000)) {
                    transitionTo(StreamingState::STATIC_CACHED);
                    return true; // Last encoding before cache mode
                }
                if (scene.motion_detected) {
                    transitionTo(StreamingState::MOTION_ACTIVE);
                }
                return true;
            }

            case StreamingState::STATIC_CACHED: {
                if (scene.motion_detected || scene.motion_level > 2.0f) {
                    transitionTo(StreamingState::MOTION_RESUME);
                    return true;
                }
                // Refresh cache every 30 seconds
                auto cache_age = std::chrono::duration_cast<std::chrono::seconds>(now - cache_timestamp_);
                if (cache_age > std::chrono::seconds(30)) {
                    cache_timestamp_ = now;
                    return true;
                }
                return false; // Send cached frame
            }

            case StreamingState::MOTION_RESUME: {
                transitionTo(StreamingState::MOTION_ACTIVE);
                return true;
            }
        }
        return true;
    }

    void transitionTo(StreamingState new_state) {
        if (new_state != current_state_) {
            const char* state_names[] = {"STARTUP", "MOTION_ACTIVE", "STABILIZING", "STATIC_CACHED", "MOTION_RESUME"};
            std::cout << "StreamingState: " << state_names[static_cast<int>(current_state_)]
                      << " -> " << state_names[static_cast<int>(new_state)] << std::endl;
            current_state_ = new_state;
        }
    }

    // Configuration methods
public:
    void setCacheSendInterval(std::chrono::milliseconds interval) {
        cache_send_interval_ = interval;
        std::cout << "Cache send interval set to " << interval.count() << "ms" << std::endl;
    }

private:
    // Detect if frame is an I-frame (keyframe)
    bool isIFrame(const std::vector<unsigned char>& h264_data) const {
        // H.264 NAL unit header analysis
        // Look for NAL unit type 5 (IDR slice) or type 7/8 (SPS/PPS)
        for (size_t i = 0; i < h264_data.size() - 4; ++i) {
            // Find NAL unit start code (0x00 0x00 0x00 0x01)
            if (h264_data[i] == 0x00 && h264_data[i+1] == 0x00 &&
                h264_data[i+2] == 0x00 && h264_data[i+3] == 0x01) {

                if (i + 4 < h264_data.size()) {
                    uint8_t nal_type = h264_data[i+4] & 0x1F;
                    // NAL type 5 = IDR slice (I-frame), 7 = SPS, 8 = PPS
                    if (nal_type == 5 || nal_type == 7 || nal_type == 8) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
};

class MotionDetector {
private:
    // Previous Y frame (downsampled)
    std::vector<unsigned char> prev_y_frame_;

    // Frame dimensions
    int full_width_, full_height_;      // 1920x1080
    int sample_width_, sample_height_;  // 480x270 (каждая 4-я строка/столбец)

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

    bool detectMotion(int dma_fd, size_t total_size, size_t y_size) {
        if (dma_fd < 0 || y_size < full_width_ * full_height_) {
            return false;
        }

        // Use shared buffer pool
        void* mapped_data = g_buffer_pool.getMapping(dma_fd, total_size, "MotionDetector");
        if (!mapped_data) {
            return false;
        }

        unsigned char* y_data = static_cast<unsigned char*>(mapped_data);

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
            ssize_t sent = send(sockfd_, data.data() + total_sent, data_size - total_sent, 0);
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
    bool encodeDMA(int dma_fd, size_t offset, size_t length, std::vector<unsigned char>& h264_data) {
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

        ext_ctrl.id = 0x009909cb;  // video_gop_size control
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

        verify_ctrl.id = 0x009909cb;
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

        force_ctrl.id = 0x009909e5;  // force_key_frame
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

        ext_ctrl.id = 0x009909e2;  // repeat_sequence_header control
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

        verify_ctrl.id = 0x009909e2;
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

        mode_ctrl.id = 0x009909ce;  // video_bitrate_mode
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

        bitrate_ctrl.id = 0x009909cf;  // video_bitrate
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

        verify_ctrl.id = 0x009909cf; // video_bitrate
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
class TcpSender {
private:
    std::unique_ptr<TcpClient> tcp_client_;
    bool first_frame_ = true;

public:
    TcpSender(const std::string& server_ip, int server_port) {
        tcp_client_ = std::make_unique<TcpClient>(server_ip, server_port);
        if (!tcp_client_->connect()) {
            // throw std::runtime_error("Failed to connect to TCP server");
            std::cerr << "Warning: Failed to connect to TCP server" << std::endl;
        }
        std::cout << "TCP sender initialized successfully" << std::endl;
    }

    void send(const std::vector<unsigned char>& data) {
        tcp_client_->sendData(data);
        if (first_frame_) {
            // Send first frame multiple times for better streaming start
            for (int i = 0; i < 3; ++i) {
                tcp_client_->sendData(data);
            }
            std::cout << "First frame sent via TCP" << std::endl;
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

    void send(const std::vector<unsigned char>& data) {
        std::string filename = base_path_ + "/frame_" + std::to_string(frame_counter_++) + extension_;
        std::ofstream file(filename, std::ios::binary);
        file.write(reinterpret_cast<const char*>(data.data()), data.size());
    }
};

// Frame handler layer classes
template<typename Sender>
class MjpegFrameHandler {
private:
    Sender sender_;

public:
    explicit MjpegFrameHandler(Sender sender) : sender_(std::move(sender)) {}

    void operator()(const std::vector<unsigned char>& mjpeg_data) {
        sender_.send(mjpeg_data);
    }
};

template<typename Sender>
class H264FrameHandler {
private:
    Sender sender_;

public:
    explicit H264FrameHandler(Sender sender) : sender_(std::move(sender)) {}

    void operator()(const std::vector<unsigned char>& h264_data) {
        sender_.send(h264_data);
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
    int target_fps_ = 30;  // Default 30 FPS
    std::chrono::steady_clock::time_point last_frame_time_;
    bool static_mode_ = false; // If true, disable motion detection and capture continuously

    // Motion detection frame skip control
    int motion_frame_skip_ = 1;        // Process every N-th frame (1 = every frame)
    int motion_frame_counter_ = 0;     // Current frame counter

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

protected:
    // Members needed by SmartCapturerV2
    std::unique_ptr<CameraConfiguration>& getConfig() { return config_; }
    std::unique_ptr<MotionDetector>& getMotionDetector() { return motion_detector_; }
    std::unique_ptr<H264Encoder>& getH264Encoder() { return h264_encoder_; }
    std::unique_ptr<JpegEncoder>& getJpegEncoder() { return jpeg_encoder_; }
    MjpegHandler& getMjpegHandler() { return mjpeg_handler_; }
    H264Handler& getH264Handler() { return h264_handler_; }
    bool isMjpegEnabled() const { return use_mjpeg_; }

public:
    CapturerV2(MjpegHandler mjpeg_handler, H264Handler h264_handler, int width = 1920, int height = 1080)
        : cm_(std::make_unique<CameraManager>()),
          mjpeg_handler_(std::move(mjpeg_handler)),
          h264_handler_(std::move(h264_handler)),
          last_motion_time_(std::chrono::steady_clock::now()) {

        // Initialize encoders
        jpeg_encoder_ = std::make_unique<JpegEncoder>(width, height, 90); // Quality=90
        h264_encoder_ = std::make_unique<H264Encoder>(width, height, 60, 5000000); // GOP=60, Bitrate=5Mbps

        // Initialize motion detector
        motion_detector_ = std::make_unique<MotionDetector>(width, height, 4); // Downsample factor 4
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
        stream_config.size = Size(1920, 1080);  // Full HD
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
        
        // std::cout << "Allocated " << ret << " buffers" << std::endl;
        
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

            ControlList af_controls = buildAutofocusControls();
            request->controls() = af_controls;

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
        
        std::cout << "Capture started at " << target_fps_ << " FPS!" << std::endl;
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
        bool h264_sent = false;

        // Calculate total YUV size and store individual plane sizes
        size_t total_yuv_size = 0;
        std::vector<size_t> plane_sizes;
        for (const auto &plane : buffer->planes()) {
            plane_sizes.push_back(plane.length);
            total_yuv_size += plane.length;
        }
        const FrameBuffer::Plane &first_plane = buffer->planes()[0];

        // Motion detection BEFORE encoding (using Y plane only)
        bool should_record = true;
        if (motion_detector_ && plane_sizes.size() >= 1) {
            // Frame skipping optimization - only process every N-th frame
            motion_frame_counter_++;
            bool should_detect = static_mode_ ? true : (motion_frame_counter_ % motion_frame_skip_ == 0);

            bool motion = false;
            if (should_detect) {
                motion = motion_detector_->detectMotion(first_plane.fd.get(), total_yuv_size, plane_sizes[0]);
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
                    // Motion stopped and tail period expired - skip recording
                    // std::cout << "[MOTION] No motion for " << time_since_motion.count()
                    //           << "s, skipping frame" << std::endl;
                    should_record = false;
                } else {
                    // Still in tail recording period
                    // std::cout << "[MOTION] Tail recording (" << time_since_motion.count()
                    //           << "/" << motion_tail_duration_.count() << "s)" << std::endl;
                }
            }
        }

        // Skip encoding if motion-based recording is disabled
        if (!should_record) {
            // int acting_fps = target_fps_;
            // if (static_mode_) {
            //     acting_fps = std::max(1, target_fps_ / motion_frame_skip_);
            // }

            static_mode_ = true; // Switch to static mode if no motion
            return;
        }
        static_mode_ = false; // Reset static mode if capturing

        // Try H.264 encoding and output if enabled
        if (use_h264_output_ && use_h264_ && h264_encoder_) {
            std::vector<unsigned char> h264_data;

            if (h264_encoder_->encodeDMA(first_plane.fd.get(), 0, total_yuv_size, h264_data)) {
                h264_handler_(h264_data);
                h264_sent = true;

                // If only H.264 output is enabled, return early
                if (!use_mjpeg_) {
                    return;
                }
            } else {
                std::cout << "H.264 DMA encoding failed" << std::endl;
            }
        }

        // JPEG encoding and output if enabled
        if (use_mjpeg_) {
            void *mapped_data = g_buffer_pool.getMapping(first_plane.fd.get(), total_yuv_size, "MJPEG");
            if (!mapped_data) {
                std::cerr << "JPEG: Failed to get buffer mapping" << std::endl;
                if (!h264_sent) {
                    std::cerr << "No frame output - both H.264 and JPEG failed" << std::endl;
                }
            } else {
                std::vector<unsigned char> jpeg_data;
                unsigned char* yuv_data = static_cast<unsigned char*>(mapped_data);

                if (jpeg_encoder_->encode(yuv_data, total_yuv_size, jpeg_data)) {
                    mjpeg_handler_(jpeg_data);
                } else {
                    std::cerr << "JPEG encoding error" << std::endl;
                    if (!h264_sent) {
                        std::cerr << "No frame output - both H.264 and JPEG failed" << std::endl;
                    }
                }
                // No munmap needed - SharedBufferPool manages this automatically!
            }
        }

        // Calculate frame interval for target FPS
        auto frame_interval = std::chrono::microseconds(1000000 / target_fps_);
        auto now = std::chrono::steady_clock::now();

        if (frames_captured_ > 1 && target_fps_ < 30) {  // Skip timing for first frame
            auto elapsed = now - last_frame_time_;
            if (elapsed < frame_interval) {
                auto sleep_time = frame_interval - elapsed;
                std::this_thread::sleep_for(sleep_time);
            }
        }

        last_frame_time_ = std::chrono::steady_clock::now();
    }

    virtual void requestComplete(Request *req) {
        if (req->status() == Request::RequestComplete) {
            frames_captured_++;

            // std::cout << "Frame #" << frames_captured_ << " captured successfully" << std::endl;

            const Request::BufferMap &buffers = req->buffers();
            for (auto bufferPair : buffers) {
                FrameBuffer *buffer = bufferPair.second;
                processingFrameBuffer(buffer);
                break;
            }
            
            // Re-queue request with FPS throttling
            if (capture_running_) {
                req->reuse(Request::ReuseBuffers);

                ControlList af_controls = buildAutofocusControls();
                req->controls() = af_controls;

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
            motion_detector_ = std::make_unique<MotionDetector>(1920, 1080, 4);
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
        target_fps_ = fps;
        int64_t frame_duration_ns = 1000000000 / fps;
        std::cout << "Target frame rate set to " << fps << " FPS (duration: " << frame_duration_ns << " ns)" << std::endl;
        return true;
    }

    int getFrameRate() const {
        return target_fps_;
    }

private:
    ControlList buildAutofocusControls() {
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

// Smart streaming wrapper with policy-based streaming decisions
template<typename MjpegHandler, typename H264Handler, typename StreamingPolicy = AlwaysStreamPolicy>
class SmartCapturerV2 : public CapturerV2<MjpegHandler, H264Handler> {
private:
    StreamingPolicy streaming_policy_;
    std::chrono::steady_clock::time_point last_motion_time_;
    std::chrono::steady_clock::time_point last_af_stable_time_;

    // AF state monitoring (moved from base class)
    int current_af_state_ = -1;        // Current AF state (-1 = unknown)
    bool af_locked_ = false;           // True when AF is locked/focused
    std::chrono::steady_clock::time_point af_last_change_;  // When AF state last changed
    int af_scan_count_ = 0;            // Count of AF scan cycles

public:
    SmartCapturerV2(MjpegHandler mjpeg_handler, H264Handler h264_handler, int width = 1920, int height = 1080)
        : CapturerV2<MjpegHandler, H264Handler>(std::move(mjpeg_handler), std::move(h264_handler), width, height),
          last_motion_time_(std::chrono::steady_clock::now()),
          last_af_stable_time_(std::chrono::steady_clock::now()) {

        std::cout << "SmartCapturerV2 initialized with streaming policy" << std::endl;
    }

    // Override requestComplete to add AF monitoring
    void requestComplete(Request *req) override {
        if (req->status() == Request::RequestComplete) {
            // AF monitoring first
            monitorAutofocusState(req);

            // Call base class implementation
            CapturerV2<MjpegHandler, H264Handler>::requestComplete(req);
        } else {
            std::cerr << "Frame capture error" << std::endl;
        }
    }

protected:
    // Override frame processing to add smart streaming logic
    void processingFrameBuffer(const FrameBuffer *buffer) override {
        if (!buffer) return;

        // Calculate total YUV size (sum of all planes)
        size_t total_yuv_size = 0;
        std::vector<size_t> plane_sizes;
        for (const auto &plane : buffer->planes()) {
            plane_sizes.push_back(plane.length);
            total_yuv_size += plane.length;
        }
        const FrameBuffer::Plane &first_plane = buffer->planes()[0];

        // Motion detection (always run for scene analysis)
        if (this->getMotionDetector()) {
            this->getMotionDetector()->detectMotion(first_plane.fd.get(), total_yuv_size, plane_sizes[0]);
        }

        // Analyze current scene state
        SceneAnalysis scene = analyzeScene(first_plane, total_yuv_size);

        // Make streaming decisions based on policy
        bool should_encode_h264 = streaming_policy_.shouldEncodeH264(scene);
        bool should_encode_mjpeg = streaming_policy_.shouldEncodeMJPEG(scene);
        bool h264_sent = false;

        // H.264 encoding if enabled and decided by policy
        if (this->isH264OutputEnabled() && should_encode_h264) {
            std::vector<unsigned char> h264_data;
            if (this->getH264Encoder() && this->getH264Encoder()->encodeDMA(first_plane.fd.get(), 0, total_yuv_size, h264_data)) {
                this->getH264Handler()(h264_data);
                streaming_policy_.onFrameEncoded(h264_data, true);
                h264_sent = true;
            }
        }

        // MJPEG encoding if enabled and decided by policy
        if (this->isMjpegEnabled() && should_encode_mjpeg) {
            void *mapped_data = g_buffer_pool.getMapping(first_plane.fd.get(), total_yuv_size, "SmartMJPEG");
            if (mapped_data) {
                std::vector<unsigned char> jpeg_data;
                unsigned char* yuv_data = static_cast<unsigned char*>(mapped_data);

                if (this->getJpegEncoder()->encode(yuv_data, total_yuv_size, jpeg_data)) {
                    this->getMjpegHandler()(jpeg_data);
                    streaming_policy_.onFrameEncoded(jpeg_data, false);
                } else {
                    std::cerr << "Smart JPEG encoding error" << std::endl;
                }
            }
        }

        // Send cached frames if policy decides not to encode
        if (!should_encode_h264 || !should_encode_mjpeg) {
            streaming_policy_.sendCachedFrame(this->getH264Handler(), this->getMjpegHandler());
        }
    }

    // Monitor autofocus state from request metadata (moved from base class)
    void monitorAutofocusState(Request *req) {
        const auto& metadata = req->metadata();

        // Check if AF state is available in metadata
        if (metadata.contains(controls::AfState.id())) {
            auto af_state_opt = metadata.get(controls::AfState);
            if (!af_state_opt) return;
            int new_af_state = *af_state_opt;

            // AF state changed?
            if (new_af_state != current_af_state_) {
                auto now = std::chrono::steady_clock::now();

                // Log state change
                const char* state_names[] = {"Idle", "Scanning", "Focused", "Failed"};
                const char* old_name = (current_af_state_ >= 0 && current_af_state_ <= 3) ?
                                       state_names[current_af_state_] : "Unknown";
                const char* new_name = (new_af_state >= 0 && new_af_state <= 3) ?
                                       state_names[new_af_state] : "Unknown";

                std::cout << "AF state: " << old_name << " -> " << new_name << std::endl;

                // Update state tracking
                current_af_state_ = new_af_state;
                af_last_change_ = now;

                // Check for specific state transitions
                if (new_af_state == 1) {  // Scanning
                    af_scan_count_++;
                    af_locked_ = false;
                    std::cout << "AF scanning... (cycle #" << af_scan_count_ << ")" << std::endl;
                } else if (new_af_state == 2) {  // Focused
                    af_locked_ = true;
                    std::cout << "AF LOCKED! Focus achieved after " << af_scan_count_
                              << " scan cycles" << std::endl;
                } else if (new_af_state == 3) {  // Failed
                    af_locked_ = false;
                    std::cout << "AF failed to find focus" << std::endl;
                } else if (new_af_state == 0) {  // Idle
                    af_locked_ = false;
                }
            }
        }

        // Also check lens position if available
        if (metadata.contains(controls::LensPosition.id())) {
            auto lens_pos_opt = metadata.get(controls::LensPosition);
            if (!lens_pos_opt) return;
            float lens_pos = *lens_pos_opt;
            // Only log significant lens movements to avoid spam
            static float last_logged_pos = -999.0f;
            if (std::abs(lens_pos - last_logged_pos) > 0.1f) {
                std::cout << "Lens position: " << std::fixed << std::setprecision(2)
                          << lens_pos << std::endl;
                last_logged_pos = lens_pos;
            }
        }
    }

private:
    SceneAnalysis analyzeScene(const FrameBuffer::Plane &plane, size_t total_size) {
        SceneAnalysis scene;
        auto now = std::chrono::steady_clock::now();

        // Motion analysis
        if (this->getMotionDetector()) {
            scene.motion_detected = this->getMotionDetector()->isMotionDetected();
            scene.motion_level = this->getMotionDetector()->getMotionLevel();

            if (scene.motion_detected) {
                last_motion_time_ = now;
            }
            scene.since_last_motion = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motion_time_);
        }

        // AF stability analysis
        scene.af_stable = isAutofocusStable();
        if (scene.af_stable) {
            last_af_stable_time_ = now;
        }
        scene.since_af_stable = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_af_stable_time_);

        // Remove per-frame logging to reduce spam

        return scene;
    }

    // AF state monitoring methods (moved from base class)
    bool isAutofocusLocked() const {
        return af_locked_;
    }

    int getAutofocusState() const {
        return current_af_state_;
    }

    const char* getAutofocusStateName() const {
        const char* state_names[] = {"Idle", "Scanning", "Focused", "Failed"};
        if (current_af_state_ >= 0 && current_af_state_ <= 3) {
            return state_names[current_af_state_];
        }
        return "Unknown";
    }

    int getAutofocusScanCount() const {
        return af_scan_count_;
    }

    // Check if AF has been stable (locked) for specified duration
    bool isAutofocusStable(std::chrono::milliseconds duration = std::chrono::milliseconds(500)) const {
        if (!af_locked_) return false;
        auto now = std::chrono::steady_clock::now();
        auto time_since_change = std::chrono::duration_cast<std::chrono::milliseconds>(now - af_last_change_);
        return time_since_change >= duration;
    }

public:
    // Access to streaming policy for configuration
    StreamingPolicy& getStreamingPolicy() {
        return streaming_policy_;
    }

    const StreamingPolicy& getStreamingPolicy() const {
        return streaming_policy_;
    }
};

int main(int argc, char *argv[]) {
    std::cout << "Camera Module v3 + Pi Zero 2W + libcamera" << std::endl;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <server_ip> <server_port>" << std::endl;
        return -1;
    }

    try {
        TcpSender tcp_sender_mjpeg(std::string(argv[1]), std::stoi(argv[2]) - 1); // Port for MJPEG

        MjpegFrameHandler<TcpSender> mjpeg_handler(std::move(tcp_sender_mjpeg));

        TcpSender tcp_sender_h264(std::string(argv[1]), std::stoi(argv[2])); // Different port
        H264FrameHandler<TcpSender> h264_handler(std::move(tcp_sender_h264));

        // Regular capturer (always encoding)
        CapturerV2 capturer(std::move(mjpeg_handler), std::move(h264_handler));

        // Smart streaming alternatives (uncomment to use):
        //
        // Option 1: Motion-triggered streaming
        // SmartCapturerV2<MjpegFrameHandler<TcpSender>, H264FrameHandler<TcpSender>, MotionStreamPolicy>
        //     capturer(std::move(mjpeg_handler), std::move(h264_handler));
        //
        // Option 2: Hybrid intelligent streaming
        // SmartCapturerV2<MjpegFrameHandler<TcpSender>, H264FrameHandler<TcpSender>, HybridStreamPolicy>
        //     capturer(std::move(mjpeg_handler), std::move(h264_handler));
        //
        // Option 3: Always stream (same as regular CapturerV2)
        // SmartCapturerV2<MjpegFrameHandler<TcpSender>, H264FrameHandler<TcpSender>, AlwaysStreamPolicy>
        //     capturer(std::move(mjpeg_handler), std::move(h264_handler));

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
        capturer.setAutofocusSpeed(0); // 0=Normal, 1=Fast
        capturer.setAutofocusRange(1); // 0=Normal, 1=Macro, 2=Full
        capturer.enableAutofocus(true);
        capturer.setFrameRate(30);  // 30 FPS
        capturer.enableMotionDetection(true); // Enable motion detection by default
        capturer.setMotionFrameSkip(30); // Process every 30th frame
        capturer.setMotionTailDuration(3); // Record for 3 seconds after motion stops


        if (capturer.isH264Available()) {
            std::cout << "H.264 hardware encoder detected!" << std::endl;
            capturer.enableH264Encoding(true);
            capturer.setH264GopSize(60);  // 1 I-frame every 60 frames
            capturer.setH264Bitrate(5000000, false);  // 5Mbps
        } else {
            std::cout << "H.264 hardware encoder not available, using JPEG only" << std::endl;
        }

        // Configure output formats - you can choose which formats to enable:

        // Option 1: Only MJPEG output (port 9998)
        capturer.enableMjpegOutput(false);
        capturer.enableH264Output(true);

        // Option 2: Only H.264 output (port 9999) - uncomment to use
        // capturer.enableMjpegOutput(false);
        // capturer.enableH264Output(true);

        // Option 3: Both outputs (MJPEG to port 9998, H.264 to port 9999) - uncomment to use
        // capturer.enableMjpegOutput(true);
        // capturer.enableH264Output(true);

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
