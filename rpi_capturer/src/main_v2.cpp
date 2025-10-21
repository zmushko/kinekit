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
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <curl/curl.h>
#include <map>
#include <string>
#include <queue>
#include <deque>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <cpptoml.h>

// FFmpeg libav for MP4 muxing
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

// ARM NEON intrinsics for SIMD optimization
#ifdef __ARM_NEON
#include <arm_neon.h>
#include <filesystem>
#include <algorithm>
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

// ============================================================================
// Configuration Management
// ============================================================================

class Config {
public:
    // Camera settings
    struct Camera {
        int width = 1920;
        int height = 1080;
        int fps = 30;
    } camera;

    // Autofocus settings
    struct Autofocus {
        bool enabled = true;
        int mode = 2;      // 0=Auto, 1=Manual, 2=Continuous
        int speed = 1;     // 0=Normal, 1=Fast
        int range = 2;     // 0=Normal, 1=Macro, 2=Full
    } autofocus;

    // Motion detection settings
    struct MotionDetection {
        bool enabled = true;
        int frame_skip = 7;
        float pixel_change_sensitivity = 25.0f;  // Pixel brightness difference threshold (0-255)
        float min_object_size = 0.05f;           // Minimum object size as fraction of frame (0.0-1.0)
    } motion_detection;

    // MJPEG settings
    struct Mjpeg {
        bool enabled = true;
        bool output_enabled = true;
        int encode_interval_ms = 1000;
        int burst_photo_count = 1;
        int max_burst_packets_during_recording = 3;  // Max burst packets to send while recording video
    } mjpeg;

    // H.264 settings
    struct H264 {
        bool enabled = true;
        bool output_enabled = true;
        int gop_size = 60;
        int bitrate = 5000000;
        bool cbr = false;
        bool sps_pps_repeat = true;
    } h264;

    // Telegram settings
    struct Telegram {
        bool enabled = true;
        std::string bot_token = "";
        std::string chat_id = "";
        int max_queue_size = 5;
    } telegram;

    // TCP settings
    struct Tcp {
        std::string format = "h264";  // "h264" or "mjpeg"

        struct Broadcast {
            bool enabled = true;
            int port = 8554;
            int max_clients = 5;
        } broadcast;

        struct Client {
            bool enabled = false;
            std::string remote_ip = "10.0.0.2";
            int remote_port = 9999;
            int reconnect_interval_sec = 1;
        } client;
    } tcp;

    // Video recording settings
    struct VideoRecording {
        bool enabled = true;
        int duration_sec = 30;
        int preroll_sec = 3;
        int tail_duration = 5;
        int max_memory_mb = 20;
        std::string failed_videos_dir = "/home/pi/recordings/failed";
        int max_failed_files = 50;
        int retry_interval_sec = 2;
        int max_retries = 2;
        bool send_to_telegram = true;
    } video_recording;

    // Load configuration from TOML file
    static Config load(const std::string& filename) {
        Config config;

        try {
            auto toml = cpptoml::parse_file(filename);

            // Camera
            auto camera = toml->get_table("camera");
            if (camera) {
                config.camera.width = camera->get_as<int>("width").value_or(config.camera.width);
                config.camera.height = camera->get_as<int>("height").value_or(config.camera.height);
                config.camera.fps = camera->get_as<int>("fps").value_or(config.camera.fps);
            }

            // Autofocus
            auto autofocus = toml->get_table("autofocus");
            if (autofocus) {
                config.autofocus.enabled = autofocus->get_as<bool>("enabled").value_or(config.autofocus.enabled);
                config.autofocus.mode = autofocus->get_as<int>("mode").value_or(config.autofocus.mode);
                config.autofocus.speed = autofocus->get_as<int>("speed").value_or(config.autofocus.speed);
                config.autofocus.range = autofocus->get_as<int>("range").value_or(config.autofocus.range);
            }

            // Motion detection
            auto motion = toml->get_table("motion_detection");
            if (motion) {
                config.motion_detection.enabled = motion->get_as<bool>("enabled").value_or(config.motion_detection.enabled);
                config.motion_detection.frame_skip = motion->get_as<int>("frame_skip").value_or(config.motion_detection.frame_skip);
                config.motion_detection.pixel_change_sensitivity = motion->get_as<double>("pixel_change_sensitivity").value_or(config.motion_detection.pixel_change_sensitivity);
                config.motion_detection.min_object_size = motion->get_as<double>("min_object_size").value_or(config.motion_detection.min_object_size);
            }

            // MJPEG
            auto mjpeg = toml->get_table("mjpeg");
            if (mjpeg) {
                config.mjpeg.enabled = mjpeg->get_as<bool>("enabled").value_or(config.mjpeg.enabled);
                config.mjpeg.output_enabled = mjpeg->get_as<bool>("output_enabled").value_or(config.mjpeg.output_enabled);
                config.mjpeg.encode_interval_ms = mjpeg->get_as<int>("encode_interval_ms").value_or(config.mjpeg.encode_interval_ms);
                config.mjpeg.burst_photo_count = mjpeg->get_as<int>("burst_photo_count").value_or(config.mjpeg.burst_photo_count);
                config.mjpeg.max_burst_packets_during_recording = mjpeg->get_as<int>("max_burst_packets_during_recording").value_or(config.mjpeg.max_burst_packets_during_recording);
            }

            // H.264
            auto h264 = toml->get_table("h264");
            if (h264) {
                config.h264.enabled = h264->get_as<bool>("enabled").value_or(config.h264.enabled);
                config.h264.output_enabled = h264->get_as<bool>("output_enabled").value_or(config.h264.output_enabled);
                config.h264.gop_size = h264->get_as<int>("gop_size").value_or(config.h264.gop_size);
                config.h264.bitrate = h264->get_as<int>("bitrate").value_or(config.h264.bitrate);
                config.h264.cbr = h264->get_as<bool>("cbr").value_or(config.h264.cbr);
                config.h264.sps_pps_repeat = h264->get_as<bool>("sps_pps_repeat").value_or(config.h264.sps_pps_repeat);
            }

            // Telegram
            auto telegram = toml->get_table("telegram");
            if (telegram) {
                config.telegram.enabled = telegram->get_as<bool>("enabled").value_or(config.telegram.enabled);
                config.telegram.bot_token = telegram->get_as<std::string>("bot_token").value_or(config.telegram.bot_token);
                config.telegram.chat_id = telegram->get_as<std::string>("chat_id").value_or(config.telegram.chat_id);
                config.telegram.max_queue_size = telegram->get_as<int>("max_queue_size").value_or(config.telegram.max_queue_size);
            }

            // TCP
            auto tcp = toml->get_table("tcp");
            if (tcp) {
                config.tcp.format = tcp->get_as<std::string>("format").value_or(config.tcp.format);

                auto tcp_broadcast = tcp->get_table("broadcast");
                if (tcp_broadcast) {
                    config.tcp.broadcast.enabled = tcp_broadcast->get_as<bool>("enabled").value_or(config.tcp.broadcast.enabled);
                    config.tcp.broadcast.port = tcp_broadcast->get_as<int>("port").value_or(config.tcp.broadcast.port);
                    config.tcp.broadcast.max_clients = tcp_broadcast->get_as<int>("max_clients").value_or(config.tcp.broadcast.max_clients);
                }

                auto tcp_client = tcp->get_table("client");
                if (tcp_client) {
                    config.tcp.client.enabled = tcp_client->get_as<bool>("enabled").value_or(config.tcp.client.enabled);
                    config.tcp.client.remote_ip = tcp_client->get_as<std::string>("remote_ip").value_or(config.tcp.client.remote_ip);
                    config.tcp.client.remote_port = tcp_client->get_as<int>("remote_port").value_or(config.tcp.client.remote_port);
                    config.tcp.client.reconnect_interval_sec = tcp_client->get_as<int>("reconnect_interval_sec").value_or(config.tcp.client.reconnect_interval_sec);
                }
            }

            // Video recording
            auto video_recording = toml->get_table("video_recording");
            if (video_recording) {
                config.video_recording.enabled = video_recording->get_as<bool>("enabled").value_or(config.video_recording.enabled);
                config.video_recording.duration_sec = video_recording->get_as<int>("duration_sec").value_or(config.video_recording.duration_sec);
                config.video_recording.preroll_sec = video_recording->get_as<int>("preroll_sec").value_or(config.video_recording.preroll_sec);
                config.video_recording.tail_duration = video_recording->get_as<int>("tail_duration").value_or(config.video_recording.tail_duration);
                config.video_recording.max_memory_mb = video_recording->get_as<int>("max_memory_mb").value_or(config.video_recording.max_memory_mb);
                config.video_recording.failed_videos_dir = video_recording->get_as<std::string>("failed_videos_dir").value_or(config.video_recording.failed_videos_dir);
                config.video_recording.max_failed_files = video_recording->get_as<int>("max_failed_files").value_or(config.video_recording.max_failed_files);
                config.video_recording.retry_interval_sec = video_recording->get_as<int>("retry_interval_sec").value_or(config.video_recording.retry_interval_sec);
                config.video_recording.max_retries = video_recording->get_as<int>("max_retries").value_or(config.video_recording.max_retries);
                config.video_recording.send_to_telegram = video_recording->get_as<bool>("send_to_telegram").value_or(config.video_recording.send_to_telegram);
            }

            std::cout << "Configuration loaded from '" << filename << "'" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error loading config: " << e.what() << std::endl;
            std::cerr << "Using default configuration" << std::endl;
        }

        return config;
    }

    // Print configuration summary
    void print() const {
        std::cout << "\n========== Configuration Summary ==========" << std::endl;
        std::cout << "Camera: " << camera.width << "x" << camera.height << " @ " << camera.fps << " FPS" << std::endl;
        std::cout << "Autofocus: " << (autofocus.enabled ? "enabled" : "disabled") << std::endl;
        std::cout << "Motion Detection: " << (motion_detection.enabled ? "enabled" : "disabled")
                  << " (skip=" << motion_detection.frame_skip << ")" << std::endl;
        std::cout << "MJPEG: " << (mjpeg.enabled ? "enabled" : "disabled")
                  << " (burst=" << mjpeg.burst_photo_count << ")" << std::endl;
        std::cout << "H.264: " << (h264.enabled ? "enabled" : "disabled")
                  << " (bitrate=" << h264.bitrate << ", gop=" << h264.gop_size << ")" << std::endl;
        std::cout << "Telegram: " << (telegram.enabled ? "enabled" : "disabled") << std::endl;
        std::cout << "TCP Format: " << tcp.format << std::endl;
        std::cout << "TCP Broadcast: " << (tcp.broadcast.enabled ? "enabled" : "disabled")
                  << " (port=" << tcp.broadcast.port << ")" << std::endl;
        std::cout << "TCP Client: " << (tcp.client.enabled ? "enabled" : "disabled");
        if (tcp.client.enabled) {
            std::cout << " (" << tcp.client.remote_ip << ":" << tcp.client.remote_port << ")";
        }
        std::cout << "\n==========================================\n" << std::endl;
    }
};

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

    // POST multipart/form-data with multiple files
    Response postMultipartFiles(
        const std::string& url,
        const std::map<std::string, std::string>& fields,
        const std::vector<std::pair<std::string, std::vector<unsigned char>>>& files, // field_name -> file_data
        int timeout_ms = 30000
    ) {
        Response response;
        CURL* curl = curl_easy_init();

        if (!curl) {
            std::cerr << "Failed to initialize CURL" << std::endl;
            return response;
        }

        curl_mime* mime = curl_mime_init(curl);

        // Add text fields
        for (auto it = fields.begin(); it != fields.end(); ++it) {
            curl_mimepart* part = curl_mime_addpart(mime);
            curl_mime_name(part, it->first.c_str());
            curl_mime_data(part, it->second.c_str(), CURL_ZERO_TERMINATED);
        }

        // Add multiple files
        for (size_t i = 0; i < files.size(); ++i) {
            const std::string& field_name = files[i].first;
            const std::vector<unsigned char>& file_data = files[i].second;

            curl_mimepart* file_part = curl_mime_addpart(mime);
            curl_mime_name(file_part, field_name.c_str());
            curl_mime_data(file_part, reinterpret_cast<const char*>(file_data.data()), file_data.size());
            curl_mime_filename(file_part, ("photo" + std::to_string(i) + ".jpg").c_str());
            curl_mime_type(file_part, "image/jpeg");
        }

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

    // Simple GET request
    Response get(const std::string& url, int timeout_ms = 35000) {
        Response response;
        CURL* curl = curl_easy_init();

        if (!curl) {
            std::cerr << "Failed to initialize CURL" << std::endl;
            return response;
        }

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK) {
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
            response.success = (response.status_code >= 200 && response.status_code < 300);
        } else {
            std::cerr << "CURL error: " << curl_easy_strerror(res) << std::endl;
            response.success = false;
        }

        curl_easy_cleanup(curl);
        return response;
    }

    // POST application/x-www-form-urlencoded
    Response postForm(const std::string& url, const std::map<std::string, std::string>& fields, int timeout_ms = 10000) {
        Response response;
        CURL* curl = curl_easy_init();

        if (!curl) {
            std::cerr << "Failed to initialize CURL" << std::endl;
            return response;
        }

        // Build form data
        std::string post_data;
        for (const auto& [key, value] : fields) {
            if (!post_data.empty()) post_data += "&";

            char* escaped_key = curl_easy_escape(curl, key.c_str(), key.length());
            char* escaped_value = curl_easy_escape(curl, value.c_str(), value.length());

            post_data += std::string(escaped_key) + "=" + std::string(escaped_value);

            curl_free(escaped_key);
            curl_free(escaped_value);
        }

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK) {
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
            response.success = (response.status_code >= 200 && response.status_code < 300);
        } else {
            std::cerr << "CURL error: " << curl_easy_strerror(res) << std::endl;
            response.success = false;
        }

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

    // Send media group (2-10 photos/videos in one message)
    bool sendMediaGroup(
        const std::string& chat_id,
        const std::vector<std::vector<unsigned char>>& photos,
        const std::string& caption = std::string()
    ) {
        if (photos.empty() || photos.size() > 10) {
            std::cerr << "Media group must contain 2-10 photos" << std::endl;
            return false;
        }

        // Build media array in JSON format
        std::string media_json = "[";
        for (size_t i = 0; i < photos.size(); ++i) {
            if (i > 0) media_json += ",";
            media_json += "{\"type\":\"photo\",\"media\":\"attach://photo" + std::to_string(i) + "\"";
            // Add caption only to first photo
            if (i == 0 && !caption.empty()) {
                // Escape quotes in caption
                std::string escaped_caption = caption;
                size_t pos = 0;
                while ((pos = escaped_caption.find("\"", pos)) != std::string::npos) {
                    escaped_caption.replace(pos, 1, "\\\"");
                    pos += 2;
                }
                media_json += ",\"caption\":\"" + escaped_caption + "\"";
            }
            media_json += "}";
        }
        media_json += "]";

        // Prepare fields
        std::map<std::string, std::string> fields;
        fields["chat_id"] = chat_id;
        fields["media"] = media_json;

        // Prepare files vector
        std::vector<std::pair<std::string, std::vector<unsigned char>>> files;
        for (size_t i = 0; i < photos.size(); ++i) {
            files.push_back({"photo" + std::to_string(i), photos[i]});
        }

        // Send with retry logic
        for (int attempt = 0; attempt < max_retries_; ++attempt) {
            std::string url = base_url_ + "sendMediaGroup";

            auto response = http_client_.postMultipartFiles(url, fields, files);

            if (response.success) {
                std::cout << "Telegram API: sendMediaGroup success (attempt "
                         << (attempt + 1) << ")" << std::endl;
                return true;
            }

            std::cerr << "Telegram API: sendMediaGroup failed (attempt "
                     << (attempt + 1) << "/" << max_retries_ << "): "
                     << "HTTP " << response.status_code << std::endl;

            if (attempt < max_retries_ - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
            }
        }

        std::cerr << "Telegram API: sendMediaGroup failed after "
                 << max_retries_ << " attempts" << std::endl;
        return false;
    }

    // Send text message
    bool sendMessage(const std::string& chat_id, const std::string& text) {
        std::string url = base_url_ + "sendMessage";

        std::map<std::string, std::string> fields;
        fields["chat_id"] = chat_id;
        fields["text"] = text;

        auto response = http_client_.postForm(url, fields);

        if (response.success) {
            std::cout << "Telegram: sent message to " << chat_id << std::endl;
            return true;
        }

        std::cerr << "Telegram: failed to send message, HTTP " << response.status_code << std::endl;
        return false;
    }

    // Get updates (for receiving commands)
    std::string getUpdates(int offset = 0, int timeout = 30) {
        std::string url = base_url_ + "getUpdates?offset=" + std::to_string(offset)
                         + "&timeout=" + std::to_string(timeout);

        auto response = http_client_.get(url);

        if (response.success) {
            return response.body;
        }

        return "";
    }
};

// Telegram Command Handler
class TelegramCommandHandler {
private:
    TelegramBotApi* bot_api_;
    std::string chat_id_;
    int last_update_id_ = 0;
    std::atomic<bool> running_{false};
    std::thread polling_thread_;

    // Callback function for handling commands
    std::function<void(const std::string&)> command_callback_;

    // Simple JSON parser for getting command text from update
    std::string extractCommand(const std::string& json_response) {
        // Look for "text":"..." pattern
        size_t text_pos = json_response.find("\"text\":\"");
        if (text_pos == std::string::npos) {
            return "";
        }

        size_t start = text_pos + 8; // Length of "text":"
        size_t end = json_response.find("\"", start);

        if (end == std::string::npos) {
            return "";
        }

        return json_response.substr(start, end - start);
    }

    // Extract update_id from JSON
    int extractUpdateId(const std::string& json_response) {
        // Look for last "update_id": pattern
        size_t pos = json_response.rfind("\"update_id\":");
        if (pos == std::string::npos) {
            return last_update_id_;
        }

        size_t start = pos + 12; // Length of "update_id":
        size_t end = json_response.find_first_of(",}", start);

        if (end == std::string::npos) {
            return last_update_id_;
        }

        try {
            return std::stoi(json_response.substr(start, end - start));
        } catch (...) {
            return last_update_id_;
        }
    }

    // Polling loop
    void pollingLoop() {
        std::cout << "Telegram command polling started" << std::endl;

        while (running_) {
            try {
                // Get updates with long polling (30 sec timeout)
                std::string response = bot_api_->getUpdates(last_update_id_ + 1, 30);

                if (response.empty()) {
                    continue;
                }

                // Extract command
                std::string command = extractCommand(response);
                if (!command.empty() && command[0] == '/') {
                    std::cout << "Received command: " << command << std::endl;

                    // Call callback
                    if (command_callback_) {
                        command_callback_(command);
                    }
                }

                // Update last_update_id
                int new_update_id = extractUpdateId(response);
                if (new_update_id > last_update_id_) {
                    last_update_id_ = new_update_id;
                }

            } catch (const std::exception& e) {
                std::cerr << "Error in polling loop: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        std::cout << "Telegram command polling stopped" << std::endl;
    }

public:
    TelegramCommandHandler(TelegramBotApi* bot_api, const std::string& chat_id)
        : bot_api_(bot_api), chat_id_(chat_id) {}

    ~TelegramCommandHandler() {
        stop();
    }

    void setCommandCallback(std::function<void(const std::string&)> callback) {
        command_callback_ = callback;
    }

    void start() {
        if (running_) {
            return;
        }

        running_ = true;
        polling_thread_ = std::thread(&TelegramCommandHandler::pollingLoop, this);
    }

    void stop() {
        if (!running_) {
            return;
        }

        running_ = false;
        if (polling_thread_.joinable()) {
            polling_thread_.join();
        }
    }

    void sendResponse(const std::string& text) {
        bot_api_->sendMessage(chat_id_, text);
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

    float getThreshold() const { return threshold_; }
    float getMotionAreaThreshold() const { return motion_area_threshold_; }

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
            // MSG_NOSIGNAL prevents SIGPIPE when connection is broken
            ssize_t sent = ::send(sockfd_, data.data() + total_sent, data_size - total_sent, MSG_NOSIGNAL);
            if (sent < 0) {
                std::cerr << "Failed to send data: " << strerror(errno) << std::endl;
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

// ============================================================================
// Circular Video Buffer - stores H.264 frames for preroll recording
// ============================================================================
class CircularVideoBuffer {
private:
    struct H264Frame {
        std::vector<unsigned char> data;
        bool is_keyframe;
        std::chrono::steady_clock::time_point timestamp;

        size_t size() const { return data.size(); }
    };

    std::deque<H264Frame> frames_;
    size_t max_size_bytes_;
    size_t current_size_bytes_;

public:
    CircularVideoBuffer(size_t max_size_mb)
        : max_size_bytes_(max_size_mb * 1024 * 1024), current_size_bytes_(0) {
        std::cout << "CircularVideoBuffer initialized: max " << max_size_mb << " MB" << std::endl;
    }

    // Add frame to buffer
    void addFrame(const std::vector<unsigned char>& data, bool is_keyframe) {
        H264Frame frame;
        frame.data = data;
        frame.is_keyframe = is_keyframe;
        frame.timestamp = std::chrono::steady_clock::now();

        size_t frame_size = frame.size();
        current_size_bytes_ += frame_size;
        frames_.push_back(std::move(frame));

        // Remove oldest frames if buffer exceeds max size
        while (current_size_bytes_ > max_size_bytes_ && !frames_.empty()) {
            current_size_bytes_ -= frames_.front().size();
            frames_.pop_front();
        }
    }

    // Get all frames (for writing preroll to file)
    std::vector<H264Frame> getAllFrames() const {
        return std::vector<H264Frame>(frames_.begin(), frames_.end());
    }

    // Get frames from specific duration (e.g., last N seconds)
    std::vector<H264Frame> getFrames(int duration_sec) const {
        std::vector<H264Frame> result;
        auto now = std::chrono::steady_clock::now();
        auto cutoff = now - std::chrono::seconds(duration_sec);

        for (const auto& frame : frames_) {
            if (frame.timestamp >= cutoff) {
                result.push_back(frame);
            }
        }
        return result;
    }

    void clear() {
        frames_.clear();
        current_size_bytes_ = 0;
    }

    size_t getFrameCount() const { return frames_.size(); }
    size_t getCurrentSizeMB() const { return current_size_bytes_ / (1024 * 1024); }
    size_t getCurrentSizeBytes() const { return current_size_bytes_; }
};

// ============================================================================
// Helper function to create directory recursively (like mkdir -p)
// ============================================================================
static bool createDirectoryRecursive(const std::string& path) {
    if (path.empty()) return false;

    // Check if already exists
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    // Find parent directory
    size_t pos = path.find_last_of('/');
    if (pos != std::string::npos && pos > 0) {
        std::string parent = path.substr(0, pos);
        if (!createDirectoryRecursive(parent)) {
            return false;
        }
    }

    // Create this directory
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create directory: " << path << " (" << strerror(errno) << ")" << std::endl;
        return false;
    }

    return true;
}

// ============================================================================
// MP4 Video File Writer - writes H.264 frames to MP4 container using libav
// ============================================================================
class VideoFileWriter {
private:
    AVFormatContext* fmt_ctx_;
    AVStream* video_stream_;
    std::string filename_;
    size_t bytes_written_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point first_frame_time_;
    int64_t video_pts_;
    bool is_open_;
    bool header_written_;
    bool first_frame_written_;
    int width_;
    int height_;
    int fps_;

public:
    VideoFileWriter()
        : fmt_ctx_(nullptr), video_stream_(nullptr),
          bytes_written_(0), video_pts_(0), is_open_(false), header_written_(false),
          first_frame_written_(false), width_(1920), height_(1080), fps_(30) {}

    ~VideoFileWriter() {
        close();
    }

    void setResolution(int width, int height, int fps) {
        width_ = width;
        height_ = height;
        fps_ = fps;
    }

    // Create new MP4 video file with timestamp
    bool open(const std::string& directory = "/home/pi/recordings") {
        if (is_open_) {
            std::cerr << "VideoFileWriter: file already open" << std::endl;
            return false;
        }

        // Create directory if it doesn't exist
        if (!createDirectoryRecursive(directory)) {
            std::cerr << "VideoFileWriter: failed to create directory: " << directory << std::endl;
            return false;
        }

        // Generate filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time_t);

        std::ostringstream oss;
        oss << directory << "/motion_"
            << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
            << ".mp4";
        filename_ = oss.str();

        // Allocate output format context
        avformat_alloc_output_context2(&fmt_ctx_, nullptr, "mp4", filename_.c_str());
        if (!fmt_ctx_) {
            std::cerr << "VideoFileWriter: cannot allocate output context" << std::endl;
            return false;
        }

        // Create video stream (no codec - just container for H.264 stream)
        video_stream_ = avformat_new_stream(fmt_ctx_, nullptr);
        if (!video_stream_) {
            std::cerr << "VideoFileWriter: cannot create video stream" << std::endl;
            avformat_free_context(fmt_ctx_);
            fmt_ctx_ = nullptr;
            return false;
        }

        // Set stream parameters for H.264
        video_stream_->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        video_stream_->codecpar->codec_id = AV_CODEC_ID_H264;
        video_stream_->codecpar->width = width_;
        video_stream_->codecpar->height = height_;
        video_stream_->time_base = { 1, 1000000 };  // microseconds
        video_stream_->avg_frame_rate = { fps_, 1 };

        // Open output file
        int ret = avio_open(&fmt_ctx_->pb, filename_.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(ret, errbuf, sizeof(errbuf));
            std::cerr << "VideoFileWriter: cannot open file: " << errbuf << std::endl;
            avformat_free_context(fmt_ctx_);
            fmt_ctx_ = nullptr;
            return false;
        }

        bytes_written_ = 0;
        video_pts_ = 0;
        start_time_ = std::chrono::steady_clock::now();
        is_open_ = true;
        header_written_ = false;
        first_frame_written_ = false;

        std::cout << "VideoFileWriter: opened MP4 " << filename_ << std::endl;
        return true;
    }

    // Write H.264 frame to MP4 with real timestamp
    bool writeFrame(const std::vector<unsigned char>& data, std::chrono::steady_clock::time_point frame_time = std::chrono::steady_clock::now()) {
        if (!is_open_ || !fmt_ctx_) {
            std::cerr << "VideoFileWriter: file not open" << std::endl;
            return false;
        }

        // Record first frame timestamp as reference
        if (!first_frame_written_) {
            first_frame_time_ = frame_time;
            first_frame_written_ = true;
        }

        // Calculate PTS based on real time delta from first frame
        auto delta = std::chrono::duration_cast<std::chrono::microseconds>(frame_time - first_frame_time_);
        video_pts_ = delta.count();

        // If header not written yet, we need to extract SPS/PPS first
        if (!header_written_) {
            // Find all NAL units in this frame
            std::vector<uint8_t> sps_data, pps_data, idr_data;
            bool has_sps = false, has_pps = false, has_idr = false;

            size_t i = 0;
            while (i + 4 < data.size()) {
                // Look for start code: 0x00 0x00 0x00 0x01
                if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1) {
                    size_t nal_start = i;
                    uint8_t nal_type = data[i+4] & 0x1F;

                    // Find next start code to determine NAL unit size
                    size_t next_start = i + 4;
                    while (next_start + 4 < data.size()) {
                        if (data[next_start] == 0 && data[next_start+1] == 0 &&
                            data[next_start+2] == 0 && data[next_start+3] == 1) {
                            break;
                        }
                        next_start++;
                    }
                    if (next_start + 4 >= data.size()) {
                        next_start = data.size();
                    }

                    // Extract this NAL unit
                    if (nal_type == 7) {  // SPS
                        sps_data.assign(data.begin() + nal_start, data.begin() + next_start);
                        has_sps = true;
                    } else if (nal_type == 8) {  // PPS
                        pps_data.assign(data.begin() + nal_start, data.begin() + next_start);
                        has_pps = true;
                    } else if (nal_type == 5) {  // IDR
                        idr_data.assign(data.begin() + nal_start, data.begin() + next_start);
                        has_idr = true;
                    }

                    i = next_start;
                } else {
                    i++;
                }
            }

            // We need SPS+PPS+IDR to write header
            if (has_sps && has_pps && has_idr) {
                std::cout << "VideoFileWriter: Found SPS (" << sps_data.size()
                          << " bytes), PPS (" << pps_data.size()
                          << " bytes), IDR (" << idr_data.size() << " bytes)" << std::endl;

                // Combine SPS and PPS for extradata
                std::vector<uint8_t> extradata;
                extradata.insert(extradata.end(), sps_data.begin(), sps_data.end());
                extradata.insert(extradata.end(), pps_data.begin(), pps_data.end());

                // Set extradata in stream codecpar
                video_stream_->codecpar->extradata_size = extradata.size();
                video_stream_->codecpar->extradata = (uint8_t*)av_malloc(extradata.size() + AV_INPUT_BUFFER_PADDING_SIZE);
                if (!video_stream_->codecpar->extradata) {
                    std::cerr << "VideoFileWriter: cannot allocate extradata" << std::endl;
                    return false;
                }
                memcpy(video_stream_->codecpar->extradata, extradata.data(), extradata.size());
                memset(video_stream_->codecpar->extradata + extradata.size(), 0, AV_INPUT_BUFFER_PADDING_SIZE);

                // NOW write MP4 header with SPS/PPS in extradata
                int ret = avformat_write_header(fmt_ctx_, nullptr);
                if (ret < 0) {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    std::cerr << "VideoFileWriter: cannot write header: " << errbuf << std::endl;
                    return false;
                }
                header_written_ = true;
                std::cout << "VideoFileWriter: MP4 header written with extradata ("
                          << extradata.size() << " bytes)" << std::endl;

                // Now write ONLY the IDR frame (without SPS/PPS, as they're in extradata now)
                AVPacket* pkt = av_packet_alloc();
                if (!pkt) {
                    std::cerr << "VideoFileWriter: cannot allocate packet" << std::endl;
                    return false;
                }

                pkt->data = idr_data.data();
                pkt->size = idr_data.size();
                pkt->stream_index = video_stream_->index;
                pkt->pts = video_pts_;
                pkt->dts = video_pts_;
                pkt->flags = AV_PKT_FLAG_KEY;

                ret = av_interleaved_write_frame(fmt_ctx_, pkt);

                pkt->data = nullptr;
                pkt->size = 0;
                av_packet_free(&pkt);

                if (ret < 0) {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    std::cerr << "VideoFileWriter: write failed: " << errbuf << std::endl;
                    return false;
                }

                bytes_written_ += idr_data.size();
                return true;
            } else {
                // Skip frames until we get SPS+PPS+IDR
                // std::cout << "VideoFileWriter: Waiting for SPS+PPS+IDR (got: "
                //           << (has_sps ? "SPS " : "")
                //           << (has_pps ? "PPS " : "")
                //           << (has_idr ? "IDR " : "") << ")" << std::endl;
                return true;
            }
        }

        // Header already written - now we need to strip SPS/PPS from subsequent frames
        // and write only the actual video data
        std::vector<uint8_t> frame_without_headers;
        size_t i = 0;
        while (i + 4 < data.size()) {
            if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1) {
                size_t nal_start = i;
                uint8_t nal_type = data[i+4] & 0x1F;

                // Find next start code
                size_t next_start = i + 4;
                while (next_start + 4 < data.size()) {
                    if (data[next_start] == 0 && data[next_start+1] == 0 &&
                        data[next_start+2] == 0 && data[next_start+3] == 1) {
                        break;
                    }
                    next_start++;
                }
                if (next_start + 4 >= data.size()) {
                    next_start = data.size();
                }

                // Skip SPS (7) and PPS (8), keep everything else (IDR=5, non-IDR=1)
                if (nal_type != 7 && nal_type != 8) {
                    frame_without_headers.insert(frame_without_headers.end(),
                                                data.begin() + nal_start,
                                                data.begin() + next_start);
                }

                i = next_start;
            } else {
                i++;
            }
        }

        // If frame is empty after stripping headers, skip it
        if (frame_without_headers.empty()) {
            return true;
        }

        // Create packet with stripped data
        AVPacket* pkt = av_packet_alloc();
        if (!pkt) {
            std::cerr << "VideoFileWriter: cannot allocate packet" << std::endl;
            return false;
        }

        pkt->data = frame_without_headers.data();
        pkt->size = frame_without_headers.size();
        pkt->stream_index = video_stream_->index;
        pkt->pts = video_pts_;
        pkt->dts = video_pts_;

        // Detect keyframes (NAL unit type 5 for H.264 IDR)
        if (frame_without_headers.size() > 4) {
            uint8_t nal_type = (frame_without_headers[4] & 0x1F);
            if (nal_type == 5) {
                pkt->flags |= AV_PKT_FLAG_KEY;
            }
        }

        int ret = av_interleaved_write_frame(fmt_ctx_, pkt);

        pkt->data = nullptr;
        pkt->size = 0;
        av_packet_free(&pkt);

        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(ret, errbuf, sizeof(errbuf));
            std::cerr << "VideoFileWriter: write failed: " << errbuf << std::endl;
            return false;
        }

        bytes_written_ += frame_without_headers.size();
        return true;
    }

    // Close MP4 file
    void close() {
        if (is_open_ && fmt_ctx_) {
            // Only write trailer if header was written
            if (header_written_) {
                av_write_trailer(fmt_ctx_);
            } else {
                std::cout << "VideoFileWriter: closing without trailer (header was never written)" << std::endl;
            }

            // Close file
            if (fmt_ctx_->pb) {
                avio_closep(&fmt_ctx_->pb);
            }

            // Free format context
            avformat_free_context(fmt_ctx_);
            fmt_ctx_ = nullptr;
            video_stream_ = nullptr;
            is_open_ = false;
            header_written_ = false;

            auto duration = std::chrono::steady_clock::now() - start_time_;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
            float mb_written = bytes_written_ / (1024.0f * 1024.0f);

            std::cout << "VideoFileWriter: closed MP4 " << filename_
                      << " (" << mb_written << " MB, " << seconds << " sec)" << std::endl;
        }
    }

    bool isOpen() const { return is_open_; }
    std::string getFilename() const { return filename_; }
    size_t getBytesWritten() const { return bytes_written_; }
};

// ============================================================================
// VideoUploader - Async video file uploader to Telegram
// ============================================================================
class VideoUploader {
private:
    struct UploadTask {
        std::string filename;
        int attempt_count;
        std::chrono::steady_clock::time_point next_retry_time;

        UploadTask(const std::string& fname = "")
            : filename(fname), attempt_count(0),
              next_retry_time(std::chrono::steady_clock::now()) {}
    };

    std::unique_ptr<TelegramBotApi> bot_api_;
    std::string chat_id_;

    // Configuration
    bool enabled_;
    int max_retries_;
    int retry_interval_sec_;
    std::string failed_videos_dir_;
    int max_failed_files_;

    // Async queue
    std::queue<UploadTask> upload_queue_;
    std::vector<UploadTask> retry_queue_;  // Failed uploads waiting for retry
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};

    // Read file into memory
    bool readFile(const std::string& filename, std::vector<unsigned char>& data) {
        std::ifstream file(filename, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            std::cerr << "VideoUploader: Cannot open file: " << filename << std::endl;
            return false;
        }

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        data.resize(size);
        if (!file.read(reinterpret_cast<char*>(data.data()), size)) {
            std::cerr << "VideoUploader: Failed to read file: " << filename << std::endl;
            return false;
        }

        std::cout << "VideoUploader: Loaded file " << filename
                  << " (" << (size / (1024.0 * 1024.0)) << " MB)" << std::endl;
        return true;
    }

    // Move file to failed directory (copy from tmpfs to SD, then delete)
    bool moveToFailed(const std::string& filename) {
        namespace fs = std::filesystem;

        try {
            // Create failed directory if it doesn't exist
            if (!fs::exists(failed_videos_dir_)) {
                fs::create_directories(failed_videos_dir_);
            }

            // Extract filename from path
            fs::path src_path(filename);
            std::string base_filename = src_path.filename().string();
            fs::path dest_path = fs::path(failed_videos_dir_) / base_filename;

            // Copy file from tmpfs to SD (copy instead of rename for cross-filesystem support)
            fs::copy_file(src_path, dest_path, fs::copy_options::overwrite_existing);

            // Delete original file from tmpfs
            fs::remove(src_path);

            std::cout << "VideoUploader: Moved failed upload to " << dest_path.string() << std::endl;

            // Clean up old failed files if exceeding max_failed_files_
            cleanupFailedFiles();
            return true;

        } catch (const fs::filesystem_error& e) {
            std::cerr << "VideoUploader: Failed to move file to failed directory: "
                      << e.what() << std::endl;
            return false;
        }
    }

    // Clean up old failed files
    void cleanupFailedFiles() {
        namespace fs = std::filesystem;

        try {
            // Check if directory exists
            if (!fs::exists(failed_videos_dir_) || !fs::is_directory(failed_videos_dir_)) {
                return;
            }

            // Collect all .mp4 files with their modification times
            std::vector<std::pair<fs::path, fs::file_time_type>> files;
            for (const auto& entry : fs::directory_iterator(failed_videos_dir_)) {
                if (entry.is_regular_file() && entry.path().extension() == ".mp4") {
                    files.push_back({entry.path(), fs::last_write_time(entry)});
                }
            }

            int file_count = files.size();
            if (file_count > max_failed_files_) {
                // Sort by modification time (newest first)
                std::sort(files.begin(), files.end(),
                    [](const auto& a, const auto& b) {
                        return a.second > b.second;
                    });

                // Delete oldest files
                int to_delete = file_count - max_failed_files_;
                int deleted = 0;
                for (size_t i = max_failed_files_; i < files.size(); ++i) {
                    if (fs::remove(files[i].first)) {
                        deleted++;
                    }
                }

                if (deleted > 0) {
                    std::cout << "VideoUploader: Cleaned up " << deleted << " old failed files" << std::endl;
                }
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "VideoUploader: Failed to cleanup files: " << e.what() << std::endl;
        }
    }

    // Calculate video duration from file
    int getVideoDuration(const std::string& filename) {
        // Simple estimation: parse filename for recording duration
        // Format: motion_YYYY-MM-DD_HH-MM-SS.mp4
        // For now, return 0 to let Telegram auto-detect
        return 0;
    }

    // Worker thread loop
    void workerLoop() {
        while (running_) {
            UploadTask task;
            bool has_task = false;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);

                // Wait for new task or retry time
                queue_cv_.wait_for(lock, std::chrono::seconds(1), [this] {
                    return !upload_queue_.empty() || !running_;
                });

                if (!running_ && upload_queue_.empty() && retry_queue_.empty()) {
                    break;
                }

                // Check retry queue first
                auto now = std::chrono::steady_clock::now();
                for (auto it = retry_queue_.begin(); it != retry_queue_.end(); ) {
                    if (now >= it->next_retry_time) {
                        task = *it;
                        has_task = true;
                        it = retry_queue_.erase(it);
                        break;
                    } else {
                        ++it;
                    }
                }

                // If no retry, check upload queue
                if (!has_task && !upload_queue_.empty()) {
                    task = upload_queue_.front();
                    upload_queue_.pop();
                    has_task = true;
                }
            }

            if (has_task) {
                processUpload(task);
            }
        }

        std::cout << "VideoUploader worker thread stopped" << std::endl;
    }

    // Process single upload task
    void processUpload(UploadTask& task) {
        task.attempt_count++;

        std::cout << "VideoUploader: Processing " << task.filename
                  << " (attempt " << task.attempt_count << "/" << max_retries_ << ")" << std::endl;

        // Read file
        std::vector<unsigned char> video_data;
        if (!readFile(task.filename, video_data)) {
            std::cerr << "VideoUploader: Failed to read file, moving to failed" << std::endl;
            moveToFailed(task.filename);
            return;
        }

        // Get video duration
        int duration = getVideoDuration(task.filename);

        // Extract filename for caption
        size_t last_slash = task.filename.find_last_of("/\\");
        std::string base_filename = (last_slash == std::string::npos)
            ? task.filename : task.filename.substr(last_slash + 1);

        std::string caption = "Motion detected: " + base_filename;

        // Send to Telegram
        bool success = bot_api_->sendVideo(chat_id_, video_data, duration, caption);

        if (success) {
            std::cout << "VideoUploader: Successfully uploaded " << task.filename << std::endl;

            // Delete file after successful upload
            if (std::filesystem::remove(task.filename)) {
                std::cout << "VideoUploader: Deleted " << task.filename << std::endl;
            } else {
                std::cerr << "VideoUploader: Failed to delete " << task.filename << std::endl;
            }
        } else {
            std::cerr << "VideoUploader: Failed to upload " << task.filename << std::endl;

            // Retry or move to failed
            if (task.attempt_count < max_retries_) {
                // Schedule retry
                task.next_retry_time = std::chrono::steady_clock::now()
                    + std::chrono::seconds(retry_interval_sec_);

                std::lock_guard<std::mutex> lock(queue_mutex_);
                retry_queue_.push_back(task);

                std::cout << "VideoUploader: Scheduled retry for " << task.filename
                          << " in " << retry_interval_sec_ << " seconds" << std::endl;
            } else {
                std::cerr << "VideoUploader: Max retries reached, moving to failed" << std::endl;
                moveToFailed(task.filename);
            }
        }
    }

public:
    VideoUploader(
        const std::string& bot_token,
        const std::string& chat_id,
        bool enabled = true,
        int max_retries = 2,
        int retry_interval_sec = 2,
        const std::string& failed_videos_dir = "/home/pi/recordings/failed",
        int max_failed_files = 50
    ) : chat_id_(chat_id),
        enabled_(enabled),
        max_retries_(max_retries),
        retry_interval_sec_(retry_interval_sec),
        failed_videos_dir_(failed_videos_dir),
        max_failed_files_(max_failed_files) {

        if (enabled_) {
            bot_api_ = std::make_unique<TelegramBotApi>(bot_token);
            std::cout << "VideoUploader initialized (chat_id: " << chat_id_
                      << ", max_retries: " << max_retries_
                      << ", retry_interval: " << retry_interval_sec_ << "s)" << std::endl;
        } else {
            std::cout << "VideoUploader disabled" << std::endl;
        }
    }

    ~VideoUploader() {
        stop();
    }

    void start() {
        if (enabled_ && !running_) {
            running_ = true;
            worker_thread_ = std::thread(&VideoUploader::workerLoop, this);
            std::cout << "VideoUploader worker thread started" << std::endl;
        }
    }

    void stop() {
        if (running_) {
            running_ = false;
            queue_cv_.notify_all();
            if (worker_thread_.joinable()) {
                worker_thread_.join();
            }
            std::cout << "VideoUploader stopped" << std::endl;
        }
    }

    // Add video file to upload queue
    void enqueueVideo(const std::string& filename) {
        if (!enabled_) {
            return;
        }

        std::lock_guard<std::mutex> lock(queue_mutex_);
        upload_queue_.emplace(filename);
        queue_cv_.notify_one();

        std::cout << "VideoUploader: Enqueued " << filename
                  << " (queue size: " << upload_queue_.size() << ")" << std::endl;
    }

    bool isEnabled() const { return enabled_; }
};

// ============================================================================
// Video Recording Manager - manages recording state and file writing
// ============================================================================
class VideoRecordingManager {
private:
    enum class State {
        IDLE,           // No motion, accumulating preroll
        RECORDING,      // Motion detected, recording to file
        TAIL            // Motion ended, recording tail duration
    };

    State state_;
    CircularVideoBuffer preroll_buffer_;
    VideoFileWriter file_writer_;

    // Video uploader (optional)
    VideoUploader* video_uploader_;

    // Configuration
    int preroll_sec_;
    int duration_sec_;
    int tail_duration_sec_;
    std::string output_dir_;
    int width_;
    int height_;
    int fps_;

    // Timing
    std::chrono::steady_clock::time_point recording_start_time_;
    std::chrono::steady_clock::time_point last_motion_time_;
    std::chrono::steady_clock::time_point tail_start_time_;

    // Enable/disable flag
    bool enabled_ = true;

public:
    VideoRecordingManager(
        const Config::VideoRecording& config,
        int width = 1920,
        int height = 1080,
        int fps = 30,
        VideoUploader* video_uploader = nullptr
    ) : state_(State::IDLE),
        preroll_buffer_(config.max_memory_mb),
        video_uploader_(video_uploader),
        preroll_sec_(config.preroll_sec),
        duration_sec_(config.duration_sec),
        tail_duration_sec_(config.tail_duration),
        output_dir_("/tmp/recordings"),  // Use tmpfs (RAM) to preserve SD card
        width_(width),
        height_(height),
        fps_(fps) {

        // Set resolution for file writer
        file_writer_.setResolution(width_, height_, fps_);

        std::cout << "VideoRecordingManager initialized: "
                  << width_ << "x" << height_ << "@" << fps_ << "fps, "
                  << "preroll=" << preroll_sec_ << "s, "
                  << "duration=" << duration_sec_ << "s, "
                  << "tail=" << tail_duration_sec_ << "s"
                  << (video_uploader_ ? " (Telegram upload enabled)" : "")
                  << std::endl;
    }

    ~VideoRecordingManager() {
        if (file_writer_.isOpen()) {
            file_writer_.close();
        }
    }

    // Process H.264 frame with timestamp
    void processFrame(const std::vector<unsigned char>& data, bool is_keyframe, bool motion_detected, std::chrono::steady_clock::time_point frame_time = std::chrono::steady_clock::now()) {
        // If disabled, don't process frames
        if (!enabled_) {
            return;
        }

        auto now = std::chrono::steady_clock::now();

        switch (state_) {
            case State::IDLE: {
                // Always add to preroll buffer
                preroll_buffer_.addFrame(data, is_keyframe);

                if (motion_detected) {
                    // Start recording
                    startRecording();
                }
                break;
            }

            case State::RECORDING: {
                // Write frame to file with real timestamp
                file_writer_.writeFrame(data, frame_time);

                if (motion_detected) {
                    // Update last motion time
                    last_motion_time_ = now;
                }

                // Check if we should stop recording
                auto recording_duration = std::chrono::duration_cast<std::chrono::seconds>(now - recording_start_time_);
                if (recording_duration.count() >= duration_sec_) {
                    stopRecording();
                    break;
                }

                // If no recent motion, enter tail state
                if (!motion_detected) {
                    auto time_since_motion = std::chrono::duration_cast<std::chrono::seconds>(now - last_motion_time_);
                    if (time_since_motion.count() > 0) {
                        enterTailState();
                    }
                }
                break;
            }

            case State::TAIL: {
                // Write frame to file with real timestamp
                file_writer_.writeFrame(data, frame_time);

                if (motion_detected) {
                    // Motion detected again, back to recording
                    state_ = State::RECORDING;
                    last_motion_time_ = now;
                    std::cout << "VideoRecordingManager: motion resumed, back to RECORDING" << std::endl;
                    break;
                }

                // Check if tail duration exceeded
                auto tail_duration = std::chrono::duration_cast<std::chrono::seconds>(now - tail_start_time_);
                if (tail_duration.count() >= tail_duration_sec_) {
                    stopRecording();
                }
                break;
            }
        }
    }

    bool isRecording() const {
        return state_ == State::RECORDING || state_ == State::TAIL;
    }

    std::string getCurrentFilename() const {
        return file_writer_.getFilename();
    }

    void setEnabled(bool enabled) {
        enabled_ = enabled;
        if (!enabled && isRecording()) {
            // Stop current recording if disabled
            stopRecording();
        }
        std::cout << "VideoRecordingManager: " << (enabled ? "enabled" : "disabled") << std::endl;
    }

    bool isEnabled() const {
        return enabled_;
    }

private:
    void startRecording() {
        std::cout << "VideoRecordingManager: starting recording" << std::endl;

        // Open new file
        if (!file_writer_.open(output_dir_)) {
            std::cerr << "Failed to start recording" << std::endl;
            return;
        }

        // Write preroll frames with their original timestamps
        auto preroll_frames = preroll_buffer_.getFrames(preroll_sec_);
        std::cout << "Writing " << preroll_frames.size() << " preroll frames" << std::endl;
        for (const auto& frame : preroll_frames) {
            file_writer_.writeFrame(frame.data, frame.timestamp);
        }

        // Update state
        state_ = State::RECORDING;
        recording_start_time_ = std::chrono::steady_clock::now();
        last_motion_time_ = recording_start_time_;
    }

    void enterTailState() {
        std::cout << "VideoRecordingManager: entering TAIL state" << std::endl;
        state_ = State::TAIL;
        tail_start_time_ = std::chrono::steady_clock::now();
    }

    void stopRecording() {
        std::cout << "VideoRecordingManager: stopping recording" << std::endl;

        // Get filename before closing
        std::string filename = file_writer_.getFilename();

        // Close file
        file_writer_.close();

        // Enqueue for upload if video uploader is available
        if (video_uploader_ && !filename.empty()) {
            video_uploader_->enqueueVideo(filename);
        }

        state_ = State::IDLE;
    }
};

class H264Encoder {
private:
    int encoder_fd_;
    int width_, height_;
    int gop_size_;  // GOP size (keyframe interval)
    int bitrate_;   // Bitrate in bps
    bool initialized_;
    bool sps_pps_repeat_enabled_;  // Enable SPS/PPS repeat for better streaming

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
    H264Encoder(int width, int height, int gop_size = 30, int bitrate = 10000000, bool sps_pps_repeat = true)
        : encoder_fd_(-1), width_(width), height_(height), gop_size_(gop_size), bitrate_(bitrate),
          initialized_(false), sps_pps_repeat_enabled_(sps_pps_repeat), formats_set_(false),
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

        // Enable SPS/PPS repeat with each I-frame for better streaming compatibility (if configured)
        if (sps_pps_repeat_enabled_) {
            if (!enableSPSPPSRepeat(true)) {
                std::cerr << "Failed to enable SPS/PPS repeat after streaming start" << std::endl;
            } else {
                std::cout << "SPS/PPS repeat enabled" << std::endl;
            }
        } else {
            std::cout << "SPS/PPS repeat disabled by configuration" << std::endl;
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
    // LOCK ORDER: keyframe_mutex_ must be acquired and released BEFORE clients_mutex_
    // to prevent potential deadlock with acceptClients()
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
            // LOCK ORDER: clients_mutex_ first, then keyframe_mutex_ (same order as in send())
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

// ============================================================================
// Sender Interfaces
// ============================================================================

// Base interface for all senders
class ISender {
public:
    virtual ~ISender() = default;
    virtual void start() {}
    virtual void stop() {}
};

// Interface for H.264 video senders (supports keyframe detection)
class IH264Sender : public ISender {
public:
    virtual void send(const std::vector<unsigned char>& data, bool is_keyframe) = 0;
};

// Interface for MJPEG/Photo senders (no keyframe concept, supports single/group sending)
class IMjpegSender : public ISender {
public:
    virtual void sendSingle(const std::vector<unsigned char>& photo) = 0;
    virtual void sendGroup(const std::vector<std::vector<unsigned char>>& photos) = 0;
};

// ============================================================================
// Composite Senders (Composite Pattern)
// ============================================================================

// Composite H.264 sender - aggregates multiple H.264 senders
class CompositeH264Sender : public IH264Sender {
private:
    std::vector<std::shared_ptr<IH264Sender>> senders_;
    std::mutex senders_mutex_;

public:
    CompositeH264Sender() = default;

    // Add a sender to the composite
    void addSender(std::shared_ptr<IH264Sender> sender) {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        senders_.push_back(std::move(sender));
    }

    // Remove all senders
    void clearSenders() {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        senders_.clear();
    }

    // Send data to all senders
    void send(const std::vector<unsigned char>& data, bool is_keyframe) override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->send(data, is_keyframe);
        }
    }

    // Start all senders
    void start() override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->start();
        }
    }

    // Stop all senders
    void stop() override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->stop();
        }
    }

    size_t getSenderCount() const {
        return senders_.size();
    }
};

// Composite MJPEG sender - aggregates multiple MJPEG senders
class CompositeMjpegSender : public IMjpegSender {
private:
    std::vector<std::shared_ptr<IMjpegSender>> senders_;
    std::mutex senders_mutex_;

public:
    CompositeMjpegSender() = default;

    // Add a sender to the composite
    void addSender(std::shared_ptr<IMjpegSender> sender) {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        senders_.push_back(std::move(sender));
    }

    // Remove all senders
    void clearSenders() {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        senders_.clear();
    }

    // Send single photo to all senders
    void sendSingle(const std::vector<unsigned char>& photo) override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->sendSingle(photo);
        }
    }

    // Send photo group to all senders
    void sendGroup(const std::vector<std::vector<unsigned char>>& photos) override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->sendGroup(photos);
        }
    }

    // Start all senders
    void start() override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->start();
        }
    }

    // Stop all senders
    void stop() override {
        std::lock_guard<std::mutex> lock(senders_mutex_);
        for (auto& sender : senders_) {
            sender->stop();
        }
    }

    size_t getSenderCount() const {
        return senders_.size();
    }
};

// ============================================================================
// Sender Implementations
// ============================================================================

// TcpBroadcastSender supports both H.264 and MJPEG formats via dual inheritance
class TcpBroadcastSender : public IH264Sender, public IMjpegSender {
private:
    std::unique_ptr<TcpBroadcaster> broadcaster_;

    // Internal method for sending raw data
    void sendRawData(const std::vector<unsigned char>& data, bool cache_as_keyframe = false) {
        if (broadcaster_) {
            broadcaster_->send(data, cache_as_keyframe);
        }
    }

public:
    TcpBroadcastSender(int server_port, int max_clients = 5) {
        broadcaster_ = std::make_unique<TcpBroadcaster>(server_port, max_clients);
    }

    // IH264Sender interface implementation - send H.264 frames
    void send(const std::vector<unsigned char>& data, bool is_keyframe) override {
        sendRawData(data, is_keyframe);
    }

    // IMjpegSender interface implementation - send single MJPEG/JPEG
    void sendSingle(const std::vector<unsigned char>& photo) override {
        sendRawData(photo, false);
    }

    // IMjpegSender interface implementation - send photo group
    void sendGroup(const std::vector<std::vector<unsigned char>>& photos) override {
        // Send photos sequentially (TCP doesn't have native group concept)
        for (const auto& photo : photos) {
            sendRawData(photo, false);
        }
    }
};

class TelegramSender : public IMjpegSender {
private:
    std::unique_ptr<TelegramBotApi> bot_api_;
    std::string chat_id_;

    // Async queue with frame dropping (now stores vector of photos)
    std::queue<std::vector<std::vector<unsigned char>>> message_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};

    size_t max_queue_size_;  // Drop oldest messages if queue grows beyond this

    // Worker thread loop - processes messages from queue
    void workerLoop() {
        while (running_) {
            std::vector<std::vector<unsigned char>> photos;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                    return !message_queue_.empty() || !running_;
                });

                if (!running_ && message_queue_.empty()) {
                    break;
                }

                if (!message_queue_.empty()) {
                    photos = std::move(message_queue_.front());
                    message_queue_.pop();
                }
            }

            if (!photos.empty()) {
                bool success = false;

                if (photos.size() == 1) {
                    // Send single photo
                    success = bot_api_->sendPhoto(chat_id_, photos[0]);
                    if (success) {
                        std::cout << "Telegram: Photo sent successfully" << std::endl;
                    } else {
                        std::cerr << "Telegram: Failed to send photo" << std::endl;
                    }
                } else if (photos.size() >= 2 && photos.size() <= 10) {
                    // Send as media group (2-10 photos)
                    success = bot_api_->sendMediaGroup(chat_id_, photos);
                    if (success) {
                        std::cout << "Telegram: Media group (" << photos.size()
                                 << " photos) sent successfully" << std::endl;
                    } else {
                        std::cerr << "Telegram: Failed to send media group" << std::endl;
                    }
                } else {
                    std::cerr << "Telegram: Invalid photo count: " << photos.size()
                             << " (must be 1-10)" << std::endl;
                }
            }
        }

        std::cout << "Telegram worker thread stopped" << std::endl;
    }

public:
    TelegramSender(const std::string& bot_token, const std::string& chat_id, size_t max_queue_size = 5)
        : chat_id_(chat_id), max_queue_size_(max_queue_size) {

        bot_api_ = std::make_unique<TelegramBotApi>(bot_token);
        std::cout << "TelegramSender initialized (chat_id: " << chat_id_
                  << ", max_queue_size: " << max_queue_size_ << ")" << std::endl;
    }

    ~TelegramSender() {
        stop();
    }

    void start() override {
        if (!running_) {
            running_ = true;
            worker_thread_ = std::thread(&TelegramSender::workerLoop, this);
            std::cout << "Telegram worker thread started" << std::endl;
        }
    }

    void stop() override {
        if (running_) {
            running_ = false;
            queue_cv_.notify_all();
            if (worker_thread_.joinable()) {
                worker_thread_.join();
            }
            std::cout << "Telegram sender stopped" << std::endl;
        }
    }

    // IMjpegSender interface implementation
    void sendSingle(const std::vector<unsigned char>& photo) override {
        std::vector<std::vector<unsigned char>> photos;
        photos.push_back(photo);
        sendGroup(photos);
    }

    void sendGroup(const std::vector<std::vector<unsigned char>>& photos) override {
        if (photos.empty()) {
            std::cerr << "Telegram: Cannot send empty photo vector" << std::endl;
            return;
        }

        if (photos.size() > 10) {
            std::cerr << "Telegram: Cannot send more than 10 photos at once (got "
                     << photos.size() << ")" << std::endl;
            return;
        }

        std::lock_guard<std::mutex> lock(queue_mutex_);

        // Drop oldest message if queue is full
        if (message_queue_.size() >= max_queue_size_) {
            message_queue_.pop();
            std::cout << "Telegram: Queue full, dropped oldest message" << std::endl;
        }

        message_queue_.push(photos);
        queue_cv_.notify_one();
    }
};

// TcpSender supports both H.264 and MJPEG formats via dual inheritance
class TcpSender : public IH264Sender, public IMjpegSender {
private:
    std::unique_ptr<TcpClient> tcp_client_;
    std::string server_ip_;
    int server_port_;
    int reconnect_interval_sec_;

    // Connection state
    std::atomic<bool> connected_{false};
    std::atomic<bool> first_frame_{true};

    // Reconnection thread
    std::thread reconnect_thread_;
    std::atomic<bool> running_{false};
    std::mutex connection_mutex_;

    // Internal method for sending raw data
    void sendRawData(const std::vector<unsigned char>& data) {
        if (!connected_) {
            // Silently skip if not connected (reconnection thread will handle it)
            return;
        }

        std::lock_guard<std::mutex> lock(connection_mutex_);

        // Try to send data
        bool success = tcp_client_->sendData(data);

        if (!success) {
            std::cerr << "TcpSender: Failed to send data, connection lost" << std::endl;
            tcp_client_->disconnect();
            connected_ = false;
            return;
        }

        // Send first frame multiple times for better streaming start
        if (first_frame_) {
            for (int i = 0; i < 3; ++i) {
                success = tcp_client_->sendData(data);
                if (!success) {
                    std::cerr << "TcpSender: Failed to send first frame multiple times" << std::endl;
                    tcp_client_->disconnect();
                    connected_ = false;
                    return;
                }
            }
            std::cout << "TcpSender: First frame sent multiple times for better start" << std::endl;
            first_frame_ = false;
        }
    }

    // Reconnection logic
    void reconnectionLoop() {
        while (running_) {
            if (!connected_) {
                std::cout << "TcpSender: Attempting to connect to "
                         << server_ip_ << ":" << server_port_ << "..." << std::endl;

                std::lock_guard<std::mutex> lock(connection_mutex_);
                if (tcp_client_->connect()) {
                    connected_ = true;
                    first_frame_ = true;
                    std::cout << "TcpSender: Connected successfully!" << std::endl;
                } else {
                    std::cout << "TcpSender: Connection failed, retrying in " << reconnect_interval_sec_ << " second(s)..." << std::endl;
                }
            }

            // Sleep for configured interval
            std::this_thread::sleep_for(std::chrono::seconds(reconnect_interval_sec_));
        }
    }

public:
    TcpSender(const std::string& server_ip, int server_port, int reconnect_interval_sec = 1)
        : server_ip_(server_ip), server_port_(server_port), reconnect_interval_sec_(reconnect_interval_sec) {

        tcp_client_ = std::make_unique<TcpClient>(server_ip_, server_port_);
        std::cout << "TcpSender initialized (reconnect_interval: " << reconnect_interval_sec_ << "s)" << std::endl;

        // Try initial connection
        if (tcp_client_->connect()) {
            connected_ = true;
            std::cout << "TcpSender: Initial connection successful" << std::endl;
        } else {
            std::cout << "TcpSender: Initial connection failed, will retry in background" << std::endl;
        }

        // Start reconnection thread
        running_ = true;
        reconnect_thread_ = std::thread(&TcpSender::reconnectionLoop, this);
    }

    ~TcpSender() {
        stop();
    }

    void stop() override {
        if (running_) {
            running_ = false;
            if (reconnect_thread_.joinable()) {
                reconnect_thread_.join();
            }

            std::lock_guard<std::mutex> lock(connection_mutex_);
            if (tcp_client_) {
                tcp_client_->disconnect();
            }
            connected_ = false;
        }
    }

    // IH264Sender interface implementation - send H.264 frames
    void send(const std::vector<unsigned char>& data, bool is_keyframe) override {
        // Note: is_keyframe is ignored in TCP client mode (no caching)
        // but could be used for future enhancements (e.g., prioritization)
        sendRawData(data);
    }

    // IMjpegSender interface implementation - send single MJPEG/JPEG
    void sendSingle(const std::vector<unsigned char>& photo) override {
        sendRawData(photo);
    }

    // IMjpegSender interface implementation - send photo group
    void sendGroup(const std::vector<std::vector<unsigned char>>& photos) override {
        // Send photos sequentially (TCP doesn't have native group concept)
        for (const auto& photo : photos) {
            sendRawData(photo);
        }
    }

    bool isConnected() const {
        return connected_;
    }
};

// H.264 file sender - saves H.264 frames to files
class H264FileSender : public IH264Sender {
private:
    std::string base_path_;
    std::string extension_;
    int frame_counter_ = 0;

public:
    H264FileSender(const std::string& path, const std::string& ext = ".h264")
        : base_path_(path), extension_(ext) {}

    void send(const std::vector<unsigned char>& data, bool is_keyframe) override {
        // Optional: save only keyframes by checking is_keyframe
        std::string filename = base_path_ + "/frame_" + std::to_string(frame_counter_++) + extension_;
        std::ofstream file(filename, std::ios::binary);
        file.write(reinterpret_cast<const char*>(data.data()), data.size());
    }
};

// MJPEG file sender - saves JPEG images to files
class MjpegFileSender : public IMjpegSender {
private:
    std::string base_path_;
    std::string extension_;
    int frame_counter_ = 0;

public:
    MjpegFileSender(const std::string& path, const std::string& ext = ".jpg")
        : base_path_(path), extension_(ext) {}

    void sendSingle(const std::vector<unsigned char>& photo) override {
        std::string filename = base_path_ + "/photo_" + std::to_string(frame_counter_++) + extension_;
        std::ofstream file(filename, std::ios::binary);
        file.write(reinterpret_cast<const char*>(photo.data()), photo.size());
    }

    void sendGroup(const std::vector<std::vector<unsigned char>>& photos) override {
        // Save each photo in the group
        for (const auto& photo : photos) {
            sendSingle(photo);
        }
    }
};

// ============================================================================
// Frame Handlers
// ============================================================================

// Handler for MJPEG frames - supports separate Telegram (burst) and TCP (streaming) senders
class MjpegFrameHandler {
private:
    std::shared_ptr<IMjpegSender> telegram_sender_;  // For burst mode (Telegram)
    std::shared_ptr<IMjpegSender> tcp_sender_;       // For streaming mode (TCP)

public:
    // Constructor with separate senders
    explicit MjpegFrameHandler(
        std::shared_ptr<IMjpegSender> telegram_sender,
        std::shared_ptr<IMjpegSender> tcp_sender = nullptr
    ) : telegram_sender_(telegram_sender), tcp_sender_(tcp_sender) {}

    // Backward compatibility - send single photo to Telegram
    void operator()(const std::vector<unsigned char>& mjpeg_data) {
        sendToTelegram(mjpeg_data);
    }

    // Backward compatibility - send photo group to Telegram
    void operator()(const std::vector<std::vector<unsigned char>>& photos) {
        sendBurstToTelegram(photos);
    }

    // Send single JPEG frame to TCP (streaming mode)
    void sendToTcp(const std::vector<unsigned char>& jpeg_data) {
        if (tcp_sender_) {
            tcp_sender_->sendSingle(jpeg_data);
        }
    }

    // Send burst (photo group) to Telegram
    void sendBurstToTelegram(const std::vector<std::vector<unsigned char>>& photos) {
        if (telegram_sender_) {
            telegram_sender_->sendGroup(photos);
        }
    }

    // Send single photo to Telegram
    void sendToTelegram(const std::vector<unsigned char>& jpeg_data) {
        if (telegram_sender_) {
            telegram_sender_->sendSingle(jpeg_data);
        }
    }

    bool hasTcpSender() const {
        return tcp_sender_ != nullptr;
    }
};

// Handler for H.264 frames - uses IH264Sender interface
class H264FrameHandler {
private:
    std::shared_ptr<IH264Sender> sender_;

public:
    explicit H264FrameHandler(std::shared_ptr<IH264Sender> sender) : sender_(sender) {}

    void operator()(const std::vector<unsigned char>& h264_data, bool is_keyframe) {
        sender_->send(h264_data, is_keyframe);
    }
};

class CapturerV2 {
private:
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;

    // Frame handlers
    MjpegFrameHandler mjpeg_handler_;
    H264FrameHandler h264_handler_;

    // Encoders
    std::unique_ptr<JpegEncoder> jpeg_encoder_;
    std::unique_ptr<H264Encoder> h264_encoder_;
    bool use_h264_ = false;

    // Output format control
    bool use_mjpeg_ = true;
    bool use_h264_output_ = false;

    // TCP format selection (true = H.264, false = MJPEG)
    bool tcp_uses_h264_ = true;

    // Motion detection
    std::unique_ptr<MotionDetector> motion_detector_;

    // Video recording
    std::unique_ptr<VideoRecordingManager> video_recorder_;

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

    // Burst photo buffer for media group sending
    std::vector<std::vector<unsigned char>> burst_photo_buffer_;
    int burst_photo_target_ = 5;  // Collect 5 photos before sending
    int burst_counter_ = 0;           // Burst photo counter
    int burst_packets_sent_during_recording_ = 0;  // Number of burst packets sent while recording
    int max_burst_packets_during_recording_ = 3;   // Max allowed burst packets during recording

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
    CapturerV2(MjpegFrameHandler mjpeg_handler, H264FrameHandler h264_handler,
               int width = 1920, int height = 1080,
               int h264_gop_size = 60, int h264_bitrate = 5000000, bool h264_sps_pps_repeat = true,
               const Config::VideoRecording* video_rec_config = nullptr,
               VideoUploader* video_uploader = nullptr)
        : cm_(std::make_unique<CameraManager>()),
          mjpeg_handler_(std::move(mjpeg_handler)),
          h264_handler_(std::move(h264_handler)),
          last_motion_time_(std::chrono::steady_clock::now()),
          last_jpeg_encode_time_(std::chrono::steady_clock::now()),
          width_(width),
          height_(height) {

        // Initialize encoders
        jpeg_encoder_ = std::make_unique<JpegEncoder>(width_, height_, 90); // Quality=90
        h264_encoder_ = std::make_unique<H264Encoder>(width_, height_, h264_gop_size, h264_bitrate, h264_sps_pps_repeat);

        // Initialize motion detector
        motion_detector_ = std::make_unique<MotionDetector>(width_, height_, 4); // Downsample factor 4

        // Initialize video recorder if enabled
        if (video_rec_config && video_rec_config->enabled) {
            video_recorder_ = std::make_unique<VideoRecordingManager>(*video_rec_config, width, height, current_fps_, video_uploader);
        }
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

        // You can now check TCP format inside this method:
        // if (tcp_uses_h264_) {
        //     // Logic specific to H.264 TCP format
        // } else {
        //     // Logic specific to MJPEG TCP format
        // }
        // Or use: getTcpFormat() returns "H.264" or "MJPEG"
        // Or use: isTcpFormatH264() returns true/false

        // Motion detection BEFORE encoding (using Y plane only)
        // bool should_record = true;
        bool motion = false;
        bool should_detect = false;
        if (motion_detector_ && first_plane.length >= 0) {
            // Frame skipping optimization - only process every N-th frame
            should_detect = (motion_frame_counter_ % motion_frame_skip_ == 0);
            motion_frame_counter_ = (motion_frame_counter_ + 1) % motion_frame_skip_;

            if (should_detect) {
                motion = motion_detector_->detectMotion(static_cast<unsigned char*>(mapped_data), first_plane.length);
            } 
            // else {
            //     // Use last known motion state when skipping frames
            //     motion = motion_detector_->isMotionDetected();
            // }

            if (motion) {
            //     // Update last motion time
            //     last_motion_time_ = std::chrono::steady_clock::now();

                std::cout << "[MOTION] Detected! Level: " << std::fixed << std::setprecision(2)
                          << motion_detector_->getMotionLevel() << "%, Pixels: "
                          << motion_detector_->getMotionPixelCount() << std::endl;
            } 
            // else {
            //     // Check if we're still in the "tail" recording period
            //     auto now = std::chrono::steady_clock::now();
            //     auto time_since_motion = std::chrono::duration_cast<std::chrono::seconds>(now - last_motion_time_);

            //     if (time_since_motion > motion_tail_duration_) {
            //         should_record = false;
            //     }
            // }
        }

        // Try H.264 encoding and output if enabled
        if (use_h264_output_ && use_h264_ && h264_encoder_) {
            std::vector<unsigned char> h264_data;
            bool is_keyframe = false;

            if (h264_encoder_->encodeDMA(first_plane.fd.get(), h264_data, is_keyframe)) {
                // Capture timestamp immediately after encoding
                auto frame_timestamp = std::chrono::steady_clock::now();

                h264_handler_(h264_data, is_keyframe);

                // Record video to file if video_recorder is enabled with real timestamp
                if (video_recorder_) {
                    video_recorder_->processFrame(h264_data, is_keyframe, motion, frame_timestamp);
                }
            } else {
                std::cout << "H.264 DMA encoding failed" << std::endl;
            }
        }

        // JPEG encoding and output if enabled
        // Collect photos when motion is detected at should_detect intervals
        std::vector<unsigned char> jpeg_data = {};

        // Check if we're recording video
        bool is_recording = video_recorder_ && video_recorder_->isRecording();

        // Reset burst packet counter when recording ends
        static bool was_recording = false;
        if (was_recording && !is_recording) {
            burst_packets_sent_during_recording_ = 0;
            std::cout << "Recording ended, reset burst packet counter" << std::endl;
        }
        was_recording = is_recording;

        // Check if enough time has passed since last JPEG encode
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_encode = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_jpeg_encode_time_);
        bool can_encode = time_since_last_encode >= jpeg_encode_interval_;

        // Check if we can send burst packets (not recording or under limit)
        bool can_send_burst = !is_recording || (burst_packets_sent_during_recording_ < max_burst_packets_during_recording_);

        if (((motion && should_detect && use_mjpeg_ && can_encode && can_send_burst) ||
                (burst_counter_ < burst_photo_target_ && burst_counter_ > 0 && use_mjpeg_))) {
            burst_counter_ = (burst_counter_ + 1) % burst_photo_target_;
            if (jpeg_encoder_->encode(static_cast<unsigned char*>(mapped_data), total_yuv_size, jpeg_data)) {
                last_jpeg_encode_time_ = now;  // Update last encode time

                // Add to burst buffer
                burst_photo_buffer_.push_back(jpeg_data);
                std::cout << "Burst buffer: " << burst_photo_buffer_.size() << "/" << burst_photo_target_;
                if (is_recording) {
                    std::cout << " (recording, packets sent: " << burst_packets_sent_during_recording_
                              << "/" << max_burst_packets_during_recording_ << ")";
                }
                std::cout << std::endl;

                // Send when buffer is full
                if (burst_photo_buffer_.size() >= static_cast<size_t>(burst_photo_target_)) {
                    std::cout << "Sending burst of " << burst_photo_buffer_.size() << " photos to Telegram" << std::endl;
                    mjpeg_handler_.sendBurstToTelegram(burst_photo_buffer_);
                    burst_photo_buffer_.clear();

                    // Increment counter if recording
                    if (is_recording) {
                        burst_packets_sent_during_recording_++;
                        std::cout << "Burst packets sent during recording: " << burst_packets_sent_during_recording_
                                  << "/" << max_burst_packets_during_recording_ << std::endl;
                    }
                }
            }
        }

        // MJPEG TCP streaming - send every frame to TCP
        if (!tcp_uses_h264_ && mjpeg_handler_.hasTcpSender()) {
            // Reuse already encoded JPEG if available, otherwise encode new one
            if (!jpeg_data.empty() || jpeg_encoder_->encode(static_cast<unsigned char*>(mapped_data), total_yuv_size, jpeg_data)) {
                mjpeg_handler_.sendToTcp(jpeg_data);
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

    // TCP format control
    void setTcpFormat(bool use_h264) {
        tcp_uses_h264_ = use_h264;
        std::cout << "TCP format set to " << (use_h264 ? "H.264" : "MJPEG") << std::endl;
    }

    bool isTcpFormatH264() const {
        return tcp_uses_h264_;
    }

    std::string getTcpFormat() const {
        return tcp_uses_h264_ ? "H.264" : "MJPEG";
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

    float getMotionThreshold() const {
        return motion_detector_ ? motion_detector_->getThreshold() : 0.0f;
    }

    float getMotionAreaThreshold() const {
        return motion_detector_ ? motion_detector_->getMotionAreaThreshold() : 0.0f;
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

    void setBurstPhotoTarget(int count) {
        if (count < 1 || count > 10) {
            std::cerr << "Burst photo count must be 1-10, using default 5" << std::endl;
            count = 5;
        }
        burst_photo_target_ = count;
        burst_photo_buffer_.clear();
        std::cout << "Burst photo target set to " << count << std::endl;
    }

    int getBurstPhotoTarget() const {
        return burst_photo_target_;
    }

    void setMaxBurstPacketsDuringRecording(int max_packets) {
        max_burst_packets_during_recording_ = max_packets;
        std::cout << "Max burst packets during recording set to " << max_packets << std::endl;
    }

    int getMaxBurstPacketsDuringRecording() const {
        return max_burst_packets_during_recording_;
    }

    // Enable/disable video recording
    void enableVideoRecording(bool enable) {
        if (!video_recorder_) {
            std::cerr << "Video recording manager not initialized" << std::endl;
            return;
        }

        video_recorder_->setEnabled(enable);
    }

    bool isVideoRecordingEnabled() const {
        return video_recorder_ != nullptr;
    }

    bool isVideoRecordingActive() const {
        return video_recorder_ && video_recorder_->isRecording();
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

    // Load configuration
    std::string config_file = "config.toml";
    if (argc >= 2) {
        config_file = argv[1];
    }

    Config config = Config::load(config_file);
    config.print();

    try {
        bool use_h264_for_tcp = (config.tcp.format == "h264");

        // ========================================================================
        // Setup MJPEG Sender (Telegram)
        // ========================================================================
        std::shared_ptr<TelegramSender> telegram_sender;

        if (config.telegram.enabled) {
            telegram_sender = std::make_shared<TelegramSender>(
                config.telegram.bot_token,
                config.telegram.chat_id,
                config.telegram.max_queue_size
            );
            telegram_sender->start();
        }

        // ========================================================================
        // Setup Video Uploader (Telegram)
        // ========================================================================
        std::shared_ptr<VideoUploader> video_uploader;

        if (config.telegram.enabled && config.video_recording.enabled && config.video_recording.send_to_telegram) {
            video_uploader = std::make_shared<VideoUploader>(
                config.telegram.bot_token,
                config.telegram.chat_id,
                true,  // enabled
                config.video_recording.max_retries,
                config.video_recording.retry_interval_sec,
                config.video_recording.failed_videos_dir,
                config.video_recording.max_failed_files
            );
            video_uploader->start();
            std::cout << "VideoUploader enabled for Telegram" << std::endl;
        }

        // ========================================================================
        // Setup TCP Senders (Broadcaster + Client)
        // ========================================================================
        std::shared_ptr<TcpBroadcastSender> tcp_broadcaster;
        std::shared_ptr<TcpSender> tcp_client;

        if (config.tcp.broadcast.enabled) {
            tcp_broadcaster = std::make_shared<TcpBroadcastSender>(
                config.tcp.broadcast.port,
                config.tcp.broadcast.max_clients
            );
            std::cout << "TCP Broadcaster enabled on port " << config.tcp.broadcast.port
                      << " (max clients: " << config.tcp.broadcast.max_clients << ")" << std::endl;
        }

        if (config.tcp.client.enabled) {
            tcp_client = std::make_shared<TcpSender>(
                config.tcp.client.remote_ip,
                config.tcp.client.remote_port,
                config.tcp.client.reconnect_interval_sec
            );
            std::cout << "TCP Client connecting to " << config.tcp.client.remote_ip
                      << ":" << config.tcp.client.remote_port << std::endl;
        }

        // ========================================================================
        // Create handlers based on selected format
        // ========================================================================
        H264FrameHandler h264_handler(nullptr);
        MjpegFrameHandler mjpeg_handler(telegram_sender, nullptr);  // Default: Telegram only

        if (use_h264_for_tcp) {
            // H.264 format for TCP - Telegram gets MJPEG burst, TCP gets H.264
            auto composite_h264 = std::make_shared<CompositeH264Sender>();

            // Cast to IH264Sender and add to composite
            std::shared_ptr<IH264Sender> h264_broadcaster = tcp_broadcaster;
            composite_h264->addSender(h264_broadcaster);

            if (tcp_client) {
                std::shared_ptr<IH264Sender> h264_client = tcp_client;
                composite_h264->addSender(h264_client);
            }

            std::cout << "H.264 Composite Sender configured with "
                      << composite_h264->getSenderCount() << " sender(s)" << std::endl;

            h264_handler = H264FrameHandler(composite_h264);

            // MJPEG handler: Telegram only (no TCP sender)
            mjpeg_handler = MjpegFrameHandler(telegram_sender, nullptr);

        } else {
            // MJPEG format for TCP - separate Telegram (burst) and TCP (streaming)
            auto tcp_composite = std::make_shared<CompositeMjpegSender>();

            // Cast to IMjpegSender and add TCP senders to composite
            std::shared_ptr<IMjpegSender> mjpeg_broadcaster = tcp_broadcaster;
            tcp_composite->addSender(mjpeg_broadcaster);

            if (tcp_client) {
                std::shared_ptr<IMjpegSender> mjpeg_client = tcp_client;
                tcp_composite->addSender(mjpeg_client);
            }

            std::cout << "MJPEG TCP Composite Sender configured with "
                      << tcp_composite->getSenderCount() << " sender(s)" << std::endl;

            // MJPEG handler: Telegram for burst, TCP composite for streaming
            mjpeg_handler = MjpegFrameHandler(telegram_sender, tcp_composite);

            // Create empty H.264 handler (won't be used)
            auto empty_h264 = std::make_shared<CompositeH264Sender>();
            h264_handler = H264FrameHandler(empty_h264);
        }

        // ========================================================================
        // Create and configure capturer
        // ========================================================================
        CapturerV2 capturer(
            std::move(mjpeg_handler),
            std::move(h264_handler),
            config.camera.width,
            config.camera.height,
            config.h264.gop_size,
            config.h264.bitrate,
            config.h264.sps_pps_repeat,
            &config.video_recording,
            video_uploader.get()
        );

        if (!capturer.initialize()) {
            return -1;
        }

        if (!capturer.configure()) {
            return -1;
        }

        if (!capturer.setupBuffers()) {
            return -1;
        }

        // Set TCP format flag
        capturer.setTcpFormat(use_h264_for_tcp);

        // Configure autofocus
        if (config.autofocus.enabled) {
            capturer.setAutofocusMode(config.autofocus.mode);
            capturer.setAutofocusSpeed(config.autofocus.speed);
            capturer.setAutofocusRange(config.autofocus.range);
            capturer.enableAutofocus(true);
            std::cout << "Autofocus enabled (mode: " << config.autofocus.mode
                      << ", speed: " << config.autofocus.speed
                      << ", range: " << config.autofocus.range << ")" << std::endl;
        } else {
            capturer.enableAutofocus(false);
            std::cout << "Autofocus disabled" << std::endl;
        }

        // Configure frame rate
        capturer.setFrameRate(config.camera.fps);

        // Configure motion detection
        if (config.motion_detection.enabled) {
            capturer.enableMotionDetection(true);
            capturer.setMotionFrameSkip(config.motion_detection.frame_skip);
            capturer.setMotionThreshold(config.motion_detection.pixel_change_sensitivity);
            capturer.setMotionAreaThreshold(config.motion_detection.min_object_size);
            std::cout << "Motion detection enabled (frame_skip: " << config.motion_detection.frame_skip
                      << ", sensitivity: " << config.motion_detection.pixel_change_sensitivity
                      << ", min_object_size: " << (config.motion_detection.min_object_size * 100) << "%)" << std::endl;
        } else {
            capturer.enableMotionDetection(false);
            std::cout << "Motion detection disabled" << std::endl;
        }

        // Configure MJPEG burst photo collection
        capturer.setBurstPhotoTarget(config.mjpeg.burst_photo_count);
        capturer.setJpegEncodeInterval(config.mjpeg.encode_interval_ms);
        capturer.setMaxBurstPacketsDuringRecording(config.mjpeg.max_burst_packets_during_recording);

        // Configure H.264 encoder
        if (capturer.isH264Available()) {
            std::cout << "H.264 hardware encoder detected!" << std::endl;
            if (config.h264.enabled) {
                capturer.enableH264Encoding(true);
                capturer.setH264GopSize(config.h264.gop_size);
                capturer.setH264Bitrate(config.h264.bitrate, config.h264.cbr);
                std::cout << "H.264 encoding enabled (GOP: " << config.h264.gop_size
                          << ", bitrate: " << config.h264.bitrate
                          << ", mode: " << (config.h264.cbr ? "CBR" : "VBR") << ")" << std::endl;
            } else {
                capturer.enableH264Encoding(false);
                std::cout << "H.264 encoding disabled by config" << std::endl;
            }
        } else {
            std::cout << "H.264 hardware encoder not available, using JPEG only" << std::endl;
        }

        // Enable/disable output formats
        capturer.enableMjpegOutput(config.mjpeg.enabled && config.mjpeg.output_enabled);
        capturer.enableH264Output(config.h264.enabled && config.h264.output_enabled);

        if (!capturer.startCapture()) {
            return -1;
        }

        // ========================================================================
        // Setup Telegram Command Handler
        // ========================================================================
        std::shared_ptr<TelegramBotApi> bot_api;
        std::unique_ptr<TelegramCommandHandler> command_handler;

        if (config.telegram.enabled) {
            // Create TelegramBotApi instance
            bot_api = std::make_shared<TelegramBotApi>(config.telegram.bot_token);

            // Create command handler
            command_handler = std::make_unique<TelegramCommandHandler>(bot_api.get(), config.telegram.chat_id);

            // Set command callback
            command_handler->setCommandCallback([&capturer, &command_handler](const std::string& command) {
                // Parse command and arguments
                std::istringstream iss(command);
                std::string cmd;
                iss >> cmd;  // First word is the command

                if (cmd == "/motion_on") {
                    capturer.enableMotionDetection(true);
                    command_handler->sendResponse("✅ Motion detection enabled");

                } else if (cmd == "/motion_off") {
                    capturer.enableMotionDetection(false);
                    command_handler->sendResponse("🛑 Motion detection disabled");

                } else if (cmd == "/status") {
                    std::string status = "📊 Camera Status:\n\n";
                    status += "Motion detection: " + std::string(capturer.isMotionDetectionEnabled() ? "ON" : "OFF") + "\n";
                    status += "Video recording: " + std::string(capturer.isVideoRecordingActive() ? "YES" : "NO") + "\n\n";
                    status += "Motion Settings:\n";
                    status += "- Sensitivity: " + std::to_string((int)capturer.getMotionThreshold()) + "\n";
                    status += "- Min object size: " + std::to_string((int)(capturer.getMotionAreaThreshold() * 100)) + "%";
                    command_handler->sendResponse(status);

                } else if (cmd == "/sensitivity") {
                    float value;
                    if (iss >> value) {
                        if (value >= 5.0f && value <= 100.0f) {
                            capturer.setMotionThreshold(value);
                            command_handler->sendResponse("✅ Pixel sensitivity set to " + std::to_string((int)value) +
                                "\n\nLower = more sensitive\n5-15: very sensitive\n20-30: balanced\n40-100: less sensitive");
                        } else {
                            command_handler->sendResponse("❌ Value must be 5-100\nExample: /sensitivity 30");
                        }
                    } else {
                        command_handler->sendResponse("📝 Current sensitivity: " + std::to_string((int)capturer.getMotionThreshold()) +
                            "\n\nTo change: /sensitivity <5-100>\nExample: /sensitivity 30");
                    }

                } else if (cmd == "/min_size") {
                    float value;
                    if (iss >> value) {
                        if (value >= 0.5f && value <= 50.0f) {
                            capturer.setMotionAreaThreshold(value / 100.0f);
                            command_handler->sendResponse("✅ Min object size set to " + std::to_string((int)value) + "%" +
                                "\n\n1% = small (cat, bird)\n5% = medium (person)\n10% = large (car)");
                        } else {
                            command_handler->sendResponse("❌ Value must be 0.5-50 (%)\nExample: /min_size 5");
                        }
                    } else {
                        command_handler->sendResponse("📝 Current min size: " + std::to_string((int)(capturer.getMotionAreaThreshold() * 100)) + "%" +
                            "\n\nTo change: /min_size <0.5-50>\nExample: /min_size 5");
                    }

                } else if (cmd == "/help") {
                    std::string help = "🤖 Available Commands:\n\n";
                    help += "/motion_on - Enable motion detection\n";
                    help += "/motion_off - Disable motion detection\n";
                    help += "/status - Show camera status\n";
                    help += "/sensitivity <5-100> - Pixel change sensitivity\n";
                    help += "/min_size <0.5-50> - Min object size (%)\n";
                    help += "/help - Show this message";
                    command_handler->sendResponse(help);

                } else {
                    command_handler->sendResponse("❓ Unknown command. Type /help for available commands.");
                }
            });

            // Start polling
            command_handler->start();
            std::cout << "Telegram command handler started" << std::endl;
        }

        std::cout << "Press Ctrl+C to stop" << std::endl;

        // For demo purposes, run indefinitely
        for (;;) {
            sleep(1);
        }

        // Stop command handler
        if (command_handler) {
            command_handler->stop();
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
