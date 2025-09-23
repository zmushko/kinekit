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
#include <sys/mman.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

using namespace libcamera;

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
            std::cerr << "Failed to connect to server" << std::endl;
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

class RpiCapturerV2 {
private:
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::unique_ptr<TcpClient> tcp_client_;
    std::unique_ptr<JpegEncoder> jpeg_encoder_;

    // Statistics
    int frames_captured_ = 0;
    bool capture_running_ = false;

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
    RpiCapturerV2() : cm_(std::make_unique<CameraManager>()) {}
    
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

        std::cout << "Camera initialized successfully" << std::endl;
        return true;
    }
    
    bool initialize(const std::string &server_ip, int server_port) {
        tcp_client_ = std::make_unique<TcpClient>(server_ip, server_port);
        if (!tcp_client_->connect()) {
            std::cerr << "Failed to connect to TCP server" << std::endl;
            return false;
        }

        std::cout << "TCP server connection successful" << std::endl;

        jpeg_encoder_ = std::make_unique<JpegEncoder>(1920, 1080, 50); // Full HD, quality 50
        if (!jpeg_encoder_) {
            std::cerr << "Failed to initialize JPEG encoder" << std::endl;
            return false;
        }

        std::cout << "Camera initialized successfully" << std::endl;
        return initialize();
    }

    bool configure() {
        std::cout << "Configuring camera..." << std::endl;
        
        // Create configuration for frame capture
        config_ = camera_->generateConfiguration({StreamRole::StillCapture});
        
        if (!config_) {
            std::cerr << "Failed to create configuration" << std::endl;
            return false;
        }
        
        // Configure parameters for Zero 2W + Camera v3
        StreamConfiguration &stream_config = config_->at(0);
        stream_config.size = Size(1920, 1080);  // Full HD
        stream_config.pixelFormat = formats::YUV420; // Efficient format
        
        std::cout << "Stream configuration:" << std::endl;
        std::cout << "   Size: " << stream_config.size.toString() << std::endl;
        std::cout << "   Format: " << stream_config.pixelFormat.toString() << std::endl;
        
        // Validate and apply configuration
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
        
        std::cout << "Allocated " << ret << " buffers" << std::endl;
        
        // Create requests
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

            // Apply autofocus controls to request
            ControlList af_controls = buildAutofocusControls();
            request->controls() = af_controls;

            requests_.push_back(std::move(request));
        }
        
        std::cout << "Buffers set up successfully" << std::endl;
        return true;
    }
    
    bool startCapture() {
        std::cout << "Starting capture..." << std::endl;
        
        // Connect request completion handler
        camera_->requestCompleted.connect(this, &RpiCapturerV2::requestComplete);
        
        // Start camera
        if (camera_->start()) {
            std::cerr << "Failed to start camera" << std::endl;
            return false;
        }
        
        capture_running_ = true;
        
        // Queue all requests
        for (auto &request : requests_) {
            if (camera_->queueRequest(request.get())) {
                std::cerr << "Failed to queue request" << std::endl;
                return false;
            }
        }
        
        std::cout << "Capture started!" << std::endl;
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
    
    void sendFrame(const FrameBuffer *buffer) {
        // 
        std::vector<unsigned char> data_vector;
        int plane_index = 0;
        for (const FrameBuffer::Plane &plane : buffer->planes()) {
            // Check if offset is page-aligned
            size_t page_size = sysconf(_SC_PAGESIZE);
            size_t aligned_offset = (plane.offset / page_size) * page_size;
            size_t offset_diff = plane.offset - aligned_offset;
            size_t map_length = plane.length + offset_diff;

            void *mapped_data = mmap(nullptr, map_length, PROT_READ, MAP_SHARED, plane.fd.get(), aligned_offset);
            if (mapped_data == MAP_FAILED) {
                std::cerr << "mmap failed for plane " << plane_index << ": fd=" << plane.fd.get()
                         << ", length=" << map_length << ", offset=" << aligned_offset
                         << ", original_offset=" << plane.offset << ", page_size=" << page_size
                         << ", errno=" << errno << " (" << strerror(errno) << ")" << std::endl;
                continue;
            }

            // Adjust pointer to actual data start
            unsigned char* actual_data = static_cast<unsigned char*>(mapped_data) + offset_diff;

            data_vector.insert(data_vector.end(), actual_data, actual_data + plane.length);
            munmap(mapped_data, map_length);

            plane_index++;
        }

        // Encode to JPEG
        std::vector<unsigned char> jpeg_data;
        if (!jpeg_encoder_->encode(data_vector.data(), data_vector.size(), jpeg_data)) {
            std::cerr << "JPEG encoding error" << std::endl;
            return;
        }

        tcp_client_->sendData(jpeg_data);
    }
    
    // Request completion handler
    void requestComplete(Request *req) {
        if (req->status() == Request::RequestComplete) {
            frames_captured_++;
            
            // std::cout << "Frame #" << frames_captured_ << " captured successfully" << std::endl;
            
            // send frame 
            const Request::BufferMap &buffers = req->buffers();
            for (auto bufferPair : buffers) {
                FrameBuffer *buffer = bufferPair.second;
                sendFrame(buffer);
                break; // Take only the first buffer
            }
            
            // Re-queue request if capture is active
            if (capture_running_) {
                req->reuse(Request::ReuseBuffers);

                // Apply updated autofocus controls
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

    // Autofocus control methods
    void setAutofocusMode(int mode) {
        af_settings_.mode = mode;
        std::string mode_str;
        switch (mode) {
            case 0: mode_str = "Auto"; break;
            case 1: mode_str = "Manual"; break;
            case 2: mode_str = "Continuous"; break;
            default: mode_str = "Unknown"; break;
        }
        std::cout << "AF Mode set to: " << mode_str << std::endl;
    }

    void setLensPosition(float position) {
        af_settings_.lens_position = position;
        std::cout << "Lens position set to: " << position << std::endl;
    }

    void triggerAutofocus() {
        af_settings_.trigger_scan = true;
        std::cout << "Autofocus scan triggered" << std::endl;
    }

    void enableAutofocus(bool enable) {
        af_settings_.enable = enable;
        std::cout << "Autofocus " << (enable ? "enabled" : "disabled") << std::endl;
    }

    void setAutofocusSpeed(int speed) {
        af_settings_.speed = speed;
        std::string speed_str;
        switch (speed) {
            case 0: speed_str = "Normal"; break;
            case 1: speed_str = "Fast"; break;
            default: speed_str = "Unknown"; break;
        }
        std::cout << "AF Speed set to: " << speed_str << std::endl;
    }

    void setAutofocusRange(int range) {
        af_settings_.range = range;
        std::string range_str;
        switch (range) {
            case 0: range_str = "Normal"; break;
            case 1: range_str = "Macro"; break;
            case 2: range_str = "Full"; break;
            default: range_str = "Unknown"; break;
        }
        std::cout << "AF Range set to: " << range_str << std::endl;
    }

    void showAutofocusSettings() {
        std::cout << "\nAutofocus Settings:" << std::endl;
        std::string mode_str;
        switch (af_settings_.mode) {
            case 0: mode_str = "Auto"; break;
            case 1: mode_str = "Manual"; break;
            case 2: mode_str = "Continuous"; break;
            default: mode_str = "Unknown"; break;
        }
        std::cout << "   Mode: " << mode_str << std::endl;
        std::cout << "   Lens Position: " << af_settings_.lens_position << std::endl;
        std::cout << "   Enabled: " << (af_settings_.enable ? "Yes" : "No") << std::endl;
    }

private:
    ControlList buildAutofocusControls() {
        ControlList controls;

        if (!af_settings_.enable) {
            return controls;
        }

        // Set AF mode
        switch (af_settings_.mode) {
            case 0: // Auto
                controls.set(controls::AfMode, controls::AfModeAuto);
                if (af_settings_.trigger_scan) {
                    controls.set(controls::AfTrigger, controls::AfTriggerStart);
                    af_settings_.trigger_scan = false; // Reset trigger
                }
                break;
            case 1: // Manual
                controls.set(controls::AfMode, controls::AfModeManual);
                controls.set(controls::LensPosition, af_settings_.lens_position);
                break;
            case 2: // Continuous
                controls.set(controls::AfMode, controls::AfModeContinuous);
                break;
        }

        // Set AF speed for faster operation
        switch (af_settings_.speed) {
            case 0:
                controls.set(controls::AfSpeed, controls::AfSpeedNormal);
                break;
            case 1:
                controls.set(controls::AfSpeed, controls::AfSpeedFast);
                break;
        }

        // Set AF range for optimization
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
    
    ~RpiCapturerV2() {
        std::cout << "Cleaning up resources..." << std::endl;
        
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
    std::cout << "\n=== Raspberry Pi Camera Capturer v2.0 ===" << std::endl;
    std::cout << "Camera Module v3 + Pi Zero 2W + libcamera" << std::endl;
    std::cout << "================================================\n" << std::endl;
    
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <server_ip> <server_port>" << std::endl;
        return -1;
    }

    RpiCapturerV2 capturer;
    
    // Step-by-step initialization
    if (!capturer.initialize(std::string(argv[1]), std::stoi(argv[2]))) {
        return -1;
    }
    
    if (!capturer.configure()) {
        return -1;
    }
    
    if (!capturer.setupBuffers()) {
        return -1;
    }
    
    // Configure autofocus settings
    std::cout << "\n=== Configuring Autofocus ===" << std::endl;
    capturer.showAutofocusSettings();

    // Example 1: Set to Fast Continuous Autofocus
    capturer.setAutofocusMode(2);  // Continuous mode
    capturer.setAutofocusSpeed(1); // Fast speed
    capturer.setAutofocusRange(0); // Normal range (faster than Full)
    capturer.enableAutofocus(true);
    capturer.showAutofocusSettings();

    if (!capturer.startCapture()) {
        return -1;
    }

    std::cout << "\n=== Running with Continuous Autofocus ===" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;


    for (;;) {
        sleep(1);
    }
    

    // Show statistics
    capturer.showStats();
    
    // Stop capture
    capturer.stopCapture();
    
    std::cout << "\nProgram completed successfully!" << std::endl;
    return 0;
}
