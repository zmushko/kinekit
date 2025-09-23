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
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>

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

class H264Encoder {
private:
    int encoder_fd_;
    int width_, height_;
    int gop_size_;  // GOP size (keyframe interval)
    size_t yuv_buffer_size_;
    size_t h264_buffer_size_;
    unsigned char* h264_buffer_;
    bool initialized_;

    // V4L2 M2M structures
    struct v4l2_format input_format_;
    struct v4l2_format output_format_;
    struct v4l2_requestbuffers input_reqbufs_;
    struct v4l2_requestbuffers output_reqbufs_;
    bool formats_set_;

public:
    H264Encoder(int width, int height, int gop_size = 30)
        : encoder_fd_(-1), width_(width), height_(height), gop_size_(gop_size),
          yuv_buffer_size_(width * height * 3 / 2),
          h264_buffer_size_(width * height),
          h264_buffer_(nullptr), initialized_(false), formats_set_(false) {

        h264_buffer_ = new unsigned char[h264_buffer_size_];
        memset(&input_format_, 0, sizeof(input_format_));
        memset(&output_format_, 0, sizeof(output_format_));
        memset(&input_reqbufs_, 0, sizeof(input_reqbufs_));
        memset(&output_reqbufs_, 0, sizeof(output_reqbufs_));
    }

    ~H264Encoder() {
        if (encoder_fd_ >= 0) {
            close(encoder_fd_);
        }
        delete[] h264_buffer_;
    }

    bool initialize() {
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
        return setupEncoder();
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

        // Set output format (H.264)
        output_format_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_format_.fmt.pix_mp.width = width_;
        output_format_.fmt.pix_mp.height = height_;
        output_format_.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
        output_format_.fmt.pix_mp.field = V4L2_FIELD_NONE;
        output_format_.fmt.pix_mp.num_planes = 1;
        output_format_.fmt.pix_mp.plane_fmt[0].sizeimage = h264_buffer_size_;

        if (ioctl(encoder_fd_, VIDIOC_S_FMT, &output_format_) < 0) {
            std::cerr << "Failed to set output format: " << strerror(errno) << std::endl;
            return false;
        }

        // Set GOP size (keyframe interval)
        if (!setGopSize(gop_size_)) {
            std::cerr << "Failed to set GOP size, continuing with default" << std::endl;
        }

        formats_set_ = true;
        std::cout << "V4L2 M2M H.264 encoder setup completed (GOP size: " << gop_size_ << ")" << std::endl;
        return true;
    }

public:
    bool encode(const unsigned char* yuv_data, size_t yuv_size, std::vector<unsigned char>& h264_data) {
        if (!initialized_) {
            std::cerr << "H.264 encoder not initialized" << std::endl;
            return false;
        }

        // Simplified encoding - in reality this would use V4L2 M2M interface
        // For now, return empty data to indicate H.264 encoding is detected but not fully implemented
        h264_data.clear();
        std::cout << "H.264 encoding placeholder (YUV size: " << yuv_size << " bytes)" << std::endl;
        return false; // Return false to fall back to JPEG for now
    }

    // New method for zero-copy encoding using DMA file descriptor
    bool encodeDMA(int dma_fd, size_t offset, size_t length, std::vector<unsigned char>& h264_data) {
        if (!initialized_) {
            std::cerr << "H.264 encoder not initialized" << std::endl;
            return false;
        }

        std::cout << "H.264 DMA encoding: fd=" << dma_fd << ", offset=" << offset << ", length=" << length << std::endl;

        if (!formats_set_) {
            std::cerr << "V4L2 formats not set" << std::endl;
            return false;
        }

        // Request input buffers (DMABUF type for zero-copy)
        input_reqbufs_.count = 1;
        input_reqbufs_.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        input_reqbufs_.memory = V4L2_MEMORY_DMABUF;

        if (ioctl(encoder_fd_, VIDIOC_REQBUFS, &input_reqbufs_) < 0) {
            std::cerr << "Failed to request input buffers: " << strerror(errno) << std::endl;
            return false;
        }

        // Request output buffers (MMAP type for H.264 output)
        output_reqbufs_.count = 1;
        output_reqbufs_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_reqbufs_.memory = V4L2_MEMORY_MMAP;

        if (ioctl(encoder_fd_, VIDIOC_REQBUFS, &output_reqbufs_) < 0) {
            std::cerr << "Failed to request output buffers: " << strerror(errno) << std::endl;
            return false;
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
        input_planes[0].m.fd = dma_fd;
        input_planes[0].data_offset = 0;
        input_planes[0].bytesused = length;  // Full YUV420 size (3110400)
        input_planes[0].length = length;     // Buffer size

        if (ioctl(encoder_fd_, VIDIOC_QBUF, &input_buf) < 0) {
            std::cerr << "Failed to queue input buffer: " << strerror(errno) << std::endl;
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

        void* output_mem = mmap(NULL, output_planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, encoder_fd_, output_planes[0].m.mem_offset);
        if (output_mem == MAP_FAILED) {
            std::cerr << "Failed to mmap output buffer: " << strerror(errno) << std::endl;
            return false;
        }

        // Queue output buffer
        if (ioctl(encoder_fd_, VIDIOC_QBUF, &output_buf) < 0) {
            std::cerr << "Failed to queue output buffer: " << strerror(errno) << std::endl;
            munmap(output_mem, output_planes[0].length);
            return false;
        }

        // Start streaming
        int input_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        int output_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        if (ioctl(encoder_fd_, VIDIOC_STREAMON, &input_type) < 0) {
            std::cerr << "Failed to start input streaming: " << strerror(errno) << std::endl;
            munmap(output_mem, output_planes[0].length);
            return false;
        }

        if (ioctl(encoder_fd_, VIDIOC_STREAMON, &output_type) < 0) {
            std::cerr << "Failed to start output streaming: " << strerror(errno) << std::endl;
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &input_type);
            munmap(output_mem, output_planes[0].length);
            return false;
        }

        // Wait for encoding completion using poll
        struct pollfd pfd;
        pfd.fd = encoder_fd_;
        pfd.events = POLLIN;

        int poll_result = poll(&pfd, 1, 1000); // 1 second timeout
        if (poll_result <= 0) {
            std::cerr << "Encoding timeout or error" << std::endl;
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &input_type);
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &output_type);
            munmap(output_mem, output_planes[0].length);
            return false;
        }

        // Dequeue output buffer to get encoded H.264 data
        memset(&output_buf, 0, sizeof(output_buf));
        memset(output_planes, 0, sizeof(output_planes));
        output_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        output_buf.memory = V4L2_MEMORY_MMAP;
        output_buf.m.planes = output_planes;
        output_buf.length = 1;

        if (ioctl(encoder_fd_, VIDIOC_DQBUF, &output_buf) < 0) {
            std::cerr << "Failed to dequeue output buffer: " << strerror(errno) << std::endl;
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &input_type);
            ioctl(encoder_fd_, VIDIOC_STREAMOFF, &output_type);
            munmap(output_mem, output_planes[0].length);
            return false;
        }

        // Copy H.264 data from output buffer
        size_t h264_size = output_planes[0].bytesused;
        h264_data.resize(h264_size);
        memcpy(h264_data.data(), output_mem, h264_size);

        // Stop streaming
        ioctl(encoder_fd_, VIDIOC_STREAMOFF, &input_type);
        ioctl(encoder_fd_, VIDIOC_STREAMOFF, &output_type);
        munmap(output_mem, output_planes[0].length);

        std::cout << "H.264 encoding completed, size: " << h264_size << " bytes" << std::endl;
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

        ext_ctrl.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
        ext_ctrl.value = gop_size;

        ext_ctrls.count = 1;
        ext_ctrls.controls = &ext_ctrl;

        if (ioctl(encoder_fd_, VIDIOC_S_EXT_CTRLS, &ext_ctrls) < 0) {
            std::cerr << "Failed to set GOP size: " << strerror(errno) << std::endl;
            return false;
        }

        std::cout << "GOP size set to " << gop_size << " frames" << std::endl;
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
    std::unique_ptr<H264Encoder> h264_encoder_;
    bool use_h264_ = false;

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

        // Try to initialize H.264 encoder
        h264_encoder_ = std::make_unique<H264Encoder>(1920, 1080, 30);  // GOP size = 30 frames (1 sec @ 30fps)
        if (h264_encoder_->initialize()) {
            std::cout << "H.264 hardware encoder available" << std::endl;
        } else {
            std::cout << "H.264 hardware encoder not available, using JPEG" << std::endl;
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
        std::vector<unsigned char> encoded_data;
        bool encoding_success = false;

        // Debug: Show all planes
        std::cout << "Buffer has " << buffer->planes().size() << " planes:" << std::endl;
        for (size_t i = 0; i < buffer->planes().size(); i++) {
            const FrameBuffer::Plane &p = buffer->planes()[i];
            std::cout << "  Plane " << i << ": fd=" << p.fd.get()
                      << ", offset=" << p.offset << ", length=" << p.length << std::endl;
        }

        // Try zero-copy H.264 encoding with full YUV420 buffer
        if (use_h264_ && h264_encoder_) {
            // Calculate total YUV420 size: Y + U + V
            size_t total_yuv_size = 0;
            for (const auto &plane : buffer->planes()) {
                total_yuv_size += plane.length;
            }

            // Use the DMA fd with offset=0 and full YUV420 length
            const FrameBuffer::Plane &first_plane = buffer->planes()[0];
            encoding_success = h264_encoder_->encodeDMA(first_plane.fd.get(), 0, total_yuv_size, encoded_data);
            if (encoding_success) {
                std::cout << "H.264 DMA frame encoded (" << encoded_data.size() << " bytes)" << std::endl;
            } else {
                std::cout << "H.264 DMA encoding failed (full YUV420), falling back to JPEG" << std::endl;
            }
        }

        // Fall back to JPEG encoding with memory copy
        if (!encoding_success) {
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

            // JPEG encoding as fallback
            if (!jpeg_encoder_->encode(data_vector.data(), data_vector.size(), encoded_data)) {
                std::cerr << "JPEG encoding error" << std::endl;
                return;
            }
        }

        tcp_client_->sendData(encoded_data);
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

    void enableH264Encoding(bool enable) {
        use_h264_ = enable && h264_encoder_;
        std::cout << "H.264 encoding " << (use_h264_ ? "enabled" : "disabled") << std::endl;
    }

    bool isH264Available() const {
        return h264_encoder_ != nullptr;
    }

    bool setH264GopSize(int gop_size) {
        if (!h264_encoder_) {
            std::cerr << "H.264 encoder not available" << std::endl;
            return false;
        }
        return h264_encoder_->setGopSize(gop_size);
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

    // Check H.264 encoding availability
    std::cout << "\n=== Checking H.264 Encoding ===" << std::endl;
    if (capturer.isH264Available()) {
        std::cout << "H.264 hardware encoder detected!" << std::endl;
        capturer.enableH264Encoding(true);

        // Configure GOP size (keyframe interval) - smaller value = more keyframes
        // 30 = 1 keyframe per second at 30fps, 15 = 2 keyframes per second
        capturer.setH264GopSize(30);  // More frequent keyframes for better streaming
    } else {
        std::cout << "H.264 hardware encoder not available, using JPEG" << std::endl;
    }

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
