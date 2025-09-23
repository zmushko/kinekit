#include <libcamera/libcamera.h>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <map>
#include <string>
#include <turbojpeg.h>
#include <vector>
#include <cmath>

using namespace libcamera;

class JpegEncoder {
private:
    tjhandle tj_;
    int quality_;
    size_t frame_size_;
    struct frame_raw_ {
        unsigned char* data;
        size_t size;
        int width;
        int height;
        int stride;
        int framerate;
    } frame_raw_;   
    unsigned char* jpeg_data;
    unsigned long jpeg_size;
    unsigned long jpeg_worst_size;
public:
    JpegEncoder(int width, int height, int framerate, int quality = 85, int stride = 0)
        : tj_(nullptr), quality_(quality), frame_size_(width * height * 3 / 2),
          jpeg_data(nullptr), jpeg_size(0), jpeg_worst_size(0) {
        frame_raw_.width = width;
        frame_raw_.height = height;
        frame_raw_.stride = (stride > 0) ? stride : width;  // Use stride if provided, otherwise use width
        frame_raw_.framerate = framerate;
        frame_raw_.size = frame_size_;
        frame_raw_.data = (unsigned char*)malloc(frame_size_);
        if (!frame_raw_.data) {
            throw std::runtime_error("Failed to allocate frame buffer");
        }

        jpeg_worst_size = tjBufSize(width, height, TJSAMP_420);
        if (jpeg_worst_size == 0) {
            free(frame_raw_.data);
            throw std::runtime_error("Error calculating worst case JPEG size");
        }

        jpeg_data = (unsigned char*)tjAlloc(jpeg_worst_size);
        if (!jpeg_data) {
            free(frame_raw_.data);
            throw std::runtime_error("Failed to allocate JPEG buffer");
        }

        tj_ = tjInitCompress();
        if (!tj_) {
            free(frame_raw_.data);
            tjFree(jpeg_data);
            throw std::runtime_error("Error initializing TurboJPEG");
        }
    }

    ~JpegEncoder() {
        if (tj_) {
            tjDestroy(tj_);
        }
        if (frame_raw_.data) {
            free(frame_raw_.data);
        }
        if (jpeg_data) {
            tjFree(jpeg_data);
        }
    }

    bool encode(const unsigned char* yuv_data, size_t yuv_size, std::vector<unsigned char>& out_jpeg) {
        // For YUV420 with stride, minimum size should be stride * height * 3/2
        size_t expected_size = frame_raw_.stride * frame_raw_.height * 3 / 2;
        if (yuv_size < expected_size) {
            std::cerr << "Input YUV data is too small for " << frame_raw_.width << "x" << frame_raw_.height 
                      << " YUV420 with stride " << frame_raw_.stride << std::endl;
            std::cerr << "Expected minimum: " << expected_size << " bytes, got: " << yuv_size << " bytes" << std::endl;
            return false;
        }

        jpeg_size = jpeg_worst_size;
        
        std::cout << "🔧 TurboJPEG параметры: width=" << frame_raw_.width 
                  << ", height=" << frame_raw_.height << ", stride=" << frame_raw_.stride 
                  << ", buffer_size=" << yuv_size << std::endl;
        
        // Используем tjCompressFromYUVPlanes для более точного контроля над плоскостями
        // Для YUV420: Y плоскость = stride * height, U и V плоскости = stride/2 * height/2 каждая
        const unsigned char* planes[3];
        int strides[3];
        
        planes[0] = yuv_data;  // Y плоскость
        planes[1] = yuv_data + frame_raw_.stride * frame_raw_.height;  // U плоскость
        planes[2] = planes[1] + (frame_raw_.stride / 2) * (frame_raw_.height / 2);  // V плоскость
        
        strides[0] = frame_raw_.stride;      // Y stride
        strides[1] = frame_raw_.stride / 2;  // U stride
        strides[2] = frame_raw_.stride / 2;  // V stride
        
        std::cout << "🔧 Плоскости YUV420: Y@" << (void*)planes[0] 
                  << ", U@" << (void*)planes[1] << ", V@" << (void*)planes[2] << std::endl;
                  
        int ret = tjCompressFromYUVPlanes(tj_, planes, frame_raw_.width, strides,
                                          frame_raw_.height, TJSAMP_420,
                                          &jpeg_data, &jpeg_size, quality_, 
                                          TJFLAG_FASTDCT | TJFLAG_NOREALLOC);
        if (ret != 0) {
            std::cerr << "TurboJPEG compression error: " << tjGetErrorStr() << std::endl;
            std::cerr << "Parameters: w=" << frame_raw_.width << ", h=" << frame_raw_.height 
                      << ", strides=[" << strides[0] << "," << strides[1] << "," << strides[2] << "]" << std::endl;
            return false;
        }

        out_jpeg.assign(jpeg_data, jpeg_data + jpeg_size);
        return true;
    }
};

// Class for connect by TCP to ffplay or other consumer of JPEG frames and send frames
class JpegTcpSender {
public:
    JpegTcpSender(const std::string& host, int port);
    ~JpegTcpSender();

    bool connect();
    void disconnect();
    bool sendFrame(const std::vector<unsigned char>& jpeg_data);

private:
    std::string host_;
    int port_;
    int sockfd_;
};

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sys/mman.h>

JpegTcpSender::JpegTcpSender(const std::string& host, int port)
    : host_(host), port_(port), sockfd_(-1) {}

JpegTcpSender::~JpegTcpSender() {
    disconnect();
}

bool JpegTcpSender::connect() {
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
        return false;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port_);

    if (inet_pton(AF_INET, host_.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported: " << host_ << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    if (::connect(sockfd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection to " << host_ << ":" << port_ << " failed: " << strerror(errno) << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    return true;
}
void JpegTcpSender::disconnect() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
}
bool JpegTcpSender::sendFrame(const std::vector<unsigned char>& jpeg_data) {
    if (sockfd_ < 0) {
        std::cerr << "Socket is not connected" << std::endl;
        return false;
    }

    ssize_t total_sent = 0;
    ssize_t to_send = jpeg_data.size();
    const unsigned char* data_ptr = jpeg_data.data();

    while (total_sent < to_send) {
        ssize_t sent = send(sockfd_, data_ptr + total_sent, to_send - total_sent, 0);
        if (sent < 0) {
            std::cerr << "Send error: " << strerror(errno) << std::endl;
            return false;
        }
        total_sent += sent;
    }

    return true;
}

class Capturer {
private:
    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::unique_ptr<JpegEncoder> jpeg_encoder_;
    std::unique_ptr<JpegTcpSender> jpeg_sender_;
    std::vector<std::unique_ptr<Request>> requests_;

    // Текущие настройки камеры
    struct CameraSettings {
        int exposure_time = 0;      // 0 = auto
        int analog_gain = 0;        // 0 = auto (ISO)
        int digital_gain = 100;     // 1.0x
        int af_mode = 0;            // 0 = auto, 1 = manual, 2 = continuous
        int af_position = 0;        // Позиция фокуса для ручного режима
        float brightness = 0.0f;    // -1.0 to 1.0
        float contrast = 1.0f;      // 0.0 to 2.0
        float saturation = 1.0f;    // 0.0 to 2.0
        bool aec_enable = true;     // Auto Exposure Control
        bool awb_enable = true;     // Auto White Balance
    } settings_;
    
    int frames_captured_ = 0;
    bool capture_running_ = false;
    bool encoder_adjusted_ = false;  // Flag to track if encoder was adjusted for real buffer size
    
    // Actual frame dimensions from camera configuration
    int actual_width_ = 0;
    int actual_height_ = 0;
    int actual_stride_ = 0;
    
public:
    Capturer() : cm_(std::make_unique<CameraManager>()), 
        jpeg_sender_(std::make_unique<JpegTcpSender>("127.0.0.1", 8080)) {}

    Capturer(const std::string& host, int port) : cm_(std::make_unique<CameraManager>()), 
        jpeg_sender_(std::make_unique<JpegTcpSender>(host, port)) {}

    bool connectToServer() {
        std::cout << "🌐 Подключение к TCP серверу..." << std::endl;
        if (!jpeg_sender_->connect()) {
            std::cerr << "❌ Не удалось подключиться к TCP серверу" << std::endl;
            return false;
        }
        std::cout << "✅ Успешно подключены к TCP серверу" << std::endl;
        return true;
    }

    void disconnectFromServer() {
        if (jpeg_sender_) {
            jpeg_sender_->disconnect();
        }
    }

    bool initialize() {
        std::cout << "🔧 Инициализация камеры..." << std::endl;
        
        int ret = cm_->start();
        if (ret) {
            std::cerr << "❌ Не удалось запустить менеджер камер: " << ret << std::endl;
            return false;
        }
        
        if (cm_->cameras().empty()) {
            std::cerr << "❌ Камеры не найдены!" << std::endl;
            return false;
        }
        
        camera_ = cm_->cameras()[0];
        std::cout << "📷 Найдена камера: " << camera_->id() << std::endl;
        
        if (camera_->acquire()) {
            std::cerr << "❌ Не удалось получить доступ к камере" << std::endl;
            return false;
        }
        
        // Показ доступных контролов
        showAvailableControls();
        
        std::cout << "✅ Камера инициализирована успешно" << std::endl;
        return true;
    }
    
    void showAvailableControls() {
        std::cout << "\n🎛️  Доступные контролы камеры:" << std::endl;
        
        const ControlInfoMap &controls = camera_->controls();
        for (const auto &ctrl : controls) {
            const ControlId *id = ctrl.first;
            
            std::cout << "   📋 " << id->name() << " (ID: " << id->id() << ")" << std::endl;
        }
        
        std::cout << "\n🔧 Основные контролы для управления:" << std::endl;
        std::cout << "   📸 ExposureTime - экспозиция в микросекундах" << std::endl;
        std::cout << "   🔆 AnalogueGain - аналоговое усиление (ISO)" << std::endl;
        std::cout << "   📊 DigitalGain - цифровое усиление" << std::endl;
        std::cout << "   🔍 AfMode - режим автофокуса" << std::endl;
        std::cout << "   🎯 LensPosition - позиция фокуса" << std::endl;
        std::cout << "   💡 Brightness - яркость (-1.0 до 1.0)" << std::endl;
        std::cout << "   🌈 Contrast - контраст (0.0 до 2.0)" << std::endl;
        std::cout << "   🎨 Saturation - насыщенность (0.0 до 2.0)" << std::endl;
        std::cout << std::endl;
    }
    
    bool configure() {
        std::cout << "⚙️  Конфигурирование камеры..." << std::endl;
        
        config_ = camera_->generateConfiguration({StreamRole::StillCapture});
        if (!config_) {
            std::cerr << "❌ Не удалось создать конфигурацию" << std::endl;
            return false;
        }
        
        StreamConfiguration &stream_config = config_->at(0);
        stream_config.size = Size(1920, 1080);  // Request high resolution, let camera adjust
        stream_config.pixelFormat = formats::YUV420;
        
        std::cout << "📐 Конфигурация потока:" << std::endl;
        std::cout << "   Запрошенный размер: " << stream_config.size.toString() << std::endl;
        std::cout << "   Формат: " << stream_config.pixelFormat.toString() << std::endl;
        
        CameraConfiguration::Status validation = config_->validate();
        if (validation == CameraConfiguration::Invalid) {
            std::cerr << "❌ Конфигурация недействительна" << std::endl;
            return false;
        } else if (validation == CameraConfiguration::Adjusted) {
            std::cout << "⚠️  Конфигурация была скорректирована" << std::endl;
            // Показать фактические размеры после корректировки
            StreamConfiguration &adjusted_config = config_->at(0);
            std::cout << "   Скорректированный размер: " << adjusted_config.size.toString() << std::endl;
        }
        
        std::cout << "📐 Финальная конфигурация перед применением:" << std::endl;
        std::cout << "   Размер: " << config_->at(0).size.toString() << std::endl;
        
        if (camera_->configure(config_.get())) {
            std::cerr << "❌ Не удалось применить конфигурацию" << std::endl;
            return false;
        }
        
        // Initialize JPEG encoder with actual camera dimensions
        StreamConfiguration &final_config = config_->at(0);
        actual_width_ = final_config.size.width;
        actual_height_ = final_config.size.height;
        actual_stride_ = final_config.stride;
        
        std::cout << "🎥 Инициализация JPEG encoder с размером: " << actual_width_ << "x" << actual_height_ << std::endl;
        std::cout << "📏 Stride (шаг строки): " << actual_stride_ << " байт" << std::endl;
        std::cout << "🔢 Ожидаемый размер буфера YUV420: " << (actual_stride_ * actual_height_ * 3 / 2) << " байт" << std::endl;
        
        try {
            jpeg_encoder_ = std::make_unique<JpegEncoder>(
                actual_width_, 
                actual_height_, 
                30, 85, actual_stride_);
            std::cout << "✅ JPEG encoder успешно инициализирован" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "❌ Ошибка инициализации JPEG encoder: " << e.what() << std::endl;
            return false;
        }
        
        std::cout << "✅ Конфигурация применена успешно" << std::endl;
        return true;
    }
    
    void showCurrentSettings() {
        std::cout << "\n🎯 Текущие настройки камеры:" << std::endl;
        std::cout << "   📸 Экспозиция: " << (settings_.exposure_time == 0 ? "Авто" : std::to_string(settings_.exposure_time) + " мкс") << std::endl;
        std::cout << "   🔆 ISO (Analog Gain): " << (settings_.analog_gain == 0 ? "Авто" : std::to_string(settings_.analog_gain)) << std::endl;
        std::cout << "   📊 Digital Gain: " << (settings_.digital_gain / 100.0f) << "x" << std::endl;
        
        std::string af_mode_str;
        switch (settings_.af_mode) {
            case 0: af_mode_str = "Авто"; break;
            case 1: af_mode_str = "Ручной"; break;
            case 2: af_mode_str = "Непрерывный"; break;
            default: af_mode_str = "Неизвестно"; break;
        }
        std::cout << "   🔍 Автофокус: " << af_mode_str;
        if (settings_.af_mode == 1) {
            std::cout << " (позиция: " << settings_.af_position << ")";
        }
        std::cout << std::endl;
        
        std::cout << "   💡 Яркость: " << settings_.brightness << std::endl;
        std::cout << "   🌈 Контраст: " << settings_.contrast << std::endl;
        std::cout << "   🎨 Насыщенность: " << settings_.saturation << std::endl;
        std::cout << "   🔄 AEC: " << (settings_.aec_enable ? "Вкл" : "Выкл") << std::endl;
        std::cout << "   🔄 AWB: " << (settings_.awb_enable ? "Вкл" : "Выкл") << std::endl;
        std::cout << std::endl;
    }
    
    void setExposure(int exposure_us) {
        settings_.exposure_time = exposure_us;
        std::cout << "📸 Установка экспозиции: " << (exposure_us == 0 ? "Авто" : std::to_string(exposure_us) + " мкс") << std::endl;
    }
    
    void setISO(int iso_value) {
        // Преобразование ISO в analog gain (примерное)
        if (iso_value == 0) {
            settings_.analog_gain = 0; // Auto
        } else {
            settings_.analog_gain = iso_value / 100; // Примерное преобразование
        }
        std::cout << "🔆 Установка ISO: " << (iso_value == 0 ? "Авто" : std::to_string(iso_value)) << std::endl;
    }
    
    void setAutofocusMode(int mode) {
        settings_.af_mode = mode;
        std::string mode_str;
        switch (mode) {
            case 0: mode_str = "Авто"; break;
            case 1: mode_str = "Ручной"; break;
            case 2: mode_str = "Непрерывный"; break;
            default: mode_str = "Неизвестно"; break;
        }
        std::cout << "🔍 Установка режима автофокуса: " << mode_str << std::endl;
    }
    
    void setFocusPosition(int position) {
        settings_.af_position = position;
        std::cout << "🎯 Установка позиции фокуса: " << position << std::endl;
    }
    
    void setBrightness(float brightness) {
        settings_.brightness = std::max(-1.0f, std::min(1.0f, brightness));
        std::cout << "💡 Установка яркости: " << settings_.brightness << std::endl;
    }
    
    void setContrast(float contrast) {
        settings_.contrast = std::max(0.0f, std::min(2.0f, contrast));
        std::cout << "🌈 Установка контраста: " << settings_.contrast << std::endl;
    }
    
    void setSaturation(float saturation) {
        settings_.saturation = std::max(0.0f, std::min(2.0f, saturation));
        std::cout << "🎨 Установка насыщенности: " << settings_.saturation << std::endl;
    }
    
    void enableAutoExposure(bool enable) {
        settings_.aec_enable = enable;
        std::cout << "🔄 Автоэкспозиция: " << (enable ? "Включена" : "Выключена") << std::endl;
    }
    
    void enableAutoWhiteBalance(bool enable) {
        settings_.awb_enable = enable;
        std::cout << "🔄 Автобаланс белого: " << (enable ? "Включен" : "Выключен") << std::endl;
    }
    
    ControlList buildControlList() {
        ControlList controls;
        
        // Управление экспозицией
        if (settings_.aec_enable) {
            controls.set(controls::AeEnable, true);
        } else {
            controls.set(controls::AeEnable, false);
            if (settings_.exposure_time > 0) {
                controls.set(controls::ExposureTime, settings_.exposure_time);
            }
        }
        
        // Управление усилением (ISO)
        if (settings_.analog_gain > 0) {
            controls.set(controls::AnalogueGain, static_cast<float>(settings_.analog_gain));
        }
        
        if (settings_.digital_gain != 100) {
            controls.set(controls::DigitalGain, settings_.digital_gain / 100.0f);
        }
        
        // Управление автофокусом
        switch (settings_.af_mode) {
            case 0: // Auto
                controls.set(controls::AfMode, controls::AfModeAuto);
                controls.set(controls::AfTrigger, controls::AfTriggerStart);
                break;
            case 1: // Manual
                controls.set(controls::AfMode, controls::AfModeManual);
                if (settings_.af_position > 0) {
                    controls.set(controls::LensPosition, static_cast<float>(settings_.af_position));
                }
                break;
            case 2: // Continuous
                controls.set(controls::AfMode, controls::AfModeContinuous);
                break;
        }
        
        // Управление изображением
        if (settings_.brightness != 0.0f) {
            controls.set(controls::Brightness, settings_.brightness);
        }
        
        if (settings_.contrast != 1.0f) {
            controls.set(controls::Contrast, settings_.contrast);
        }
        
        if (settings_.saturation != 1.0f) {
            controls.set(controls::Saturation, settings_.saturation);
        }
        
        // Баланс белого
        controls.set(controls::AwbEnable, settings_.awb_enable);
        
        return controls;
    }
    
    bool setupBuffers() {
        std::cout << "🗂️  Настройка буферов..." << std::endl;
        
        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
        Stream *stream = config_->at(0).stream();
        
        int ret = allocator_->allocate(stream);
        if (ret < 0) {
            std::cerr << "❌ Не удалось выделить буферы" << std::endl;
            return false;
        }
        
        std::cout << "📦 Выделено " << ret << " буферов" << std::endl;
        
        for (unsigned int i = 0; i < allocator_->buffers(stream).size(); ++i) {
            auto request = camera_->createRequest();
            if (!request) {
                std::cerr << "❌ Не удалось создать запрос " << i << std::endl;
                return false;
            }
            
            const std::unique_ptr<FrameBuffer> &buffer = allocator_->buffers(stream)[i];
            int ret = request->addBuffer(stream, buffer.get());
            if (ret < 0) {
                std::cerr << "❌ Не удалось добавить буфер в запрос " << i << std::endl;
                return false;
            }
            
            // Применение настроек к каждому запросу
            ControlList controls = buildControlList();
            request->controls() = controls;
            
            requests_.push_back(std::move(request));
        }
        
        std::cout << "✅ Буферы настроены успешно" << std::endl;
        return true;
    }
    
    bool startCapture() {
        std::cout << "🚀 Запуск захвата..." << std::endl;
        
        camera_->requestCompleted.connect(this, &Capturer::requestComplete);
        
        if (camera_->start()) {
            std::cerr << "❌ Не удалось запустить камеру" << std::endl;
            return false;
        }
        
        capture_running_ = true;
        
        for (auto &request : requests_) {
            if (camera_->queueRequest(request.get())) {
                std::cerr << "❌ Не удалось поставить запрос в очередь" << std::endl;
                return false;
            }
        }
        
        std::cout << "✅ Захват запущен!" << std::endl;
        return true;
    }
    
    void captureWithSettings(int num_frames, int interval_ms = 2000) {
        std::cout << "\n📸 Захват " << num_frames << " кадров с применением настроек..." << std::endl;
        showCurrentSettings();

        for (int i = 0; (!num_frames || i < num_frames) && capture_running_; i++) {
            std::cout << "📷 Кадр " << (i + 1) << "/" << num_frames;
            
            // Показ текущих метаданных
            if (i > 0) {
                std::cout << " (применены настройки)";
            }
            std::cout << "..." << std::endl;
            if (interval_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        }
        
        std::cout << "✅ Захват завершен! Всего кадров: " << frames_captured_ << std::endl;
    }
    
    void stopCapture() {
        std::cout << "⏹️  Остановка захвата..." << std::endl;
        capture_running_ = false;
        
        if (camera_) {
            camera_->stop();
            // Отсоединяем обработчик событий
            camera_->requestCompleted.disconnect();
        }
        
        // Полная очистка ресурсов для повторного использования
        requests_.clear();
        allocator_.reset();
        config_.reset();
        
        std::cout << "✅ Захват остановлен и ресурсы освобождены" << std::endl;
    }
    
    void fullCleanup() {
        std::cout << "🧹 Полная очистка ресурсов..." << std::endl;
        
        capture_running_ = false;
        
        if (camera_) {
            if (camera_->start() == 0) { // Если камера была запущена
                camera_->stop();
            }
            camera_->requestCompleted.disconnect();
            camera_->release();
            camera_.reset();
        }
        
        requests_.clear();
        allocator_.reset();
        config_.reset();
        
        if (cm_) {
            cm_->stop();
        }
        
        std::cout << "✅ Полная очистка завершена" << std::endl;
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
    
    void saveFrameToFile(const FrameBuffer *buffer, const std::string &filename) {
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "❌ Не удалось создать файл: " << filename << std::endl;
            return;
        }
        
        for (const FrameBuffer::Plane &plane : buffer->planes()) {
            std::string info = "YUV420 Frame - size: " + std::to_string(plane.length) + " bytes\n";
            file.write(info.c_str(), info.length());
        }
        
        file.close();
        std::cout << "💾 Кадр сохранен: " << filename << std::endl;
    }
    
    void requestComplete(Request *req) {
        if (req->status() == Request::RequestComplete) {
            frames_captured_++;
            
            // // Показ метаданных из ответа
            // const ControlList &metadata = req->metadata();
            // std::cout << "📋 Кадр #" << frames_captured_ << " - ";
            
            // // Показ реальных значений экспозиции и усиления
            // if (metadata.contains(controls::ExposureTime.id())) {
            //     auto exposure_opt = metadata.get(controls::ExposureTime);
            //     if (exposure_opt.has_value()) {
            //         std::cout << "Exp:" << exposure_opt.value() << "μs ";
            //     }
            // }
            
            // if (metadata.contains(controls::AnalogueGain.id())) {
            //     auto gain_opt = metadata.get(controls::AnalogueGain);
            //     if (gain_opt.has_value()) {
            //         std::cout << "Gain:" << std::fixed << std::setprecision(1) << gain_opt.value() << "x ";
            //     }
            // }
            
            // if (metadata.contains(controls::LensPosition.id())) {
            //     auto focus_opt = metadata.get(controls::LensPosition);
            //     if (focus_opt.has_value()) {
            //         std::cout << "Focus:" << std::fixed << std::setprecision(1) << focus_opt.value() << " ";
            //     }
            // }
            
            // std::cout << std::endl;
            
            // Сохранение кадра
            const Request::BufferMap &buffers = req->buffers();
            for (auto bufferPair : buffers) {
                FrameBuffer *buffer = bufferPair.second;
                std::string filename = "frame_" + getCurrentTimestamp() + ".yuv";
                // saveFrameToFile(buffer, filename);
                // Кодирование в JPEG и отправка по TCP
                std::vector<unsigned char> jpeg_data;
                
                // Получаем указатель на данные через memory mapping
                const FrameBuffer::Plane &plane = buffer->planes()[0];
                std::cout << "🔍 Размер буфера: " << plane.length << " байт" << std::endl;
                
                // Если размер буфера не совпадает с ожидаемым, пересчитываем размеры и пересоздаем encoder
                size_t expected_buffer_size = actual_stride_ * actual_height_ * 3 / 2;
                if (plane.length != expected_buffer_size && !encoder_adjusted_) {
                    std::cout << "⚠️  Размер буфера (" << plane.length << ") не совпадает с ожидаемым (" << expected_buffer_size << ")" << std::endl;
                    std::cout << "🔄 Пересчитываем размеры из фактического буфера..." << std::endl;
                    
                    // Используем stride из ИСХОДНОЙ конфигурации камеры
                    // и вычисляем фактическую высоту из размера буфера
                    int config_stride = actual_stride_;  // Сохраненный stride из конфигурации
                    int new_stride = config_stride;
                    int new_width = actual_width_;       // Сохраненная ширина из конфигурации
                    
                    // Вычисляем фактическую высоту из размера буфера
                    int new_height = plane.length / (new_stride * 3 / 2);
                    
                    std::cout << "🎯 Используем stride из конфигурации: " << new_stride << std::endl;
                    std::cout << "🎯 Определена фактическая высота: " << new_height << " (для ширины " << new_width << ")" << std::endl;
                    
                    // Проверяем разумность результата
                    if (new_height < 480 || new_height > 1080) {
                        std::cout << "⚠️  Высота " << new_height << " выглядит неразумно, попробуем стандартные соотношения..." << std::endl;
                        
                        // Попробуем стандартные разрешения
                        if (plane.length == 2073600) {
                            new_width = 1920;
                            new_height = 720;
                            new_stride = 1920;
                        } else {
                            // Возврат к старой логике
                            new_width = 1280;
                            new_height = 1080;
                            new_stride = plane.length / (new_height * 3 / 2);
                        }
                    }
                    
                    std::cout << "🎯 Определены финальные размеры: " << new_width << "x" << new_height << " со stride: " << new_stride << std::endl;
                    
                    // Проверяем и корректируем stride для TurboJPEG (должен быть кратен 4)
                    if (new_stride % 4 != 0) {
                        new_stride = (new_stride + 3) & ~3;
                        std::cout << "⚙️  Скорректирован stride до " << new_stride << " (кратен 4)" << std::endl;
                    }
                    
                    std::cout << "🔄 Пересоздаем JPEG encoder с правильными размерами..." << std::endl;
                    
                    // Обновляем сохраненные размеры
                    actual_width_ = new_width;
                    actual_height_ = new_height;
                    actual_stride_ = new_stride;  // Используем вычисленный stride
                    
                    // Пересоздаем encoder с правильными размерами
                    try {
                        jpeg_encoder_ = std::make_unique<JpegEncoder>(
                            actual_width_, 
                            actual_height_, 
                            30, 85, actual_stride_);
                        encoder_adjusted_ = true;
                        std::cout << "✅ JPEG encoder пересоздан с размерами: " << actual_width_ << "x" << actual_height_ << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "❌ Ошибка пересоздания JPEG encoder: " << e.what() << std::endl;
                        continue;
                    }
                }
                
                std::cout << "📐 Используем размеры: " << actual_width_ << "x" << actual_height_ << " со stride: " << actual_stride_ << std::endl;
                
                void *mapped_data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), plane.offset);
                if (mapped_data == MAP_FAILED) {
                    std::cerr << "❌ Ошибка отображения памяти" << std::endl;
                    continue;
                }
                
                // Проверка инициализации JPEG encoder
                if (!jpeg_encoder_) {
                    std::cerr << "❌ JPEG encoder не инициализирован!" << std::endl;
                    munmap(mapped_data, plane.length);
                    continue;
                }
                
                if (jpeg_encoder_->encode(static_cast<unsigned char*>(mapped_data),
                                          plane.length, jpeg_data)) {
                    // Отправка по TCP с переподключением при ошибке
                    bool sent = false;
                    int retry_attempts = 3;
                    
                    for (int attempt = 0; attempt < retry_attempts && !sent; attempt++) {
                        if (!jpeg_sender_->sendFrame(jpeg_data)) {
                            std::cerr << "❌ Ошибка отправки JPEG по TCP (попытка " << (attempt + 1) << ")" << std::endl;
                            
                            // Пробуем переподключиться
                            if (attempt < retry_attempts - 1) {
                                std::cout << "🔄 Попытка переподключения..." << std::endl;
                                jpeg_sender_->disconnect();
                                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                                if (jpeg_sender_->connect()) {
                                    std::cout << "✅ Переподключение успешно" << std::endl;
                                } else {
                                    std::cerr << "❌ Переподключение неудачно" << std::endl;
                                }
                            }
                        } else {
                            std::cout << "📤 JPEG отправлен по TCP (" << jpeg_data.size() << " байт)" << std::endl;     
                            sent = true;
                        }
                    }
                    
                    if (!sent) {
                        std::cerr << "❌ Не удалось отправить JPEG после " << retry_attempts << " попыток" << std::endl;
                    }
                } else {
                    std::cerr << "❌ Ошибка кодирования JPEG" << std::endl;
                }
                
                // Освобождаем отображенную память
                munmap(mapped_data, plane.length);
                
                break;
            }
            
            // Повторная отправка запроса с обновленными настройками
            if (capture_running_) {
                req->reuse(Request::ReuseBuffers);
                ControlList controls = buildControlList();
                req->controls() = controls;
                camera_->queueRequest(req);
            }
        } else {
            std::cerr << "❌ Ошибка захвата кадра" << std::endl;
        }
    }
    
    ~Capturer() {
        std::cout << "🧹 Очистка ресурсов..." << std::endl;
        if (camera_) {
            if (capture_running_) {
                camera_->stop();
            }
            camera_->release();
        }
        if (cm_) {
            cm_->stop();
        }
        std::cout << "✅ Очистка завершена" << std::endl;
    }
};

int main(int argc, char** argv) {
    std::cout << "\n🚀 === Raspberry Pi Camera Capturer v3.0 ===" << std::endl;
    std::cout << "🎛️  С управлением параметрами камеры" << std::endl;
    std::cout << "📋 Camera Module v3 + Pi Zero 2W + libcamera" << std::endl;
    std::cout << "==============================================\n" << std::endl;
    
    // Проверка параметров командной строки
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <host> <port>" << std::endl;
        return -1;
    }
    std::string host = argv[1];
    int port = std::stoi(argv[2]);

    // Тест 1: Автоматические настройки
    // {
    //     std::cout << "\n1️⃣ Тест с автоматическими настройками:" << std::endl;
    //     Capturer capturer1;
        
    //     if (!capturer1.initialize() || !capturer1.configure()) {
    //         return -1;
    //     }
        
    //     capturer1.showCurrentSettings();
        
    //     if (!capturer1.setupBuffers() || !capturer1.startCapture()) {
    //         return -1;
    //     }
        
    //     capturer1.captureWithSettings(3, 2000);
    //     capturer1.stopCapture();
        
    //     std::cout << "✅ Тест 1 завершен\n" << std::endl;
    // } // capturer1 автоматически очищается здесь
    
    // Пауза между тестами
    // std::cout << "⏳ Пауза 3 секунды между тестами..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // // Тест 2: Ручные настройки
    // {
    //     std::cout << "\n2️⃣ Тест с ручными настройками:" << std::endl;
    //     Capturer capturer2;
        
    //     if (!capturer2.initialize() || !capturer2.configure()) {
    //         return -1;
    //     }
        
    //     capturer2.setExposure(10000);      // 10ms экспозиция
    //     capturer2.setISO(400);             // ISO 400
    //     capturer2.setAutofocusMode(1);     // Ручной фокус
    //     capturer2.setFocusPosition(5);     // Позиция фокуса
    //     capturer2.setBrightness(0.2f);     // +20% яркости
    //     capturer2.setContrast(1.2f);       // +20% контраста
    //     capturer2.enableAutoExposure(false); // Выключить автоэкспозицию
        
    //     if (!capturer2.setupBuffers() || !capturer2.startCapture()) {
    //         return -1;
    //     }
        
    //     capturer2.captureWithSettings(3, 2000);
    //     capturer2.stopCapture();
        
    //     std::cout << "✅ Тест 2 завершен\n" << std::endl;
    // } // capturer2 автоматически очищается здесь
    
    // // Пауза между тестами
    // std::cout << "⏳ Пауза 3 секунды между тестами..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Тест 3: Непрерывный автофокус
    {
        std::cout << "\n3️⃣ Тест с непрерывным автофокусом:" << std::endl;
        Capturer capturer3(host, port);
        
        if (!capturer3.connectToServer()) {
            return -1;
        }
        
        if (!capturer3.initialize() || !capturer3.configure()) {
            return -1;
        }
        
        capturer3.setAutofocusMode(2);     // Непрерывный автофокус
        capturer3.enableAutoExposure(true); // Включить автоэкспозицию
        
        if (!capturer3.setupBuffers() || !capturer3.startCapture()) {
            return -1;
        }

        capturer3.captureWithSettings(0, 5000);
        capturer3.stopCapture();
        capturer3.disconnectFromServer();
        
        std::cout << "✅ Тест 3 завершен\n" << std::endl;
    } // capturer3 автоматически очищается здесь

    std::cout << "\n🎉 Все демонстрации завершены успешно!" << std::endl;
    std::cout << "💡 Теперь вы можете конвертировать YUV файлы в JPEG!" << std::endl;
    std::cout << "\n📊 Итого создано YUV файлов для конвертации." << std::endl;
    
    return 0;
}