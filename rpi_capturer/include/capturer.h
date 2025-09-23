#ifndef CAPTURER_H
#define CAPTURER_H

#include <libcamera/libcamera.h>
#include <memory>
#include <iostream>

class Capturer {
private:
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::Request> request_;
    
public:
    Capturer();
    ~Capturer();
    
    bool initialize();
    bool configure();
    bool start();
    void stop();
    void capture();
    
private:
    void listCameras();
    void onRequestComplete(libcamera::Request *req);
};

#endif // CAPTURER_H
