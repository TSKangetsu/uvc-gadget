#ifndef UVC_GADGET_HPP
#define UVC_GADGET_HPP

#include <string>
#include <vector>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/usb/ch9.h>
#include "uvc.h"

namespace UvcGadget
{

// 视频帧信息
struct FrameInfo
{
    unsigned int width;
    unsigned int height;
    std::vector<unsigned int> intervals;
};

// 视频格式信息
struct FormatInfo
{
    unsigned int fcc; // FourCC
    std::vector<FrameInfo> frames;
};

// 视频数据缓冲区
struct Buffer
{
    void *start = nullptr;
    size_t length = 0;
    struct v4l2_buffer v4l2_buf;
};

class UvcDevice
{
public:
    UvcDevice(const std::string& dev_name, bool standalone_mode);
    ~UvcDevice();

    bool Open();
    void Close();
    void Run();

private:
    // 初始化
    bool Init();
    void InitEvents();

    // 事件处理
    void ProcessEvents();
    void HandleSetupEvent(const struct uvc_event& uvc_event);
    void HandleStreamOnEvent();
    void HandleStreamOffEvent();
    void HandleDataEvent(const struct uvc_request_data& data);

    // 视频流处理
    void ProcessVideo();
    void FillBuffer(Buffer& buffer);

    // 缓冲区管理
    bool RequestBuffers();
    bool QueueBuffers();
    void UninitDevice();

    // 控制请求处理
    void ProcessClassRequest(const struct usb_ctrlrequest& ctrl, struct uvc_request_data& resp);
    void ProcessControlRequest(uint8_t req, uint8_t cs, uint8_t entity_id, uint8_t len, struct uvc_request_data& resp);
    void ProcessStreamingRequest(uint8_t req, uint8_t cs, struct uvc_request_data& resp);
    void FillStreamingControl(struct uvc_streaming_control& ctrl, int iframe, int iformat);

private:
    // 设备属性
    std::string device_name_;
    int fd_ = -1;
    bool is_streaming_ = false;
    bool standalone_mode_;
    
    // 格式和分辨率
    unsigned int fcc_;
    unsigned int width_;
    unsigned int height_;
    
    // 缓冲区
    std::vector<Buffer> buffers_;
    unsigned int buffer_count_ = 4; // 默认缓冲区数量

    // UVC 控制
    struct uvc_streaming_control probe_control_;
    struct uvc_streaming_control commit_control_;
    int current_control_ = 0;
    
    // 颜色，用于生成 dummy 数据
    uint8_t color_ = 0;

    // MJPEG dummy data
    void* mjpeg_data_ = nullptr;
    size_t mjpeg_size_ = 0;

    // 支持的格式
    static const std::vector<FormatInfo> supported_formats_;
};

} // namespace UvcGadget

#endif // UVC_GADGET_HPP