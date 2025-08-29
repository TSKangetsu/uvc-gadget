#include "uvc_gadget.hpp"
#include "uvc.h"
#include "mjpeg_image.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/usb/ch9.h>

namespace UvcGadget {

// 仅支持 YUYV 格式
const std::vector<FormatInfo> UvcDevice::supported_formats_ = {
    {
        V4L2_PIX_FMT_YUYV,
        {
            {640, 360, {333333, 666666, 10000000, 50000000}},
            {1280, 720, {50000000}},
        }
    },
    {
        V4L2_PIX_FMT_MJPEG,
        {
            {640, 360, {333333, 666666, 10000000, 50000000}},
            {1280, 720, {50000000}},
        }
    }
};

UvcDevice::UvcDevice(const std::string& dev_name, bool standalone_mode)
    : device_name_(dev_name),
      standalone_mode_(standalone_mode),
      fd_(-1),
      is_streaming_(false),
      fcc_(V4L2_PIX_FMT_YUYV),
      width_(640),
      height_(360),
      color_(0)
{
    memset(&probe_control_, 0, sizeof(probe_control_));
    memset(&commit_control_, 0, sizeof(commit_control_));

    // Use build-in MJPEG image
    mjpeg_data_ = mjpeg_image_data;
    mjpeg_size_ = mjpeg_image_data_len;
}

UvcDevice::~UvcDevice()
{
    Close();
}

bool UvcDevice::Open()
{
    fd_ = open(device_name_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Error: Cannot open UVC device " << device_name_ << std::endl;
        return false;
    }

    struct v4l2_capability cap;
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        std::cerr << "Error: VIDIOC_QUERYCAP failed" << std::endl;
        Close();
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)) {
        std::cerr << "Error: " << device_name_ << " is not a video output device" << std::endl;
        Close();
        return false;
    }
    
    std::cout << "UVC device opened: " << device_name_ << std::endl;
    return Init();
}

void UvcDevice::Close()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
        std::cout << "UVC device closed." << std::endl;
    }
}

bool UvcDevice::Init()
{
    // Set a default format on the device at init time.
    // This helps clients like v4l2-ctl that query the format before setting it.
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = fcc_;
    // For compressed formats, sizeimage MUST be provided.
    if (fcc_ == V4L2_PIX_FMT_MJPEG) {
        fmt.fmt.pix.sizeimage = mjpeg_size_;
    } else {
        fmt.fmt.pix.sizeimage = width_ * height_ * 2;
    }

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "Error: VIDIOC_S_FMT failed on init" << std::endl;
    }

    FillStreamingControl(probe_control_, 0, 0);
    FillStreamingControl(commit_control_, 0, 0);
    InitEvents();
    return true;
}

void UvcDevice::InitEvents()
{
    struct v4l2_event_subscription sub;
    memset(&sub, 0, sizeof(sub));
    
    sub.type = UVC_EVENT_SETUP;
    ioctl(fd_, VIDIOC_SUBSCRIBE_EVENT, &sub);
    
    sub.type = UVC_EVENT_DATA;
    ioctl(fd_, VIDIOC_SUBSCRIBE_EVENT, &sub);

    sub.type = UVC_EVENT_STREAMON;
    ioctl(fd_, VIDIOC_SUBSCRIBE_EVENT, &sub);

    sub.type = UVC_EVENT_STREAMOFF;
    ioctl(fd_, VIDIOC_SUBSCRIBE_EVENT, &sub);
}

void UvcDevice::Run()
{
    fd_set r_fds, w_fds, e_fds;

    while (true) {
        FD_ZERO(&r_fds);
        FD_ZERO(&w_fds);
        FD_ZERO(&e_fds);

        FD_SET(fd_, &e_fds);
        if (is_streaming_) {
            FD_SET(fd_, &w_fds); // Wait for the device to be ready for DQBUF
        } else {
            // When not streaming, we are only interested in control events
            FD_SET(fd_, &r_fds);
        }

        struct timeval tv = {2, 0}; // 2-second timeout

        int result = select(fd_ + 1, &r_fds, &w_fds, &e_fds, &tv);

        if (result < 0) {
            if (errno == EINTR) continue;
            std::cerr << "Error: select() failed: " << strerror(errno) << std::endl;
            break;
        }

        if (result == 0) {
            // std::cout << "select() timeout" << std::endl;
            continue;
        }
        
        if (FD_ISSET(fd_, &e_fds) || FD_ISSET(fd_, &r_fds)) {
            ProcessEvents();
        }

        if (is_streaming_ && FD_ISSET(fd_, &w_fds)) {
            ProcessVideo();
        }
    }
}

void UvcDevice::ProcessEvents()
{
    struct v4l2_event v4l2_event;
    int result = ioctl(fd_, VIDIOC_DQEVENT, &v4l2_event);

    if (result < 0) {
        // std::cerr << "Error: VIDIOC_DQEVENT failed: " << strerror(errno) << std::endl;
        return;
    }

    switch (v4l2_event.type) {
        case UVC_EVENT_SETUP:
            HandleSetupEvent(*reinterpret_cast<struct uvc_event*>(&v4l2_event.u.data));
            break;
        case UVC_EVENT_DATA:
            HandleDataEvent(reinterpret_cast<struct uvc_event*>(&v4l2_event.u.data)->data);
            break;
        case UVC_EVENT_STREAMON:
            HandleStreamOnEvent();
            break;
        case UVC_EVENT_STREAMOFF:
            HandleStreamOffEvent();
            break;
        default:
            break;
    }
}

void UvcDevice::HandleSetupEvent(const struct uvc_event& uvc_event)
{
    struct uvc_request_data resp;
    memset(&resp, 0, sizeof(resp));
    resp.length = -EL2HLT;

    ProcessClassRequest(uvc_event.req, resp);

    if (ioctl(fd_, UVCIOC_SEND_RESPONSE, &resp) < 0) {
        // std::cerr << "Error: UVCIOC_SEND_RESPONSE failed" << std::endl;
    }
}

void UvcDevice::ProcessClassRequest(const struct usb_ctrlrequest& ctrl, struct uvc_request_data& resp)
{
    if ((ctrl.bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS)
        return;

    switch (ctrl.wIndex & 0xff) {
        case UVC_INTF_CONTROL:
            ProcessControlRequest(ctrl.bRequest, ctrl.wValue >> 8, ctrl.wIndex >> 8, ctrl.wLength, resp);
            break;
        case UVC_INTF_STREAMING:
            ProcessStreamingRequest(ctrl.bRequest, ctrl.wValue >> 8, resp);
            break;
    }
}

void UvcDevice::HandleStreamOnEvent()
{
    std::cout << "Stream ON" << std::endl;

    if (!RequestBuffers()) {
        std::cerr << "Error: RequestBuffers failed in StreamOn" << std::endl;
        return;
    }

    if (!QueueBuffers()) {
        std::cerr << "Error: QueueBuffers failed in StreamOn" << std::endl;
        UninitDevice();
        return;
    }

    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        std::cerr << "Error: VIDIOC_STREAMON failed" << std::endl;
        UninitDevice();
        return;
    }
    is_streaming_ = true;
}

void UvcDevice::HandleStreamOffEvent()
{
    if (!is_streaming_) {
        return;
    }
    std::cout << "Stream OFF" << std::endl;
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    is_streaming_ = false;
    UninitDevice();
}

void UvcDevice::HandleDataEvent(const struct uvc_request_data& data)
{
    struct uvc_streaming_control* ctrl = (struct uvc_streaming_control*)data.data;

    if (current_control_ == UVC_VS_PROBE_CONTROL) {
        // The host is probing. We should check its proposed settings and
        // adjust them to our capabilities, then store them in probe_control_.
        probe_control_ = *ctrl;

        int format_idx = ctrl->bFormatIndex - 1;
        int frame_idx = ctrl->bFrameIndex - 1;
        
        if (format_idx < 0 || format_idx >= supported_formats_.size()) format_idx = 0;
        const auto& format = supported_formats_[format_idx];
        
        if (frame_idx < 0 || frame_idx >= format.frames.size()) frame_idx = 0;
        const auto& frame = format.frames[frame_idx];

        // Find the closest frame interval
        unsigned int best_interval = -1;
        for(auto interval : frame.intervals) {
            if (best_interval == -1 || abs((int)interval - (int)ctrl->dwFrameInterval) < abs((int)best_interval - (int)ctrl->dwFrameInterval)) {
                best_interval = interval;
            }
        }
        
        probe_control_.bFormatIndex = format_idx + 1;
        probe_control_.bFrameIndex = frame_idx + 1;
        probe_control_.dwFrameInterval = best_interval;

    } else if (current_control_ == UVC_VS_COMMIT_CONTROL) {
        // The host is committing the settings. We apply them.
        commit_control_ = *ctrl;
        
        const auto& format_info = supported_formats_[commit_control_.bFormatIndex - 1];
        const auto& frame_info = format_info.frames[commit_control_.bFrameIndex - 1];
        
        fcc_ = format_info.fcc;
        width_ = frame_info.width;
        height_ = frame_info.height;

        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        fmt.fmt.pix.pixelformat = fcc_;
        // For compressed formats, sizeimage MUST be provided.
        if (fcc_ == V4L2_PIX_FMT_MJPEG) {
            fmt.fmt.pix.sizeimage = mjpeg_size_;
        } else {
            fmt.fmt.pix.sizeimage = width_ * height_ * 2;
        }
        
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            std::cerr << "Error: VIDIOC_S_FMT failed on commit" << std::endl;
        }
    }
}

void UvcDevice::ProcessVideo()
{
    if (!is_streaming_) return;

    struct v4l2_buffer v4l2_buf;
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    v4l2_buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &v4l2_buf) < 0) {
        // In non-blocking mode, EAGAIN is expected when no buffer is available.
        if (errno == EAGAIN) {
            return;
        }
        std::cerr << "Error: VIDIOC_DQBUF failed: " << strerror(errno) << std::endl;
        // Consider stopping the stream on other errors
        return;
    }
    
    FillBuffer(buffers_[v4l2_buf.index]);

    if (ioctl(fd_, VIDIOC_QBUF, &buffers_[v4l2_buf.index].v4l2_buf) < 0) {
        std::cerr << "Error: VIDIOC_QBUF failed in ProcessVideo: " << strerror(errno) << std::endl;
        is_streaming_ = false; // Signal to stop streaming
    }
}

void UvcDevice::FillBuffer(Buffer& buffer)
{
    if (fcc_ == V4L2_PIX_FMT_YUYV) {
        size_t bpl = width_ * 2;
        for (unsigned int i = 0; i < height_; ++i) {
            memset(static_cast<char*>(buffer.start) + i * bpl, color_++, bpl);
        }
        buffer.v4l2_buf.bytesused = bpl * height_;
    } else if (fcc_ == V4L2_PIX_FMT_MJPEG) {
        if (mjpeg_data_ && mjpeg_size_ > 0) {
            memcpy(buffer.start, mjpeg_data_, mjpeg_size_);
            buffer.v4l2_buf.bytesused = mjpeg_size_;
        }
    }
}

bool UvcDevice::RequestBuffers()
{
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = buffer_count_;

    if (ioctl(fd_, VIDIOC_REQBUFS, &reqbuf) < 0) {
        std::cerr << "Error: VIDIOC_REQBUFS failed" << std::endl;
        return false;
    }

    buffers_.resize(reqbuf.count);
    buffer_count_ = reqbuf.count;

    for (unsigned int i = 0; i < buffer_count_; ++i) {
        auto& buf = buffers_[i];
        memset(&buf.v4l2_buf, 0, sizeof(buf.v4l2_buf));
        buf.v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.v4l2_buf.memory = V4L2_MEMORY_MMAP;
        buf.v4l2_buf.index = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf.v4l2_buf) < 0) {
            std::cerr << "Error: VIDIOC_QUERYBUF failed for buffer " << i << std::endl;
            return false;
        }

        buf.length = buf.v4l2_buf.length;
        buf.start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.v4l2_buf.m.offset);

        if (buf.start == MAP_FAILED) {
            std::cerr << "Error: mmap failed for buffer " << i << std::endl;
            return false;
        }
    }
    return true;
}

bool UvcDevice::QueueBuffers()
{
    for (unsigned int i = 0; i < buffer_count_; ++i) {
        FillBuffer(buffers_[i]);
        if (ioctl(fd_, VIDIOC_QBUF, &buffers_[i].v4l2_buf) < 0) {
            std::cerr << "Error: VIDIOC_QBUF failed for buffer " << i << std::endl;
            return false;
        }
    }
    return true;
}


void UvcDevice::UninitDevice()
{
    for (auto& buf : buffers_) {
        if (buf.start) {
            munmap(buf.start, buf.length);
        }
    }
    buffers_.clear();
    
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = 0;
    ioctl(fd_, VIDIOC_REQBUFS, &reqbuf);
}

void UvcDevice::ProcessControlRequest(uint8_t req, uint8_t cs, uint8_t entity_id, uint8_t len, struct uvc_request_data& resp)
{
    std::cout << "control request (entity " << (int)entity_id << " cs " << (int)cs << " req " << (int)req << ")" << std::endl;

    /*
     * This is a minimalist implementation. A real UVC device would need
     * to handle many more controls.
     */
    switch (entity_id) {
        case 0: // VideoControl Interface
            break;

        case 1: // Input Terminal
            switch (cs) {
                case UVC_CT_AE_MODE_CONTROL:
                    if (req == UVC_GET_CUR || req == UVC_GET_DEF || req == UVC_GET_RES) {
                        resp.data[0] = 2; // Auto exposure
                        resp.length = 1;
                    }
                    break;
            }
            break;

        case 2: // Processing Unit
            switch (cs) {
                case UVC_PU_BRIGHTNESS_CONTROL:
                    if (req == UVC_GET_MIN || req == UVC_GET_MAX || req == UVC_GET_CUR || req == UVC_GET_DEF || req == UVC_GET_RES) {
                        // Dummy values
                        resp.data[0] = 0;
                        resp.data[1] = 0;
                        resp.length = 2;
                    }
                    break;
            }
            break;
    }

    if (resp.length <= 0) {
        resp.length = -EL2HLT;
    }
}

void UvcDevice::ProcessStreamingRequest(uint8_t req, uint8_t cs, struct uvc_request_data& resp)
{
    std::cout << "streaming request (cs " << (int)cs << " req " << (int)req << ")" << std::endl;
    if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL) {
        resp.length = -EL2HLT;
        return;
    }

    struct uvc_streaming_control* ctrl = (struct uvc_streaming_control*)resp.data;
    resp.length = sizeof(*ctrl);

    switch (req) {
        case UVC_SET_CUR:
            current_control_ = cs;
            resp.length = 34; // UVC 1.1 probe/commit control size
            break;
        case UVC_GET_CUR:
            if (cs == UVC_VS_PROBE_CONTROL) {
                memcpy(ctrl, &probe_control_, sizeof(*ctrl));
            } else {
                memcpy(ctrl, &commit_control_, sizeof(*ctrl));
            }
            break;
        case UVC_GET_MIN:
        case UVC_GET_DEF:
            FillStreamingControl(*ctrl, 0, 0);
            break;
        case UVC_GET_MAX:
            FillStreamingControl(*ctrl, -1, -1);
            break;
        case UVC_GET_RES:
            memset(ctrl, 0, sizeof(*ctrl));
            break;
        case UVC_GET_LEN:
             resp.data[0] = 0x00;
             resp.data[1] = 0x22;
             resp.length = 2;
             break;
        case UVC_GET_INFO:
            resp.data[0] = 0x03; // Support GET and SET
            resp.length = 1;
            break;
        default:
            resp.length = -EL2HLT;
            break;
    }
}

void UvcDevice::FillStreamingControl(struct uvc_streaming_control& ctrl, int iframe, int iformat)
{
    if (iformat < 0) iformat = supported_formats_.size() + iformat;
    if (iformat < 0 || iformat >= (int)supported_formats_.size()) return;
    const auto& format = supported_formats_[iformat];

    int nframes = format.frames.size();
    if (iframe < 0) iframe = nframes + iframe;
    if (iframe < 0 || iframe >= nframes) return;
    const auto& frame = format.frames[iframe];

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.bmHint = 1;
    ctrl.bFormatIndex = iformat + 1;
    ctrl.bFrameIndex = iframe + 1;
    ctrl.dwFrameInterval = frame.intervals[0];
    if (format.fcc == V4L2_PIX_FMT_YUYV) {
        ctrl.dwMaxVideoFrameSize = frame.width * frame.height * 2;
    } else {
        ctrl.dwMaxVideoFrameSize = mjpeg_size_ > 0 ? mjpeg_size_ : frame.width * frame.height * 2;
    }
    // For Bulk endpoints, this should be the size of the video frame.
    // For Isochronous, it's more complex, but for this dummy driver,
    // setting it to the frame size is a safe bet.
    ctrl.dwMaxPayloadTransferSize = ctrl.dwMaxVideoFrameSize;
    ctrl.bmFramingInfo = 3;
    ctrl.bPreferedVersion = 1;
    ctrl.bMaxVersion = 1;
}

} // namespace UvcGadget
