// Copyright (c) 2019 GitHub, Inc.
// Use of this source code is governed by the MIT license that can be
// found in the LICENSE file.

#ifndef SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_
#define SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_

#include <v8.h>
#include <functional>
#include <memory>

#if defined(__APPLE__)
#include <CoreVideo/CoreVideo.h>
#endif

namespace CustomMediaStream {

// Frame interface for non-GC frames
// (When accessing the API from C++)
class VideoFrame {
 public:
  // YUV Plane names
  enum class Plane { Y, U, V, A };

  // Pixel format
  enum class PixelFormat {
    UNKNOWN,  // Unknown or unspecified format value.
    I420,     // 12bpp YUV planar 1x1 Y, 2x2 UV samples, a.k.a. YU12.
    ARGB,     // 32bpp BGRA (byte-order), 1 plane.
    ABGR      // 32bpp RGBA (byte-order), 1 plane.
  };

  // Frame format
  struct Format {
    PixelFormat pixel_format = PixelFormat::UNKNOWN;
    int width = 0;
    int height = 0;
  };

  virtual ~VideoFrame() = default;

  // Gets the frame format
  virtual Format format() const = 0;

  // Gets the stride of the plane
  virtual int stride(Plane plane) const = 0;

  // Gets rows count of the plane
  virtual int rows(Plane plane) const = 0;

  // Gets bytes count per row of the plane
  virtual int rowBytes(Plane plane) const = 0;

  // Gets data of the plane
  virtual void* data(Plane plane) const = 0;
};

// Controller object interface for non-GC frames
// Manages the life cycle of frames
// (When accessing the API from C++)
class VideoFramesController {
 public:
  using Format = VideoFrame::Format;

  static const int kImplFieldIdx = 0;
  static const int kInterfaceFieldIdx = 1;

  // Retrieves VideoFramesController pointer
  // from an internal field of the wrapper
  static VideoFramesController* unwrap(v8::Local<v8::Object> wrapper) {
    if (wrapper->InternalFieldCount() != 2)
      return nullptr;

    void* ptr = wrapper->GetAlignedPointerFromInternalField(kInterfaceFieldIdx);
    return static_cast<VideoFramesController*>(ptr);
  }

  virtual ~VideoFramesController() = default;

  // Allocates a frame of the specific format
  // timestamp is in milliseconds
  virtual VideoFrame* allocateFrame(double timestamp,
                                    const Format* format = nullptr) = 0;

  // Enqueues the frame
  // timestamp is in milliseconds
  virtual void queueFrame(double timestamp, VideoFrame* frame) = 0;

#if defined(__APPLE__)
  virtual void queueFrame(CVPixelBufferRef pixelBuffer, double timestamp) = 0;
#endif

  // Releases the frame
  virtual void releaseFrame(VideoFrame* frame) = 0;
};

// Helper object that wraps VideoFramesController functionality
// Simplifies allocation and deletion of non-GC frames
// in a safe manner.
// Holds a strong reference to the VideoFramesController wrapper
class ControllerHolder final
    : public std::enable_shared_from_this<ControllerHolder> {
 public:
  using Format = VideoFrame::Format;
  using FramePtr =
      std::unique_ptr<VideoFrame, std::function<void(VideoFrame*)>>;
  using HolderPtr = std::shared_ptr<ControllerHolder>;

  // Creates the holder based on VideoFramesController wrapper object
  static HolderPtr create(v8::Isolate* isolate, v8::Local<v8::Object> wrapper) {
    auto* controller = VideoFramesController::unwrap(wrapper);
    if (!controller)
      return nullptr;

    auto* holder = new ControllerHolder(isolate, wrapper, controller);
    return HolderPtr(holder);
  }

  ~ControllerHolder();

  // Allocates a frame of the specific format
  // timestamp is in milliseconds
  FramePtr allocate(double timestamp, const Format* format) {
    auto shared_this = shared_from_this();
    auto frame_deleter = [shared_this](VideoFrame* f) {
      shared_this->controller_->releaseFrame(f);
    };

    return FramePtr(controller_->allocateFrame(timestamp, format),
                    frame_deleter);
  }

  // Allocates a frame of the specific format
  // timestamp is in milliseconds
  FramePtr allocate(double timestamp, const Format& f) {
    return allocate(timestamp, &f);
  }

  // Allocates a frame of a format that was passed to createTrack from JS
  // timestamp is in milliseconds
  FramePtr allocate(double timestamp) { return allocate(timestamp, nullptr); }

  // Enqueues the frame
  // timestamp is in milliseconds
  void queue(double timestamp, FramePtr ptr) {
    controller_->queueFrame(timestamp, ptr.release());
  }

#if defined(__APPLE__)
  void queue(CVPixelBufferRef pixelBuffer, double timestamp) {
    controller_->queueFrame(pixelBuffer, timestamp);
  }
#endif

 private:
  ControllerHolder(v8::Isolate* isolate,
                   v8::Local<v8::Object> wrapper,
                   VideoFramesController* controller);

  // Controller wrapper
  v8::Global<v8::Object> wrapper_;

  // Controller
  VideoFramesController* controller_;
};

inline ControllerHolder::ControllerHolder(v8::Isolate* isolate,
                                          v8::Local<v8::Object> wrapper,
                                          VideoFramesController* controller)
    : wrapper_(isolate, wrapper), controller_(controller) {}

inline ControllerHolder::~ControllerHolder() {}

}  // namespace CustomMediaStream

#endif  // SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_
