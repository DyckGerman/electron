// Copyright (c) 2019 GitHub, Inc.
// Use of this source code is governed by the MIT license that can be
// found in the LICENSE file.

#ifndef SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_
#define SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_

#include <v8.h>
#include <memory>

namespace CustomMediaStream {

// Frame interface for non-GC frames
// (When accessing the API from C++)
class VideoFrame {
 public:
  // YUV Plane names
  enum class Plane { Y, U, V };

  // Frame format
  struct Format {
    int width = 0;
    int height = 0;
  };

  // Gets the frame format
  virtual Format format() = 0;

  // Gets the stride of the plane
  virtual int stride(Plane plane) = 0;

  // Gets rows count of the plane
  virtual int rows(Plane plane) = 0;

  // Gets data of the plane
  virtual void* data(Plane plane) = 0;
};

// Controller object interface for non-GC frames
// Manages the life cycle of frames
// (When accessing the API from C++)
class VideoFramesController {
 public:
  using Format = VideoFrame::Format;

  // Retrieves VideoFramesController pointer from an internal field of an object
  static VideoFramesController* unwrap(v8::Local<v8::Value> value) {
    if (!value->IsObject())
      return nullptr;

    auto obj = v8::Local<v8::Object>::Cast(value);
    if (obj->InternalFieldCount() != 2)
      return nullptr;

    void* ptr = obj->GetAlignedPointerFromInternalField(1);
    return static_cast<VideoFramesController*>(ptr);
  }

  // Allocates a frame of the specific format
  // timestamp is in milliseconds
  virtual VideoFrame* allocateFrame(double timestamp,
                                    const Format* format = nullptr) = 0;

  // Enqueues the frame
  // timestamp is in milliseconds
  virtual void queueFrame(double timestamp, VideoFrame* frame) = 0;

  // Releases the frame
  virtual void releaseFrame(VideoFrame* frame) = 0;
};

// Helper object that wraps VideoFramesController functionality
// Simplifies allocation and deletion of non-GC frames
// in a safe manner.
// Holds a strong reference to the VideoFramesController
// TODO: Refactor to a class, groom, make non-copyable
// TODO: Rename to ControllerHolder
struct VideoFrameCallbackHolder
    : std::enable_shared_from_this<VideoFrameCallbackHolder> {
  using Format = VideoFrame::Format;
  using FramePtr =
      std::unique_ptr<VideoFrame, std::function<void(VideoFrame*)>>;

  // Allocates a frame with a specific format
  // timestamp is in milliseconds
  FramePtr allocate(double timestamp, const Format* format) {
    auto sharedThis = shared_from_this();
    auto frameDeleter = [sharedThis](VideoFrame* f) {
      sharedThis->callback_->releaseFrame(f);
    };

    return FramePtr(callback_->allocateFrame(timestamp, format), frameDeleter);
  }
  // TODO: Move implementations to .cc
  FramePtr allocate(double timestamp, const Format& f) {
    return allocate(timestamp, &f);
  }

  FramePtr allocate(double timestamp) { return allocate(timestamp, nullptr); }

  // Enqueues the frame
  // timestamp is in milliseconds
  void queue(double timestamp, FramePtr ptr) {
    callback_->queueFrame(timestamp, ptr.release());
  }

  // Creates the holder based on VideoFramesController object
  static std::shared_ptr<VideoFrameCallbackHolder> unwrap(
      v8::Isolate* isolate,
      v8::Local<v8::Value> value) {
    auto* cb = VideoFramesController::unwrap(value);
    if (cb) {
      return std::make_shared<VideoFrameCallbackHolder>(
          isolate, v8::Local<v8::Object>::Cast(value), cb);
    }
    return nullptr;
  }

  VideoFrameCallbackHolder(v8::Isolate* isolate,
                           v8::Local<v8::Object> wrapper,
                           VideoFramesController* cb);
  ~VideoFrameCallbackHolder();

 private:
  // Controller wrapper
  v8::Global<v8::Object> wrapper_;

  // Controller
  VideoFramesController* callback_;
};

// TODO: Why are these inline? groom, move to .cc if possible
inline VideoFrameCallbackHolder::VideoFrameCallbackHolder(
    v8::Isolate* isolate,
    v8::Local<v8::Object> wrapper,
    VideoFramesController* cb)
    : wrapper_(isolate, wrapper), callback_(cb) {}

inline VideoFrameCallbackHolder::~VideoFrameCallbackHolder() = default;

}  // namespace CustomMediaStream

#endif  // SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_
