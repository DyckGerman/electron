// Copyright (c) 2014 GitHub, Inc.
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

// TODO: Why is this a thing? use double directly
struct Timestamp {
  double milliseconds;
};

// Controller object interface for non-GC frames
// (When accessing the API from C++)
// TODO: Rename to VideoFrameControl ???
struct VideoFrameCallback {
  using Format = VideoFrame::Format;

  // Allocates a frame of the specific format
  virtual VideoFrame* allocateFrame(Timestamp ts,
                                    const Format* format = nullptr) = 0;

  // Enqueues the frame
  virtual void queueFrame(Timestamp ts, VideoFrame* frame) = 0;

  // Releases the frame
  virtual void releaseFrame(VideoFrame* frame) = 0;
};

// Retrieves VideoFrameCallback pointer from an internal field of an object
// TODO: Why inline? move implementation to .cc
inline VideoFrameCallback* unwrapCallback(v8::Local<v8::Value> value) {
  if (!value->IsObject())
    return nullptr;
  v8::Local<v8::Object> obj = v8::Local<v8::Object>::Cast(value);
  if (obj->InternalFieldCount() != 2)
    return nullptr;
  return static_cast<VideoFrameCallback*>(
      obj->GetAlignedPointerFromInternalField(1));
}

struct VideoFrameCallbackHolder;

namespace detail {

using VideoFrameCallbackHolderPtr = std::shared_ptr<VideoFrameCallbackHolder>;

// Custom deleter for VideoFrame
// TODO: Refactor to a class, groom, rename to VideoFrameDeleter
// TODO: Rename callback_ to holder_
struct VideoFrameReleaser {
  void operator()(VideoFrame* frame) const;
  // private:
  friend VideoFrameCallbackHolder;
  explicit VideoFrameReleaser(VideoFrameCallbackHolderPtr);
  VideoFrameReleaser(VideoFrameReleaser const&);
  ~VideoFrameReleaser();

  // Controller holder
  VideoFrameCallbackHolderPtr callback_;
};

// TODO: Why inline? move implementation to .cc
// TODO: Before moving to .cc - check that it will compile in the C++ project
inline VideoFrameReleaser::VideoFrameReleaser(VideoFrameCallbackHolderPtr cb)
    : callback_(cb) {}
inline VideoFrameReleaser::VideoFrameReleaser(VideoFrameReleaser const&) =
    default;
inline VideoFrameReleaser::~VideoFrameReleaser() = default;

}  // namespace detail

// Helper object that wraps VideoFrameCallback functionality
// Simplifies allocation and deletion of non-GC frames
// in a safe manner.
// Holds a strong reference to the VideoFrameCallback
// TODO: Refactor to a class, groom, make non-copyable
// TODO: Rename to ControllerHolder
struct VideoFrameCallbackHolder
    : std::enable_shared_from_this<VideoFrameCallbackHolder> {
  using Format = VideoFrame::Format;
  using FramePtr = std::unique_ptr<VideoFrame, detail::VideoFrameReleaser>;

  // Allocates a frame with a specific format
  FramePtr allocate(Timestamp ts, const Format* format) {
    return {callback_->allocateFrame(ts, format),
            detail::VideoFrameReleaser(shared_from_this())};
  }
  // TODO: Move implementations to .cc
  FramePtr allocate(Timestamp ts, const Format& f) { return allocate(ts, &f); }
  FramePtr allocate(Timestamp ts) { return allocate(ts, nullptr); }

  void queue(Timestamp ts, FramePtr ptr) {
    callback_->queueFrame(ts, ptr.release());
  }

  // Creates the holder based on VideoFrameCallback object
  static std::shared_ptr<VideoFrameCallbackHolder> unwrap(
      v8::Isolate* isolate,
      v8::Local<v8::Value> value) {
    auto* cb = unwrapCallback(value);
    if (cb) {
      return std::make_shared<VideoFrameCallbackHolder>(
          isolate, v8::Local<v8::Object>::Cast(value), cb);
    }
    return nullptr;
  }

  VideoFrameCallbackHolder(v8::Isolate* isolate,
                           v8::Local<v8::Object> wrapper,
                           VideoFrameCallback* cb);
  ~VideoFrameCallbackHolder();

 private:
  // TODO: Remove friend, make accessors
  friend detail::VideoFrameReleaser;

  // Controller wrapper
  v8::Global<v8::Object> wrapper_;

  // Controller
  VideoFrameCallback* callback_;
};

// TODO: Why are these inline? groom, move to .cc if possible
inline VideoFrameCallbackHolder::VideoFrameCallbackHolder(
    v8::Isolate* isolate,
    v8::Local<v8::Object> wrapper,
    VideoFrameCallback* cb)
    : wrapper_(isolate, wrapper), callback_(cb) {}

inline VideoFrameCallbackHolder::~VideoFrameCallbackHolder() = default;

namespace detail {
// TODO: WHy inline? move to .cc
inline void VideoFrameReleaser::operator()(VideoFrame* frame) const {
  callback_->callback_->releaseFrame(frame);
}
}  // namespace detail

}  // namespace CustomMediaStream

#endif  // SHELL_COMMON_API_CUSTOM_MEDIA_STREAM_H_
