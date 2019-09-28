// Copyright (c) 2019 GitHub, Inc.
// Use of this source code is governed by the MIT license that can be
// found in the LICENSE file.

#include "shell/common/api/custom_media_stream.h"

#define INSIDE_BLINK \
  1  // we need to be able to convert WebMediaStreamTrack to
     // MediaStreamComponent*
#include <third_party/blink/public/platform/web_media_stream_track.h>
#undef INSIDE_BLINK

// TODO: Check that all includes are required
#include <base/base64.h>
#include <base/rand_util.h>
#include <base/strings/utf_string_conversions.h>

#include <content/public/renderer/render_thread.h>
#include <media/base/video_frame.h>
#include <media/base/video_frame_pool.h>
#include <media/capture/video_capturer_source.h>
#include <third_party/blink/public/platform/web_media_stream_source.h>
#include <third_party/blink/public/web/modules/mediastream/media_stream_video_capturer_source.h>
#include <third_party/blink/public/web/modules/mediastream/media_stream_video_source.h>
#include <third_party/blink/public/web/modules/mediastream/media_stream_video_track.h>
#include <third_party/blink/renderer/bindings/core/v8/v8_binding_for_core.h>
#include <third_party/blink/renderer/modules/mediastream/media_stream_track.h>
#include <third_party/blink/renderer/platform/bindings/to_v8.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "electron/shell/common/native_mate_converters/callback.h"
#include "electron/shell/common/native_mate_converters/gfx_converter.h"
#include "electron/shell/common/node_includes.h"
#include "native_mate/arguments.h"
#include "native_mate/dictionary.h"
#include "native_mate/handle.h"
#include "native_mate/wrappable.h"

namespace {
struct ControlObject;
}

namespace mate {

// blink::WebMediaStreamTrack to v8::Value mate converter
template <>
struct Converter<blink::WebMediaStreamTrack> {
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate,
                                   blink::WebMediaStreamTrack track) {
    return blink::ToV8(
        blink::MediaStreamTrack::Create(
            blink::ToExecutionContext(isolate->GetCurrentContext()), track),
        isolate->GetCurrentContext()->Global(), isolate);
  }
};

// v8::ArrayBuffer to v8::Value mate converter
// TODO: Why is this explicit? check if everyting compiles without it
template <>
struct Converter<v8::Local<v8::ArrayBuffer>> {
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate,
                                   v8::Local<v8::ArrayBuffer> v) {
    return v;
  }
};

// base::TimeDelta to/from v8::Value mate converter
template <>
struct Converter<base::TimeDelta> {
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate, base::TimeDelta v) {
    return v8::Number::New(isolate, v.InMillisecondsF());
  }

  static bool FromV8(v8::Isolate* isolate,
                     v8::Local<v8::Value> val,
                     base::TimeDelta* out) {
    double d;
    if (Converter<double>::FromV8(isolate, val, &d)) {
      *out = base::TimeDelta::FromMillisecondsD(d);
      return true;
    }

    return false;
  }
};

// base::TimeTicks to/from v8::Value mate converter
template <>
struct Converter<base::TimeTicks> {
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate, base::TimeTicks v) {
    return v8::Number::New(
        isolate, (v - base::TimeTicks::UnixEpoch()).InMillisecondsF());
  }

  static bool FromV8(v8::Isolate* isolate,
                     v8::Local<v8::Value> val,
                     base::TimeTicks* out) {
    base::TimeDelta d;
    if (Converter<base::TimeDelta>::FromV8(isolate, val, &d)) {
      *out = base::TimeTicks::UnixEpoch() + d;
      return true;
    }
    return false;
  }
};

// ControlObject ptr to/from v8::Value mate converter
// Note that ToV8 creates a new Local that has internal fields
// while FromV8 just extracts ControlObject ptr from an
// internal field
template <>
struct Converter<ControlObject*> {
  // TODO: Why impl is not here?
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate, ControlObject* v);

  static bool FromV8(v8::Isolate* isolate,
                     v8::Local<v8::Value> value,
                     ControlObject** out) {
    if (!value->IsObject())
      return false;

    v8::Local<v8::Object> obj = v8::Local<v8::Object>::Cast(value);

    if (obj->InternalFieldCount() != 2)
      return false;

    *out =
        static_cast<ControlObject*>(obj->GetAlignedPointerFromInternalField(0));
    return true;
  }
};

}  // namespace mate

namespace {

// TODO: Remove
void test(mate::Arguments* args) {
  std::cout << "test" << std::endl;
}

// GC wrapper for a media::VideoFrame object
// (When accessing the API from JS)
// TODO: groom, turn into a class
struct Frame : mate::Wrappable<Frame> {
  Frame(v8::Isolate* isolate, scoped_refptr<media::VideoFrame> frame)
      : frame_(std::move(frame)) {
    Init(isolate);
  }

  std::uint32_t width() {
    if (!frame_)
      return 0;
    return frame_->visible_rect().width();
  }

  std::uint32_t height() {
    if (!frame_)
      return 0;
    return frame_->visible_rect().height();
  }

  std::uint32_t stride(int plane) {
    if (!frame_)
      return 0;

    switch (plane) {
      case media::VideoFrame::kYPlane:
      case media::VideoFrame::kUPlane:
      case media::VideoFrame::kVPlane:
        return frame_->stride(plane);
      default:
        return 0;
    }
  }

  base::TimeDelta timestamp() {
    if (!frame_)
      return {};

    return frame_->timestamp();
  }

  // Creates an ArrayBuffer over an existing memory
  // of the frame_, memory is freed only when frame_ is deleted
  v8::Local<v8::ArrayBuffer> data(int plane) {
    if (!frame_)
      return {};

    switch (plane) {
      case media::VideoFrame::kYPlane:
      case media::VideoFrame::kUPlane:
      case media::VideoFrame::kVPlane:
        return v8::ArrayBuffer::New(
            isolate(), frame_->visible_data(plane),
            frame_->stride(plane) * frame_->rows(plane));
      default:
        return {};
    }
  }

  v8::Local<v8::ArrayBuffer> y() { return data(media::VideoFrame::kYPlane); }

  v8::Local<v8::ArrayBuffer> u() { return data(media::VideoFrame::kUPlane); }

  v8::Local<v8::ArrayBuffer> v() { return data(media::VideoFrame::kVPlane); }

  static void BuildPrototype(v8::Isolate* isolate,
                             v8::Local<v8::FunctionTemplate> prototype) {
    prototype->SetClassName(
        mate::StringToV8(isolate, "CustomMediaStreamVideoFrame"));
    mate::ObjectTemplateBuilder(isolate, prototype->PrototypeTemplate())
        .SetMethod("data", &Frame::data)
        .SetMethod("stride", &Frame::stride)
        .SetProperty("width", &Frame::width)
        .SetProperty("height", &Frame::height)
        .SetProperty("timestamp", &Frame::timestamp)
        .SetProperty("y", &Frame::y)
        .SetProperty("u", &Frame::u)
        .SetProperty("v", &Frame::v);
  }

  scoped_refptr<media::VideoFrame> frame_;
};

// Non-GC wrapper for a media::VideoFrame object
// (When accessing the API from C++)
// TODO: Create helpers that convert plane enums
// TODO: Add explicit virtual keyword
struct NonGCFrame final : CustomMediaStream::VideoFrame {
  explicit NonGCFrame(scoped_refptr<media::VideoFrame> frame) : frame_(frame) {}

  Format format() override {
    return {frame_->visible_rect().width(), frame_->visible_rect().height()};
  }

  int stride(Plane plane) override {
    switch (plane) {
      case Plane::Y:
        return frame_->stride(media::VideoFrame::kYPlane);
      case Plane::U:
        return frame_->stride(media::VideoFrame::kUPlane);
      case Plane::V:
        return frame_->stride(media::VideoFrame::kVPlane);
      default:
        return 0;
    }
  }

  int rows(Plane plane) override {
    switch (plane) {
      case Plane::Y:
        return frame_->rows(media::VideoFrame::kYPlane);
      case Plane::U:
        return frame_->rows(media::VideoFrame::kUPlane);
      case Plane::V:
        return frame_->rows(media::VideoFrame::kVPlane);
      default:
        return 0;
    }
  }

  void* data(Plane plane) override {
    switch (plane) {
      case Plane::Y:
        return frame_->visible_data(media::VideoFrame::kYPlane);
      case Plane::U:
        return frame_->visible_data(media::VideoFrame::kUPlane);
      case Plane::V:
        return frame_->visible_data(media::VideoFrame::kVPlane);
      default:
        return nullptr;
    }
  }

  scoped_refptr<media::VideoFrame> frame_;
};

// Controller wrapper object, holds media::VideoFramePool
// and allows to allocate, enqueue and deallocate frames
// NOTE: Not deriving from mate::wrappable because
// we need 2 internal fields
// TODO: improve naming, may be name all wrappers as ***Wrapper?
// TODO: turn into a class, groom
// TODO: incapsulate private members, add accessors
struct ControlObject final : CustomMediaStream::VideoFramesController {
  // Ctor sets only the weak reference to the wrapper
  // so you need to call wrapper() function after ctor
  // otherwise it will be garbage collected
  ControlObject(v8::Isolate* isolate, gfx::Size resolution)
      : isolate_(isolate), resolution_(resolution) {
    auto templ = GetConstructor(isolate);

    v8::Local<v8::Object> wrapper;
    CHECK(templ->InstanceTemplate()
              ->NewInstance(isolate->GetCurrentContext())
              .ToLocal(&wrapper));

    wrapper_.Reset(isolate, wrapper);
    wrapper_.SetWeak(this, FirstWeakCallback, v8::WeakCallbackType::kParameter);

    auto w = this->wrapper();
    w->SetAlignedPointerInInternalField(0, this);
    w->SetAlignedPointerInInternalField(
        1, static_cast<CustomMediaStream::VideoFramesController*>(this));
  }

  v8::Local<v8::Object> wrapper() const {
    return v8::Local<v8::Object>::New(isolate_, wrapper_);
  }

  v8::Isolate* isolate() const { return isolate_; }

  ~ControlObject() {
    if (wrapper_.IsEmpty())
      return;

    wrapper()->SetAlignedPointerInInternalField(0, nullptr);
    wrapper()->SetAlignedPointerInInternalField(1, nullptr);
    wrapper_.ClearWeak();
    wrapper_.Reset();
  }

  // Creates or retrieves an existing function template of this object
  // and sets up instance and prototype templates
  static v8::Local<v8::FunctionTemplate> GetConstructor(v8::Isolate* isolate) {
    auto* data = gin::PerIsolateData::From(isolate);
    auto templ = data->GetFunctionTemplate(&kWrapperInfo);
    if (templ.IsEmpty()) {
      templ = v8::FunctionTemplate::New(isolate);
      templ->InstanceTemplate()->SetInternalFieldCount(2);
      BuildPrototype(isolate, templ);
      data->SetFunctionTemplate(&kWrapperInfo, templ);
    }
    return templ;
  }

  static void FirstWeakCallback(
      const v8::WeakCallbackInfo<ControlObject>& data) {
    ControlObject* p = data.GetParameter();
    p->wrapper_.Reset();
    data.SetSecondPassCallback(SecondWeakCallback);
  }

  static void SecondWeakCallback(
      const v8::WeakCallbackInfo<ControlObject>& data) {
    ControlObject* c = data.GetParameter();
    delete c;
  }

  static void BuildPrototype(v8::Isolate* isolate,
                             v8::Local<v8::FunctionTemplate> prototype) {
    prototype->SetClassName(
        mate::StringToV8(isolate, "CustomMediaStreamController"));
    mate::ObjectTemplateBuilder(isolate, prototype->PrototypeTemplate())
        .SetMethod("allocateFrame", &ControlObject::allocate)
        .SetMethod("queueFrame", &ControlObject::queue);
  }

  // Creates the object and sets a strong local ref to it
  static std::pair<v8::Local<v8::Object>, ControlObject*> create(
      v8::Isolate* isolate,
      gfx::Size resolution) {
    ControlObject* p = new ControlObject(isolate, resolution);
    return std::make_pair(p->wrapper(), p);
  }

  // Allocates a GC wrapper for a media::VideoFrame
  Frame* allocate(gfx::Size size, base::TimeDelta timestamp) {
    auto f = framePool_.CreateFrame(media::PIXEL_FORMAT_I420, size,
                                    gfx::Rect(size), size, timestamp);
    return new Frame(isolate(), f);
  }

  // Enqueues a GC wrapper of a media::VideoFrame
  void queue(Frame* frame, base::TimeTicks timestamp) {
    io_task_runner_->PostTask(FROM_HERE,
                              base::Bind(deliver_, frame->frame_, timestamp));
    frame->frame_ = nullptr;
  }

  // Allocates a non-GC wrapper for a media::VideoFrame
  // timestamp is in milliseconds
  CustomMediaStream::VideoFrame* allocateFrame(double timestamp,
                                               const Format* format) override {
    gfx::Size size =
        format ? gfx::Size{format->width, format->height} : resolution_;
    auto f = framePool_.CreateFrame(
        media::PIXEL_FORMAT_I420, size, gfx::Rect(size), size,
        base::TimeDelta::FromMillisecondsD(timestamp));
    return new NonGCFrame(f);
  }

  // Enqueues a non-GC wrapper of a media::VideoFrame
  // timestamp is in milliseconds
  void queueFrame(double timestamp,
                  CustomMediaStream::VideoFrame* frame) override {
    NonGCFrame* f = static_cast<NonGCFrame*>(frame);
    io_task_runner_->PostTask(
        FROM_HERE,
        base::Bind(deliver_, f->frame_,
                   base::TimeTicks::UnixEpoch() +
                       base::TimeDelta::FromMillisecondsD(timestamp)));
    delete f;
  }

  // Releases a con-GC wrapper of a media::VideoFrame
  void releaseFrame(CustomMediaStream::VideoFrame* frame) override {
    NonGCFrame* f = static_cast<NonGCFrame*>(frame);
    delete f;
  }

  v8::Isolate* isolate_;
  v8::Global<v8::Object> wrapper_;  // Weak

  gfx::Size resolution_;
  media::VideoCapturerSource::VideoCaptureDeliverFrameCB deliver_;
  media::VideoFramePool framePool_;
  const scoped_refptr<base::SingleThreadTaskRunner> io_task_runner_ =
      content::RenderThread::Get()->GetIOTaskRunner();

  static gin::WrapperInfo kWrapperInfo;
};

// Capture source object, holds the controller
// and manages emission of frames
// TODO: Mark virtual functions as virtual for better readibility
// TODO: remove unnecessary move calls
struct CustomCapturerSource : media::VideoCapturerSource {
  CustomCapturerSource(
      v8::Isolate* isolate,
      gfx::Size resolution,
      std::size_t framerate,
      base::RepeatingCallback<void(ControlObject*)> onStartCapture,
      base::RepeatingCallback<void()> onStopCapture)
      : format_(resolution, framerate, media::PIXEL_FORMAT_I420),
        onStartCapture_(std::move(onStartCapture)),
        onStopCapture_(std::move(onStopCapture)) {
    auto r = ControlObject::create(isolate, resolution);
    control_wrapper_ = v8::Global<v8::Value>(isolate, r.first);
    control_ = r.second;
  }

  ~CustomCapturerSource() override {
    onStartCapture_ = base::RepeatingCallback<void(ControlObject*)>();
    onStopCapture_ = base::RepeatingCallback<void()>();
    control_wrapper_.Reset();
  }

  media::VideoCaptureFormats GetPreferredFormats() override {
    return {format_};
  }

  void StartCapture(const media::VideoCaptureParams& params,
                    const VideoCaptureDeliverFrameCB& frame_callback,
                    const RunningCallback& running_callback) override {
    control_->deliver_ = frame_callback;
    running_callback.Run(true);
  }

  void StopCapture() override {}

  void Resume() override { onStartCapture_.Run(control_); }

  void MaybeSuspend() override {
    // In some circumstances this can be called
    // from Document::Shutdown
    if (!blink::ScriptForbiddenScope::IsScriptForbidden())
      onStopCapture_.Run();
  }

  // Controller wrapper
  v8::Global<v8::Value> control_wrapper_;

  // Controller
  ControlObject* control_;

  // Video format specification
  media::VideoCaptureFormat format_;

  // User-defined startCapture callback
  base::RepeatingCallback<void(ControlObject*)> onStartCapture_;

  // User-defined stopCapture callback
  base::RepeatingCallback<void()> onStopCapture_;
};

// Creates a WebKit media stream track based on the
// CustomCapturerSource which allows a user to
// provide his frames
// TODO: Groom
blink::WebMediaStreamTrack createTrack(
    v8::Isolate* isolate,
    gfx::Size resolution,
    int framerate,
    base::RepeatingCallback<void(ControlObject*)> onStartCapture,
    base::RepeatingCallback<void()> onStopCapture) {
  auto source = std::make_unique<CustomCapturerSource>(
      isolate, resolution, framerate, onStartCapture, onStopCapture);

  std::string str_track_id;
  base::Base64Encode(base::RandBytesAsString(64), &str_track_id);
  const blink::WebString track_id = blink::WebString::FromASCII(str_track_id);

  auto platform_source =
      std::make_unique<blink::MediaStreamVideoCapturerSource>(
          blink::WebPlatformMediaStreamSource::SourceStoppedCallback(),
          std::move(source));

  blink::MediaStreamVideoCapturerSource* t = platform_source.get();
  blink::WebMediaStreamSource webkit_source;
  webkit_source.Initialize(track_id, blink::WebMediaStreamSource::kTypeVideo,
                           track_id, false);
  webkit_source.SetPlatformSource(std::move(platform_source));

  auto platform_track = std::make_unique<blink::MediaStreamVideoTrack>(
      t, blink::MediaStreamVideoSource::ConstraintsCallback(), true);

  blink::WebMediaStreamTrack track;
  track.Initialize(webkit_source);
  track.SetPlatformTrack(std::move(platform_track));

  return track;
}

// Registers module functions
void Initialize(v8::Local<v8::Object> exports,
                v8::Local<v8::Value> unused,
                v8::Local<v8::Context> context,
                void* priv) {
  std::cout << "Initialize" << std::endl;
  mate::Dictionary dict(context->GetIsolate(), exports);
  dict.SetMethod("test", &test);
  dict.SetMethod("createTrack", &createTrack);
}

}  // namespace

// TODO: Why impl is here?
v8::Local<v8::Value> mate::Converter<ControlObject*>::ToV8(v8::Isolate* isolate,
                                                           ControlObject* v) {
  return v->wrapper();
}

gin::WrapperInfo ControlObject::kWrapperInfo = {gin::kEmbedderNativeGin};

NODE_LINKED_MODULE_CONTEXT_AWARE(atom_renderer_custom_media_stream, Initialize)
