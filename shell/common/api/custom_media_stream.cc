// Copyright (c) 2019 GitHub, Inc.
// Use of this source code is governed by the MIT license that can be
// found in the LICENSE file.

#include "shell/common/api/custom_media_stream.h"

#define INSIDE_BLINK \
  1  // we need to be able to convert WebMediaStreamTrack to
     // MediaStreamComponent*
#include <third_party/blink/public/platform/web_media_stream_track.h>
#undef INSIDE_BLINK

#include <base/base64.h>
#include <base/rand_util.h>

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

#include <memory>
#include <string>
#include <utility>

#include "electron/shell/common/native_mate_converters/callback.h"
#include "electron/shell/common/native_mate_converters/gfx_converter.h"
#include "electron/shell/common/node_includes.h"
#include "native_mate/dictionary.h"
#include "native_mate/wrappable.h"

namespace {

class ControllerWrapper;

}  // namespace

namespace mate {

// blink::WebMediaStreamTrack to v8::Value mate converter
template <>
struct Converter<blink::WebMediaStreamTrack> {
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate,
                                   blink::WebMediaStreamTrack track) {
    auto ctx = isolate->GetCurrentContext();
    auto* exec_ctx = blink::ToExecutionContext(ctx);
    auto* media_track = blink::MediaStreamTrack::Create(exec_ctx, track);
    return blink::ToV8(media_track, ctx->Global(), isolate);
  }
};

// v8::ArrayBuffer to v8::Value mate converter
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
    double ms = (v - base::TimeTicks::UnixEpoch()).InMillisecondsF();
    return v8::Number::New(isolate, ms);
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

// ControllerWrapper ptr to/from v8::Value mate converter
// Note that ToV8 returns a v8::Object that has internal fields
// while FromV8 just extracts ControllerWrapper pointer from an
// internal field
template <>
struct Converter<ControllerWrapper*> {
  static v8::Local<v8::Value> ToV8(v8::Isolate* isolate, ControllerWrapper* v);

  static bool FromV8(v8::Isolate* isolate,
                     v8::Local<v8::Value> value,
                     ControllerWrapper** out) {
    if (!value->IsObject())
      return false;

    v8::Local<v8::Object> obj = v8::Local<v8::Object>::Cast(value);
    if (obj->InternalFieldCount() != 2)
      return false;

    int fieldIdx = CustomMediaStream::VideoFramesController::kImplFieldIdx;
    auto* ptr = obj->GetAlignedPointerFromInternalField(fieldIdx);
    *out = static_cast<ControllerWrapper*>(ptr);
    return true;
  }
};

}  // namespace mate

namespace {

// GC wrapper for a media::VideoFrame object
// (When accessing the API from JS)
// TODO: groom, turn into a class
class FrameWrapper final : public mate::Wrappable<FrameWrapper> {
 public:
  static void BuildPrototype(v8::Isolate* isolate,
                             v8::Local<v8::FunctionTemplate> prototype) {
    auto classname = mate::StringToV8(isolate, "CustomMediaStreamVideoFrame");
    prototype->SetClassName(classname);

    mate::ObjectTemplateBuilder(isolate, prototype->PrototypeTemplate())
        .SetMethod("data", &FrameWrapper::data)
        .SetMethod("stride", &FrameWrapper::stride)
        .SetProperty("width", &FrameWrapper::width)
        .SetProperty("height", &FrameWrapper::height)
        .SetProperty("timestamp", &FrameWrapper::timestamp)
        .SetProperty("y", &FrameWrapper::y)
        .SetProperty("u", &FrameWrapper::u)
        .SetProperty("v", &FrameWrapper::v);
  }

  FrameWrapper(v8::Isolate* isolate, scoped_refptr<media::VideoFrame> frame)
      : frame_(std::move(frame)) {
    Init(isolate);
  }

  // Gets the frame width
  int width() const {
    if (!frame_)
      return 0;

    return frame_->visible_rect().width();
  }

  // Gets the frame height
  int height() const {
    if (!frame_)
      return 0;

    return frame_->visible_rect().height();
  }

  // Gets the stride of the plane
  int stride(int plane) const {
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

  // Gets the frame timestamp
  base::TimeDelta timestamp() const {
    if (!frame_)
      return base::TimeDelta();

    return frame_->timestamp();
  }

  // Creates an ArrayBuffer over an existing memory
  // of the frame_, memory is freed only when frame_ is deleted
  v8::Local<v8::ArrayBuffer> data(int plane) const {
    if (!frame_)
      return v8::Local<v8::ArrayBuffer>();

    switch (plane) {
      case media::VideoFrame::kYPlane:
      case media::VideoFrame::kUPlane:
      case media::VideoFrame::kVPlane:
        return v8::ArrayBuffer::New(
            isolate(), frame_->visible_data(plane),
            frame_->stride(plane) * frame_->rows(plane));
      default:
        return v8::Local<v8::ArrayBuffer>();
    }
  }

  // Gets an ArrayBuffer over the Y plane
  v8::Local<v8::ArrayBuffer> y() const {
    return data(media::VideoFrame::kYPlane);
  }

  // Gets an ArrayBuffer over the U plane
  v8::Local<v8::ArrayBuffer> u() const {
    return data(media::VideoFrame::kUPlane);
  }

  // Gets an ArrayBuffer over the V plane
  v8::Local<v8::ArrayBuffer> v() const {
    return data(media::VideoFrame::kVPlane);
  }

  // Extracts the wrapped frame
  scoped_refptr<media::VideoFrame> extractFrame() {
    scoped_refptr<media::VideoFrame> ret;
    ret.swap(frame_);
    return ret;
  }

 private:
  // Video frame
  scoped_refptr<media::VideoFrame> frame_;
};

// Non-GC wrapper for a media::VideoFrame object
// (When accessing the API from C++)
class NonGCFrameWrapper final : public CustomMediaStream::VideoFrame {
 public:
  explicit NonGCFrameWrapper(scoped_refptr<media::VideoFrame> frame)
      : frame_(frame) {}

  // Gets the frame format
  Format format() const override {
    return {frame_->visible_rect().width(), frame_->visible_rect().height()};
  }

  // Gets the stride of the plane
  int stride(Plane plane) const override {
    return frame_->stride(CMSPlaneToMediaPlane(plane));
  }

  // Gets rows count of the plane
  int rows(Plane plane) const override {
    return frame_->rows(CMSPlaneToMediaPlane(plane));
  }

  // Gets data of the plane
  void* data(Plane plane) const override {
    return frame_->visible_data(CMSPlaneToMediaPlane(plane));
  }

  // Gets the wrapped frame
  scoped_refptr<media::VideoFrame> frame() { return frame_; }

 private:
  // Helper converter between plane types
  static size_t CMSPlaneToMediaPlane(Plane p) {
    switch (p) {
      case Plane::Y:
        return media::VideoFrame::kYPlane;
      case Plane::U:
        return media::VideoFrame::kUPlane;
      case Plane::V:
        return media::VideoFrame::kVPlane;
      default:
        return media::VideoFrame::kYPlane;
    };
  }

  // Video frame
  scoped_refptr<media::VideoFrame> frame_;
};

// Controller wrapper object, holds media::VideoFramePool
// and allows to allocate, enqueue and deallocate frames
// NOTE: Not deriving from mate::wrappable because
// we need 2 internal fields
// TODO: improve naming, may be name all wrappers as ***Wrapper?
// TODO: turn into a class, groom
// TODO: incapsulate private members, add accessors
class ControllerWrapper final
    : public CustomMediaStream::VideoFramesController {
 public:
  // Ctor sets only the weak reference to the wrapper
  // so you need to call wrapper() function after ctor
  // otherwise it will be garbage collected
  ControllerWrapper(v8::Isolate* isolate, gfx::Size resolution)
      : isolate_(isolate), resolution_(resolution) {
    auto templ = GetConstructor(isolate);
    auto ctx = isolate->GetCurrentContext();

    v8::Local<v8::Object> wrapper;
    CHECK(templ->InstanceTemplate()->NewInstance(ctx).ToLocal(&wrapper));

    wrapper_.Reset(isolate, wrapper);
    wrapper_.SetWeak(this, FirstWeakCallback, v8::WeakCallbackType::kParameter);

    auto* base = static_cast<CustomMediaStream::VideoFramesController*>(this);
    auto wrap = this->wrapper();
    wrap->SetAlignedPointerInInternalField(kImplFieldIdx, this);
    wrap->SetAlignedPointerInInternalField(kInterfaceFieldIdx, base);
  }

  v8::Local<v8::Object> wrapper() const {
    return v8::Local<v8::Object>::New(isolate_, wrapper_);
  }

  v8::Isolate* isolate() const { return isolate_; }

  ~ControllerWrapper() {
    if (wrapper_.IsEmpty())
      return;

    wrapper()->SetAlignedPointerInInternalField(kImplFieldIdx, nullptr);
    wrapper()->SetAlignedPointerInInternalField(kInterfaceFieldIdx, nullptr);
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
      const v8::WeakCallbackInfo<ControllerWrapper>& data) {
    ControllerWrapper* p = data.GetParameter();
    p->wrapper_.Reset();
    data.SetSecondPassCallback(SecondWeakCallback);
  }

  static void SecondWeakCallback(
      const v8::WeakCallbackInfo<ControllerWrapper>& data) {
    ControllerWrapper* c = data.GetParameter();
    delete c;
  }

  static void BuildPrototype(v8::Isolate* isolate,
                             v8::Local<v8::FunctionTemplate> prototype) {
    prototype->SetClassName(
        mate::StringToV8(isolate, "CustomMediaStreamController"));
    mate::ObjectTemplateBuilder(isolate, prototype->PrototypeTemplate())
        .SetMethod("allocateFrame", &ControllerWrapper::allocate)
        .SetMethod("queueFrame", &ControllerWrapper::queue);
  }

  // Creates the object and sets a strong local ref to it
  static std::pair<v8::Local<v8::Object>, ControllerWrapper*> create(
      v8::Isolate* isolate,
      gfx::Size resolution) {
    ControllerWrapper* p = new ControllerWrapper(isolate, resolution);
    return std::make_pair(p->wrapper(), p);
  }

  // Allocates a GC wrapper for a media::VideoFrame
  FrameWrapper* allocate(gfx::Size size, base::TimeDelta timestamp) {
    auto f = framePool_.CreateFrame(media::PIXEL_FORMAT_I420, size,
                                    gfx::Rect(size), size, timestamp);
    return new FrameWrapper(isolate(), f);
  }

  // Enqueues a GC wrapper of a media::VideoFrame
  void queue(FrameWrapper* framewapper, base::TimeTicks timestamp) {
    auto f = framewapper->extractFrame();
    io_task_runner_->PostTask(FROM_HERE, base::Bind(deliver_, f, timestamp));
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
    return new NonGCFrameWrapper(f);
  }

  // Enqueues a non-GC wrapper of a media::VideoFrame
  // timestamp is in milliseconds
  void queueFrame(double timestamp,
                  CustomMediaStream::VideoFrame* frame) override {
    NonGCFrameWrapper* f = static_cast<NonGCFrameWrapper*>(frame);
    io_task_runner_->PostTask(
        FROM_HERE,
        base::Bind(deliver_, f->frame(),
                   base::TimeTicks::UnixEpoch() +
                       base::TimeDelta::FromMillisecondsD(timestamp)));
    delete f;
  }

  // Releases a con-GC wrapper of a media::VideoFrame
  void releaseFrame(CustomMediaStream::VideoFrame* frame) override {
    NonGCFrameWrapper* f = static_cast<NonGCFrameWrapper*>(frame);
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

gin::WrapperInfo ControllerWrapper::kWrapperInfo = {gin::kEmbedderNativeGin};

// Capture source object, holds the controller
// and manages emission of frames
// TODO: Mark virtual functions as virtual for better readibility
// TODO: remove unnecessary move calls
struct CustomCapturerSource : media::VideoCapturerSource {
  CustomCapturerSource(
      v8::Isolate* isolate,
      gfx::Size resolution,
      std::size_t framerate,
      base::RepeatingCallback<void(ControllerWrapper*)> onStartCapture,
      base::RepeatingCallback<void()> onStopCapture)
      : format_(resolution, framerate, media::PIXEL_FORMAT_I420),
        onStartCapture_(std::move(onStartCapture)),
        onStopCapture_(std::move(onStopCapture)) {
    auto r = ControllerWrapper::create(isolate, resolution);
    control_wrapper_ = v8::Global<v8::Value>(isolate, r.first);
    control_ = r.second;
  }

  ~CustomCapturerSource() override {
    onStartCapture_ = base::RepeatingCallback<void(ControllerWrapper*)>();
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
  ControllerWrapper* control_;

  // Video format specification
  media::VideoCaptureFormat format_;

  // User-defined startCapture callback
  base::RepeatingCallback<void(ControllerWrapper*)> onStartCapture_;

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
    base::RepeatingCallback<void(ControllerWrapper*)> onStartCapture,
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
  mate::Dictionary dict(context->GetIsolate(), exports);

  dict.SetMethod("createTrack", &createTrack);
}

}  // namespace

namespace mate {
v8::Local<v8::Value> mate::Converter<ControllerWrapper*>::ToV8(
    v8::Isolate* isolate,
    ControllerWrapper* v) {
  return v->wrapper();
}
}  // namespace mate

NODE_LINKED_MODULE_CONTEXT_AWARE(atom_renderer_custom_media_stream, Initialize)
