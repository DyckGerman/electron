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
class FrameWrapper final : public mate::Wrappable<FrameWrapper> {
 public:
  static void BuildPrototype(v8::Isolate* isolate,
                             v8::Local<v8::FunctionTemplate> prototype) {
    auto classname = mate::StringToV8(isolate, "CustomMediaStreamVideoFrame");
    prototype->SetClassName(classname);

    mate::ObjectTemplateBuilder(isolate, prototype->PrototypeTemplate())
        .SetMethod("data", &FrameWrapper::data)
        .SetMethod("stride", &FrameWrapper::stride)
        .SetMethod("rows", &FrameWrapper::rows)
        .SetMethod("rowBytes", &FrameWrapper::rowBytes)
        .SetProperty("width", &FrameWrapper::width)
        .SetProperty("height", &FrameWrapper::height)
        .SetProperty("timestamp", &FrameWrapper::timestamp)
        .SetProperty("y", &FrameWrapper::y)
        .SetProperty("u", &FrameWrapper::u)
        .SetProperty("v", &FrameWrapper::v)
        .SetProperty("v", &FrameWrapper::a);
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
    if (!frame_ || !IsValidMediaPlane(plane))
      return 0;

    return frame_->stride(IntPlaneToMediaPlane(plane));
  }

  // Gets rows count of the plane
  int rows(int plane) const {
    if (!frame_ || !IsValidMediaPlane(plane))
      return 0;

    return frame_->rows(IntPlaneToMediaPlane(plane));
  }

  // Gets bytes count per row of the plane
  int rowBytes(int plane) const {
    if (!frame_ || !IsValidMediaPlane(plane))
      return 0;

    return frame_->row_bytes(IntPlaneToMediaPlane(plane));
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
    if (!frame_ || !IsValidMediaPlane(plane))
      return v8::Local<v8::ArrayBuffer>();

    size_t mediaPlane = IntPlaneToMediaPlane(plane);
    return v8::ArrayBuffer::New(
        isolate(), frame_->visible_data(mediaPlane),
        frame_->stride(mediaPlane) * frame_->rows(mediaPlane));
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

  // Gets an ArrayBuffer over the A plane
  v8::Local<v8::ArrayBuffer> a() const {
    return data(media::VideoFrame::kAPlane);
  }

  // Extracts the wrapped frame
  scoped_refptr<media::VideoFrame> extractFrame() {
    scoped_refptr<media::VideoFrame> ret;
    ret.swap(frame_);
    return ret;
  }

 private:
  // Helper check that plane is valid
  static bool IsValidMediaPlane(int p) { return (p >= 0 && p <= 3); }

  // Helper converter between plane types
  static size_t IntPlaneToMediaPlane(int p) {
    switch (p) {
      case 0:
        return media::VideoFrame::kYPlane;
      case 1:
        return media::VideoFrame::kUPlane;
      case 2:
        return media::VideoFrame::kVPlane;
      case 3:
        return media::VideoFrame::kAPlane;
      default:
        return media::VideoFrame::kYPlane;
    };
  }

  // Video frame
  scoped_refptr<media::VideoFrame> frame_;
};

// Non-GC wrapper for a media::VideoFrame object
// (When accessing the API from C++)
class NonGCFrameWrapper final : public CustomMediaStream::VideoFrame {
 public:
  using CMSPixelFormat = CustomMediaStream::VideoFrame::PixelFormat;

  // Converts from PixelFormat to media::VideoPixelFormat
  static media::VideoPixelFormat CMSToMediaPixelFormat(CMSPixelFormat cms_pf) {
    switch (cms_pf) {
      case CMSPixelFormat::I420:
        return media::PIXEL_FORMAT_I420;
      case CMSPixelFormat::ARGB:
        return media::PIXEL_FORMAT_ARGB;
      case CMSPixelFormat::ABGR:
        return media::PIXEL_FORMAT_ABGR;
      default:
        return media::PIXEL_FORMAT_UNKNOWN;
    }
  }

  // Converts from media::VideoPixelFormat to PixelFormat
  static CMSPixelFormat MediaToCMSPixelFormat(
      media::VideoPixelFormat media_pf) {
    switch (media_pf) {
      case media::PIXEL_FORMAT_I420:
        return CMSPixelFormat::I420;
      case media::PIXEL_FORMAT_ARGB:
        return CMSPixelFormat::ARGB;
      case media::PIXEL_FORMAT_ABGR:
        return CMSPixelFormat::ABGR;
      default:
        return CMSPixelFormat::UNKNOWN;
    }
  }

  // Converts from int to PixelFormat
  static CMSPixelFormat IntToCMSPixelFormat(int pf) {
    switch (pf) {
      case 1:
        return CMSPixelFormat::I420;
      case 2:
        return CMSPixelFormat::ARGB;
      case 3:
        return CMSPixelFormat::ABGR;
      default:
        return CMSPixelFormat::UNKNOWN;
    }
  }

  // Converts from int to media::VideoPixelFormat
  static media::VideoPixelFormat IntToMediaPixelFormat(int pf) {
    return CMSToMediaPixelFormat(IntToCMSPixelFormat(pf));
  }

  explicit NonGCFrameWrapper(scoped_refptr<media::VideoFrame> frame)
      : frame_(frame) {}

  // Gets the frame format
  Format format() const override {
    Format fmt = {MediaToCMSPixelFormat(frame_->format()),
                  frame_->visible_rect().width(),
                  frame_->visible_rect().height()};
    return fmt;
  }

  // Gets the stride of the plane
  int stride(Plane plane) const override {
    return frame_->stride(CMSPlaneToMediaPlane(plane));
  }

  // Gets rows count of the plane
  int rows(Plane plane) const override {
    return frame_->rows(CMSPlaneToMediaPlane(plane));
  }

  // Gets bytes count per row of the plane
  int rowBytes(Plane plane) const override {
    return frame_->row_bytes(CMSPlaneToMediaPlane(plane));
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
      case Plane::A:
        return media::VideoFrame::kAPlane;
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
class ControllerWrapper final
    : public CustomMediaStream::VideoFramesController {
 public:
  using ControllerPtrs = std::pair<v8::Local<v8::Object>, ControllerWrapper*>;
  using DeliverFrameCB = media::VideoCapturerSource::VideoCaptureDeliverFrameCB;
  using TaskRunnerPtr = scoped_refptr<base::SingleThreadTaskRunner>;
  using RenderThread = content::RenderThread;
  using VideoFrame = CustomMediaStream::VideoFrame;
  using CMSPixelFormat = VideoFrame::PixelFormat;

  // Creates the object and sets a strong local ref to it
  static ControllerPtrs create(v8::Isolate* isolate,
                               gfx::Size resolution,
                               CMSPixelFormat pixel_format) {
    auto* p = new ControllerWrapper(isolate, resolution, pixel_format);
    return std::make_pair(p->wrapper(), p);
  }

  // Ctor sets only the weak reference to the wrapper
  // so you need to call wrapper() function after ctor
  // otherwise it will be garbage collected.
  // Sets this and base pointers into the internal fields
  ControllerWrapper(v8::Isolate* isolate,
                    gfx::Size resolution,
                    CMSPixelFormat cms_pf)
      : isolate_(isolate), resolution_(resolution), pixel_format_(cms_pf) {
    auto templ = GetConstructor(isolate);
    auto ctx = isolate->GetCurrentContext();

    v8::Local<v8::Object> wrapper;
    CHECK(templ->InstanceTemplate()->NewInstance(ctx).ToLocal(&wrapper));

    wrapper_.Reset(isolate, wrapper);
    wrapper_.SetWeak(this, FirstCbk, v8::WeakCallbackType::kParameter);

    auto* base = static_cast<CustomMediaStream::VideoFramesController*>(this);
    wrapper->SetAlignedPointerInInternalField(kImplFieldIdx, this);
    wrapper->SetAlignedPointerInInternalField(kInterfaceFieldIdx, base);
  }

  ~ControllerWrapper() override {
    if (wrapper_.IsEmpty())
      return;

    wrapper()->SetAlignedPointerInInternalField(kImplFieldIdx, nullptr);
    wrapper()->SetAlignedPointerInInternalField(kInterfaceFieldIdx, nullptr);
    wrapper_.ClearWeak();
    wrapper_.Reset();
  }

  // Creates a strong local ref
  v8::Local<v8::Object> wrapper() const {
    return v8::Local<v8::Object>::New(isolate(), wrapper_);
  }

  // Gets the isolate
  v8::Isolate* isolate() const { return isolate_; }

  // Sets the frame deliver callback
  void setDeliverCb(const DeliverFrameCB& cb) { deliver_ = cb; }

  // Allocates a GC wrapper for a media::VideoFrame
  FrameWrapper* allocateGCFrame(gfx::Size size,
                                int pixelFormat,
                                base::TimeDelta timestamp) {
    auto media_pf = NonGCFrameWrapper::IntToMediaPixelFormat(pixelFormat);
    auto f = frame_pool_.CreateFrame(media_pf, size, gfx::Rect(size), size,
                                     timestamp);
    return new FrameWrapper(isolate(), f);
  }

  // Enqueues a GC wrapper of a media::VideoFrame
  // Frame will be delivered in a renderer thread queue
  void queueGCFrame(FrameWrapper* framewapper, base::TimeTicks timestamp) {
    auto f = framewapper->extractFrame();
    io_task_runner_->PostTask(FROM_HERE, base::Bind(deliver_, f, timestamp));
  }

  // Allocates a non-GC wrapper for a media::VideoFrame
  // timestamp is in milliseconds
  VideoFrame* allocateFrame(double timestamp, const Format* format) override {
    auto size = format ? gfx::Size(format->width, format->height) : resolution_;
    auto pf = format ? format->pixel_format : pixel_format_;
    auto media_pf = NonGCFrameWrapper::CMSToMediaPixelFormat(pf);

    auto f =
        frame_pool_.CreateFrame(media_pf, size, gfx::Rect(size), size,
                                base::TimeDelta::FromMillisecondsD(timestamp));
    return new NonGCFrameWrapper(f);
  }

  // Enqueues a non-GC wrapper of a media::VideoFrame
  // Frame will be delivered in a renderer thread queue
  // timestamp is in milliseconds
  void queueFrame(double timestamp, VideoFrame* frame) override {
    auto f = static_cast<NonGCFrameWrapper*>(frame)->frame();
    auto delta = base::TimeDelta::FromMillisecondsD(timestamp);
    auto ticks = base::TimeTicks::UnixEpoch() + delta;
    io_task_runner_->PostTask(FROM_HERE, base::Bind(deliver_, f, ticks));
    delete frame;
  }

  // Releases a con-GC wrapper of a media::VideoFrame
  void releaseFrame(VideoFrame* frame) override {
    NonGCFrameWrapper* f = static_cast<NonGCFrameWrapper*>(frame);
    delete f;
  }

 private:
  // Creates or retrieves an existing function template of this wrapper
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

  // Registers methods for the prototype template
  static void BuildPrototype(v8::Isolate* isolate,
                             v8::Local<v8::FunctionTemplate> prototype) {
    auto classname = mate::StringToV8(isolate, "CustomMediaStreamController");
    prototype->SetClassName(classname);
    mate::ObjectTemplateBuilder(isolate, prototype->PrototypeTemplate())
        .SetMethod("allocateFrame", &ControllerWrapper::allocateGCFrame)
        .SetMethod("queueFrame", &ControllerWrapper::queueGCFrame);
  }

  // First weak callback, resets the weak ref
  static void FirstCbk(const v8::WeakCallbackInfo<ControllerWrapper>& data) {
    auto* p = data.GetParameter();
    p->wrapper_.Reset();
    data.SetSecondPassCallback(SecondCbk);
  }

  // Second weak callack, deletes the wrapper
  static void SecondCbk(const v8::WeakCallbackInfo<ControllerWrapper>& data) {
    auto* c = data.GetParameter();
    delete c;
  }

 private:
  // Class-related wrapper info, its pointer is the key in the PerIsolate map
  static gin::WrapperInfo kWrapperInfo;

  // Isolate
  v8::Isolate* isolate_;

  // Weak wrapper ref
  v8::Global<v8::Object> wrapper_;

  // Default frames resolution
  gfx::Size resolution_;

  // Default pixel format
  CMSPixelFormat pixel_format_;

  // Video frames deliver callback, signalizes when a frame is ready
  DeliverFrameCB deliver_;

  // media frames pool
  media::VideoFramePool frame_pool_;

  // IO task runner, used to deliver the frames
  const TaskRunnerPtr io_task_runner_ = RenderThread::Get()->GetIOTaskRunner();
};

gin::WrapperInfo ControllerWrapper::kWrapperInfo = {gin::kEmbedderNativeGin};

// Capture source object, holds the controller
// and manages emission of frames
class CustomCapturerSource final : public media::VideoCapturerSource {
 public:
  using CMSPixelFormat = CustomMediaStream::VideoFrame::PixelFormat;

  CustomCapturerSource(
      v8::Isolate* isolate,
      gfx::Size resolution,
      std::size_t framerate,
      CMSPixelFormat pixel_format,
      const base::RepeatingCallback<void(ControllerWrapper*)>& on_start_capture,
      const base::RepeatingCallback<void()>& on_stop_capture)
      : on_start_capture_(on_start_capture), on_stop_capture_(on_stop_capture) {
    format_ = {resolution, framerate,
               NonGCFrameWrapper::CMSToMediaPixelFormat(pixel_format)};

    auto wrapper_ptrs =
        ControllerWrapper::create(isolate, resolution, pixel_format);
    control_wrapper_ = v8::Global<v8::Value>(isolate, wrapper_ptrs.first);
    control_ = wrapper_ptrs.second;
  }

  ~CustomCapturerSource() override {
    on_start_capture_.Reset();
    on_stop_capture_.Reset();
    control_wrapper_.Reset();
  }

  // Gets the default frame format
  media::VideoCaptureFormats GetPreferredFormats() override {
    return {format_};
  }

  // Called when a media stream is ready to receive frames
  void StartCapture(const media::VideoCaptureParams& params,
                    const VideoCaptureDeliverFrameCB& frame_callback,
                    const RunningCallback& running_callback) override {
    control_->setDeliverCb(frame_callback);
    running_callback.Run(true);
  }

  // Called when a media stream wants no more frames
  void StopCapture() override {}

  // Called when a media stream gets the first cunsumer
  void Resume() override { on_start_capture_.Run(control_); }

  // Called when a media stream loses all the consumers
  void MaybeSuspend() override {
    // In some circumstances this can be called
    // from Document::Shutdown, thus this check
    if (!blink::ScriptForbiddenScope::IsScriptForbidden())
      on_stop_capture_.Run();
  }

 private:
  // Controller wrapper
  v8::Global<v8::Value> control_wrapper_;

  // Controller
  ControllerWrapper* control_;

  // Video format specification
  media::VideoCaptureFormat format_;

  // User-defined startCapture callback
  base::RepeatingCallback<void(ControllerWrapper*)> on_start_capture_;

  // User-defined stopCapture callback
  base::RepeatingCallback<void()> on_stop_capture_;
};

// Creates a WebKit media stream track based on the
// CustomCapturerSource which allows a user to
// provide his frames
blink::WebMediaStreamTrack createTrack(
    v8::Isolate* isolate,
    gfx::Size resolution,
    int framerate,
    int pixel_format,
    base::RepeatingCallback<void(ControllerWrapper*)> on_start_capture,
    base::RepeatingCallback<void()> on_stop_capture) {
  std::string random_str;
  base::Base64Encode(base::RandBytesAsString(64), &random_str);
  const auto track_id = blink::WebString::FromASCII(random_str);

  auto cms_pf = NonGCFrameWrapper::IntToCMSPixelFormat(pixel_format);

  auto custom_source = std::make_unique<CustomCapturerSource>(
      isolate, resolution, framerate, cms_pf, on_start_capture,
      on_stop_capture);
  auto platform_source =
      std::make_unique<blink::MediaStreamVideoCapturerSource>(
          blink::WebPlatformMediaStreamSource::SourceStoppedCallback(),
          std::move(custom_source));
  auto* platform_source_raw = platform_source.get();

  blink::WebMediaStreamSource webkit_source;
  webkit_source.Initialize(track_id, blink::WebMediaStreamSource::kTypeVideo,
                           track_id, false);
  webkit_source.SetPlatformSource(std::move(platform_source));

  auto platform_track = std::make_unique<blink::MediaStreamVideoTrack>(
      platform_source_raw, blink::MediaStreamVideoSource::ConstraintsCallback(),
      true);

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
