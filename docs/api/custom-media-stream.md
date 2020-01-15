# customMediaStream

> Allows to create controlled [MediaStreamTrack](https://developer.mozilla.org/en-US/docs/Web/API/MediaStreamTrack) that can be attached to media stream via `MediaStream.addTrack()`. Custom media track can be used to convert bitmaps sequence to [MediaStream](https://developer.mozilla.org/en-US/docs/Web/API/MediaStream) for `<video>` element or transfer with `WebRTC`

Process: [Renderer](../glossary.md#renderer-process)

The following example shows how to create a media track, attach to a media stream and put it to the `<video>` element. Example stream is filled with green frames with varying saturation

```javascript
// In the renderer process.
const { customMediaStream } = require('electron')

let trackInfo = {
  resolution: { width: 1920, height: 1080 },
  frameRate: 30,
  pixelFormat: 3 // 1 - YUV12, 2 - BGRA, 3 - RGBA
}

let track = customMediaStream.createTrack(
  trackInfo,
  (controller) => {
    // Called when capture starts
    let i = 0
      this.interval = setInterval(() => {
        let frame = controller.allocateFrame()

        // frame.y, frame.u and frame.v are ArrayBuffers
        // containing pixels from each color plane.
        // Only 'y' plane is used for RGBA.
        var arr = new Uint8Array(frame.y);
        var e;
        for (e = 0; e < arr.length; e = e + 4) {
            arr[e + 0] = i % 255; // filling buffer with red
            arr[e + 1] = 0;
            arr[e + 2] = 0;
            arr[e + 3] = 255;
        }

        controller.queueFrame(frame, Date.now())
        i++;
      }, 10)
  },
  stopCapture: () => {
    // Called when capture stops
    clearInterval(this.interval)
  }
)

let mediaStream = new MediaStream()
mediaStream.addTrack(track)

// Attach to HTML5 <video> element
const videoElem = document.getElementById('video-element')
videoElem.srcObject = mediaStream
videoElem.play()
```



## Class: CustomMediaStream
### Static Methods

### `createTrack(info, onStartCapture, onStopCapture)`

* `info` Object
    * `resolution` [Size](structures/size.md) - The video frame size
    * `frameRate` Number - Frames per second parameter of resulting MediaStreamTrack
    * `pixelFormat` Number - Pixel format: 1 - YUV12, 2 - BGRA, 3 - RGBA
* `onStartCapture` Function - A callback that is called once the customTrack is instantiated and ready for frames. Recieves a `CMSVideoFramesController` instance as a single parameter for video track content manipulation
* `onStopCapture` Function - A callback that is called once before customTrack is done working with. e.g. when you reset the `srcObject` of a `<video>` element.

Returns `WebMediaStreamTrack` instance, that can be used in `<video>` element or for `WebRTC`

## Class: CMSVideoFramesController
There is an internal frame buffer which allows to avoid extra frame objects allocation. The `CMSVideoFramesController` instance wrapps the buffer and provides the following interface

### Instance methods
### `allocateFrame(resolution, pixelFormat, timestamp)`

* `frame` CMSFrameWrapper - frame to put into the custom video track (obtained from `allocateFrame` method)
* `captureTime` Number - timestamp [ms] of frame capture

Returns `CMSFrameWrapper` instance, which can be modified and put to the custom video track via `queueFrame` method

### `queueFrame(frame, captureTime)`
* `frame` CMSFrameWrapper - frame to put into the custom video track (obtained from `allocateFrame` method)
* `captureTime` Number - timestamp [ms] of frame capture

## Class: CMSFrameWrapper
### Instance properties
#### `frame.width`
A `Integer` representing frame width
#### `frame.height`
A `Integer` representing frame height
#### `frame.timestamp`
A `Integer` representing the frame capture time (timestamp [ms]). Affects frames presentation sequence - an implementation will not not present the frame before this point-in-time
##### Pixel format fields
The only available for now pixel coding format is [I420](http://www.fourcc.org/pixel-format/yuv-i420/) (12bpp YUV planar 1x1 Y, 2x2 UV samples) since it is one of the most popular ones in video applications. Each plane pixels data is represented by
#### `frame.y`
A `Uint8Array` representing luminance color component
#### `frame.u`
A `Uint8Array` representing blue chrominance projection
#### `frame.v`
A `Uint8Array` representing red chrominance projection
