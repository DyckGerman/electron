'use strict'

const internal = process.electronBinding('custom_media_stream')

// This function wraps call to native createTrack function
// in a way that allows user to avoid passing time-related arguments
// to allocateFrame and queueFrame functions
exports.createTrack = (info, onStartCapture, onStopCapture) => {
  function onStartCaptureDecorator (control) {
    const originalAllocateFrame = control.allocateFrame
    const originalQueueFrame = control.queueFrame

    const defResolution = info.resolution
    const defPixelFormat = info.pixelFormat
    const epoch = Date.now()

    control.allocateFrame = (resolution, pixelFormat, timestamp) => {
      resolution = resolution || defResolution
      pixelFormat = pixelFormat || defPixelFormat
      timestamp = timestamp || (Date.now() - epoch)

      return originalAllocateFrame.call(control, resolution, pixelFormat, timestamp)
    }

    control.queueFrame = (frame, estimatedCaptureTime) => {
      estimatedCaptureTime = estimatedCaptureTime || Date.now()

      originalQueueFrame.call(control, frame, estimatedCaptureTime)
    }

    onStartCapture(control)
  }

  return internal.createTrack(info.resolution, info.frameRate, info.pixelFormat,
    onStartCaptureDecorator, onStopCapture)
}
