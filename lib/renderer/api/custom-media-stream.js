'use strict'

const internal = process.electronBinding('custom_media_stream')

// This function wraps call to native createTrack function
// in a way that allows user to avoid passing time-related arguments
// to allocateFrame and queueFrame functions
exports.createTrack = (info, onStartCapture, onStopCapture) => {
  function onStartCaptureDecorator (control) {
    const originalAllocateFrame = control.allocateFrame
    const originalQueueFrame = control.queueFrame

    const resolution = { width: info.resolution.width, height: info.resolution.height }
    const epoch = Date.now()

    control.allocateFrame = (format, timestamp) => {
      format = format || resolution
      timestamp = timestamp || (Date.now() - epoch)

      return originalAllocateFrame.call(control, format, timestamp)
    }

    control.queueFrame = (frame, estimatedCaptureTime) => {
      estimatedCaptureTime = estimatedCaptureTime || Date.now()

      originalQueueFrame.call(control, frame, estimatedCaptureTime)
    }

    onStartCapture(control)
  }

  return internal.createTrack(info.resolution, info.frameRate,
    onStartCaptureDecorator, onStopCapture)
}
