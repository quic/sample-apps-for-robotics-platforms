#!/usr/bin/env python3
import sys
import gi

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib

if __name__ == "__main__":
    # Initialize GStreamer passing command line argument
    Gst.init(sys.argv)
    loop = GLib.MainLoop()

if len(sys.argv) != 3:
    print("Missing <output-location> parameter")
    sys.exit(1)
else:
    camera_id = int(sys.argv[1])
    if (camera_id < 0 or camera_id > 3):
        print("camera id incorrect.")
        sys.exit(1)


# Create the elements
source = Gst.ElementFactory.make("qtiqmmfsrc", "qmmf-source")
framefilter = Gst.ElementFactory.make("capsfilter", "frame-filter")
parse = Gst.ElementFactory.make("h264parse", "h264-parse")
muxer = Gst.ElementFactory.make("mpegtsmux", "mpegts-muxer")
queue = Gst.ElementFactory.make("queue", "queue")
sink = Gst.ElementFactory.make("filesink", "file-output")

# Create the empty pipeline
pipeline = Gst.Pipeline.new("video-encoding")

if not pipeline or not source or not framefilter or not parse or not muxer or not queue or not sink:
    print("Create pipeline/source/framefilter/parse/muxer/queue/sink element failed.")
    sys.exit

 # Set properties to elements
source.set_property("camera", camera_id)


if camera_id == 0:
    framefilter.set_property("caps", Gst.Caps.from_string(
        "video/x-h264,format=NV12,stream-format={byte-stream},framerate=30/1,width=1920, height=1080"))
else:
    framefilter.set_property("caps", Gst.Caps.from_string(
        "video/x-h264,framerate=30/1,width=1280, stream-format:byte-stream,height=720"))


sink.set_property("location", sys.argv[2])

# Build the pipeline
pipeline.add(source)
pipeline.add(framefilter)
pipeline.add(parse)
pipeline.add(muxer)
pipeline.add(queue)
pipeline.add(sink)

if not Gst.Element.link(source, framefilter):
    print("source and framefilter elements could not be linked.")
    sys.exit(1)
if not Gst.Element.link(framefilter, parse):
    print("framefilter and parse elements could not be linked.")
    sys.exit(1)
if not Gst.Element.link(parse, muxer):
    print("parse and muxer elements could not be linked.")
    sys.exit(1)
if not Gst.Element.link(muxer, queue):
    print("muxer and queue elements could not be linked.")
    sys.exit(1)
if not Gst.Element.link(queue, sink):
    print("queue and sink elements could not be linked.")
    sys.exit(1)


try:
    pipeline.set_state(Gst.State.PLAYING)
    print("Video recording started..")
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Video recording stopped")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)
