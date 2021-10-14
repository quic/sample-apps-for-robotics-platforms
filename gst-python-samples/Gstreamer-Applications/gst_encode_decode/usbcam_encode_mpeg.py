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


if len(sys.argv) < 3:
    print("Missing <output-location> parameter")
    sys.exit(1)


# Create the empty pipeline
pipeline = Gst.Pipeline.new("video-encoder")

# Create the elements
source = Gst.ElementFactory.make("v4l2src",   "v4l2-source")
videoconv = Gst.ElementFactory.make("videoconvert",   "converter")
framefilter = Gst.ElementFactory.make("capsfilter", "frame-filter")
encoder = Gst.ElementFactory.make("x264enc",    "x264-encoder")
muxer = Gst.ElementFactory.make("mpegtsmux",       "mpeg-muxer")
queue = Gst.ElementFactory.make("queue",        "queue")
sink = Gst.ElementFactory.make("filesink",     "file-output")

if not pipeline or not source or not videoconv or not framefilter or not muxer or not encoder or not sink or not queue:
    print("Element not created.")
    sys.exit(1)

source.set_property("device", sys.argv[1])
source.set_property("num-buffers", 300)
framefilter.set_property("caps", Gst.caps_from_string(
    "video/x-raw,framerate=30/1,width=1280,height=720"))
sink.set_property("location", sys.argv[2])

# Build the pipeline
pipeline.add(source)
pipeline.add(videoconv)
pipeline.add(framefilter)
pipeline.add(encoder)
pipeline.add(muxer)
pipeline.add(queue)
pipeline.add(sink)

if not source.link(videoconv):
    print("ERROR: Could not link source to framefilter")
    sys.exit(1)

if not videoconv.link(framefilter):
    print("ERROR: Could not link framefilter to parse")
    sys.exit(1)

if not framefilter.link(encoder):
    print("ERROR: Could not link framefilter to parse")
    sys.exit(1)

if not encoder.link(muxer):
    print("ERROR: Could not link encoder to muxer")
    sys.exit(1)

if not muxer.link(queue):
    print("ERROR: Could not muxer muxer to queue")
    sys.exit(1)

if not queue.link(sink):
    print("ERROR: Could not link queue to sink")
    sys.exit(1)

try:
    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    print("Video recording started..")
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Video recording stopped")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)
