#!/usr/bin/env python3
import sys
import os
import gi
import logging

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib

logging.basicConfig(level=logging.DEBUG,
                    format="[%(name)s] [%(levelname)8s] - %(message)s")
logger = logging.getLogger(__name__)


if __name__ == "__main__":

    Gst.init(sys.argv)
    loop = GLib.MainLoop()

    if len(sys.argv) != 2:
        print("Usage: %s </dev/videoX>", sys.argv[0])
        sys.exit(1)

    # creating empty pipeline
    pipeline = Gst.Pipeline.new("video-display")

    # creating elements
    source = Gst.ElementFactory.make("v4l2src",      "v4l2-source")
    videoconv = Gst.ElementFactory.make("videoconvert", "video-convert")
    videorate = Gst.ElementFactory.make("videorate",    "video-rate")
    framefilter = Gst.ElementFactory.make("capsfilter",   "frame-filter")
    sink = Gst.ElementFactory.make("waylandsink",  "display")

    # videoconvert: Convert video frames between a great variety of video formats.
    # we add videoconvert to convert the format from different usb camera input.

    if not pipeline or not source or not videoconv or not videorate or not framefilter or not sink:
        logger.error("Create element failed")
        sys.exit(1)

    source.set_property("device", sys.argv[1])

     # Modify the properties for framefilter
    framefilter.set_property("caps",
                             Gst.caps_from_string("video/x-raw,framerate=15/1,width=1280,height=720"))

     # Modify the properties for sink
    sink.set_property("x", 200)
    sink.set_property("y", 200)
    sink.set_property("width", 1280)
    sink.set_property("height", 720)

    bus = pipeline.get_bus()
    bus_id = bus.add_signal_watch()
    Gst.Object.unref(bus)

    #Build the pipeline
    pipeline.add(source)
    pipeline.add(videoconv)
    pipeline.add(videorate)
    pipeline.add(framefilter)
    pipeline.add(sink)

    if not source.link(videoconv):
        print("ERROR: Could not link source to framefilter")
        sys.exit(1)
    if not videoconv.link(videorate):
        print("ERROR: Could not link framefilter to sink")
        sys.exit(1)
    if not videorate.link(framefilter):
        print("ERROR: Could not link source to framefilter")
        sys.exit(1)
    if not framefilter.link(sink):
        print("ERROR: Could not link framefilter to sink")
        sys.exit(1)

try:
    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    print("Started Liveview")
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Liveview stopped")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)
