#!/usr/bin/env python3
from signal import signal
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
    # Initialize GStreamer passing command line argument
    Gst.init(sys.argv)

    loop = GLib.MainLoop()

    if len(sys.argv) != 2:
        print("Usage:[camera id: 0|1|2|3]", sys.argv[0])
        sys.exit(1)

    camera_id = int(sys.argv[1])
    if camera_id < 0:
        print("Usage: %s [camera id: 0|1|2|3]\n", sys.argv[0])
        sys.exit(1)
    elif camera_id > 3:
        print("Usage: %s [camera id: 0|1|2|3]\n", sys.argv[0])
        sys.exit(1)

    # creating empty pipeline
    pipeline = Gst.Pipeline.new("video-display")
    # create element
    source = Gst.ElementFactory.make("qtiqmmfsrc", "qmmf-source")
    framefilter = Gst.ElementFactory.make("capsfilter", "frame-filter")
    sink = Gst.ElementFactory.make("waylandsink", "display")

    if not pipeline or not source or not framefilter or not sink:
        logger.error("Create element failed")
        sys.exit(1)

    source.set_property("camera", camera_id)

    # Modify the properties for framefilter
    if camera_id == 0:
        framefilter.set_property("caps",
                                 Gst.caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1920,height=1080"))
    else:
        framefilter.set_property("caps",
                                 Gst.caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1280,height=720"))

    # Modify the properties for sink
    sink.set_property("x", 200)
    sink.set_property("y", 200)
    sink.set_property("width", 1280)
    sink.set_property("height", 720)

    bus = pipeline.get_bus()
    bus_id = bus.add_signal_watch()
    Gst.Object.unref(bus)

    # Build the pipeline
    pipeline.add(source)
    pipeline.add(framefilter)
    pipeline.add(sink)

    if not source.link(framefilter):
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
