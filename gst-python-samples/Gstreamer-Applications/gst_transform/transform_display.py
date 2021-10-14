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


logging.basicConfig(level=logging.DEBUG,
                    format="[%(name)s] [%(levelname)8s] - %(message)s")
logger = logging.getLogger(__name__)


def usage(argv0):

    print("Usage: ", argv0, " [camera id: 0|1|2|3] [Operation]")
    print(" Operation:")
    print("    flip v|h (v:vertical, h:horizontal)")
    print("    rotate 1|2|3  (1:90CW, 2:90CCW, 3:180)")
    print("    crop x y width height")
    print(" Example:")
    print("    ",argv0," 0 flip v")
    print("    ",argv0," 0 flip h")
    print("    ",argv0," 0 rotate 2")
    print("    ",argv0," 0 crop 10 10 960 540")


if __name__ == "__main__":

    # initialize GStreamer
    Gst.init(sys.argv)

    loop = GLib.MainLoop()

    if len(sys.argv) < 3:
        usage(sys.argv[0])
        sys.exit(1)

    # checking camera id
    camera_id = int(sys.argv[1])
    if camera_id < 0:
        print("camera id incorrect.")
        usage(sys.argv[0])
        sys.exit(1)
    elif camera_id > 3:
        print("camera id incorrect.")
        usage(sys.argv[0])
        sys.exit(1)

    if sys.argv[2] == "flip":
        if len(sys.argv) == 4:
            if sys.argv[3] != "v" and sys.argv[3] != "h":
                print("flip should specify v or h.")
                usage(sys.argv[0])
                sys.exit(1)
        else:
            print("flip should specify v or h.")
            usage(sys.argv[0])
            sys.exit(1)
    elif sys.argv[2] == "rotate":
        if len(sys.argv) == 4:
            rotate_type = int(sys.argv[3])
            if rotate_type < 1 or rotate_type > 3:
                print("rotate type incorrect. (1|2|3)")
                usage(sys.argv[0])
                sys.exit(1)
        else:
            print("rotate type (1|2|3) needs to be specified")
            usage(sys.argv[0])
            sys.exit(1)
    elif sys.argv[2] == "crop":
        if len(sys.argv) != 7:
            print("crop position and size (x y w h) need to be specified")
            usage(sys.argv[0])
            sys.exit(1)
    else:
        print("Unknown operation:", sys.argv[2])
        usage(sys.argv[0])
        sys.exit(1)

    # creating empty pipeline
    pipeline = Gst.Pipeline.new("video-display")

    # create element
    source = Gst.ElementFactory.make("qtiqmmfsrc",   "qmmf-source")
    framefilter = Gst.ElementFactory.make("capsfilter",   "frame-filter")
    transform = Gst.ElementFactory.make("qtivtransform", "transform")
    sink = Gst.ElementFactory.make("waylandsink",  "display")

    if not pipeline or not source or not framefilter or not sink:
        logger.error("Element create failed.")
        sys.exit(1)

    source.set_property("camera", camera_id)

    # Modify the properties for framefilter
    if camera_id == 0:
        framefilter.set_property("caps",
                                 Gst.caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1920,height=1080"))
    else:
        framefilter.set_property("caps",
                                 Gst.caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1280,height=720"))

    if sys.argv[2] == "flip":
        if sys.argv[3] == "v":
            transform.set_property("flip-vertical", 1)
        elif sys.argv[3] == "h":
            transform.set_property("flip-horizontal", 1)
    elif sys.argv[2] == "rotate":
        transform.set_property("rotate", int(sys.argv[3]))
    elif sys.argv[2] == "crop":
        transform.set_property("crop-x", int(sys.argv[3]))
        transform.set_property("crop-y", int(sys.argv[4]))
        transform.set_property("crop-width", int(sys.argv[5]))
        transform.set_property("crop-height", int(sys.argv[6]))

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
    pipeline.add(transform)
    pipeline.add(sink)

    if not source.link(framefilter):
        print("ERROR: Could not link source to framefilter")
        sys.exit(1)
    if not framefilter.link(transform):
        print("ERROR: Could not link framefilter to transform")
        sys.exit(1)
    if not transform.link(sink):
        print("ERROR: Could not link transform to sink")
        sys.exit(1)

try:
    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    print("Start")
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Stopped")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)
