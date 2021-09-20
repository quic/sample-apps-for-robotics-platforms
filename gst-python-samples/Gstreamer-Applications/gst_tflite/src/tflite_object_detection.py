#!/usr/bin/env python3
import sys
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

    # initialize GStreamer
    Gst.init(sys.argv)

    loop = GLib.MainLoop()

    # create element
    source = Gst.ElementFactory.make("qtiqmmfsrc", "source")
    capsfilter = Gst.ElementFactory.make("capsfilter", "caps-filter")
    qtimletflite = Gst.ElementFactory.make("qtimletflite", "qti-mletflite")
    queue = Gst.ElementFactory.make("queue", "queue")
    qtioverlay = Gst.ElementFactory.make("qtioverlay", "qti-overlay")
    sink = Gst.ElementFactory.make("waylandsink", "sink")

    # creating empty pipeline
    pipeline = Gst.Pipeline.new("tflite-object-detection")

    if not source or not capsfilter or not qtimletflite or not queue or not qtioverlay or not sink:
        print("Create element failed")
        sys.exit(1)

    # Build the pipeline
    pipeline.add(source)
    pipeline.add(capsfilter)
    pipeline.add(qtimletflite)
    pipeline.add(queue)
    pipeline.add(qtioverlay)
    pipeline.add(sink)

    if not Gst.Element.link(source, capsfilter):
        print("Source & Capsfilter can not be linked")
        sys.exit(1)
    if not Gst.Element.link(capsfilter, qtimletflite):
        print("Capsfilter & qtitflite can not be linked")
        sys.exit(1)
    if not Gst.Element.link(qtimletflite, queue):
        print("qtitflite & queue can not be linked")
        sys.exit(1)
    if not Gst.Element.link(queue, qtioverlay):
        print("Queue & Qtioverlay could not be linked")
        sys.exit(1)
    if not Gst.Element.link(qtioverlay, sink):
        print("Qtioverlay & Sink could not be linked")
        sys.exit(1)

    # Modify the properties for caps-fitler
    capsfilter.set_property("caps", Gst.caps_from_string(
        "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1,camera=0"))

    # Modify the properties for qtimle-tflite
    qtimletflite.set_property("config", "/data/misc/camera/mle_tflite.config")
    qtimletflite.set_property("model", "/data/misc/camera/detect.tflite")
    qtimletflite.set_property("labels", "/data/misc/camera/labelmap.txt")
    qtimletflite.set_property("postprocessing", "detection")

    # Modify the properties for sink
    sink.set_property("fullscreen", "true")
    sink.set_property("async", "true")
    sink.set_property("enable-last-sample", "true")

try:
    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    print("Object Detection Started..")
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Stopping Object Detection")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)
