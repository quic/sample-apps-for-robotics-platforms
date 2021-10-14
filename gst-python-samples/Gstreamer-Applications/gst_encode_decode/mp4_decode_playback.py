#!/usr/bin/env python3
import sys
import gi
import logging

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib, GObject

logging.basicConfig(level=logging.DEBUG,
                    format="[%(name)s] [%(levelname)8s] - %(message)s")
logger = logging.getLogger(__name__)


#Pad for demux 
def on_pad_added(bus, pad, data):
    srcpad = data
    sinkpad = Gst.Element.get_static_pad(srcpad, "sink")
    Gst.Pad.link(pad, sinkpad)
    return True


if __name__ == "__main__":
    # Initialize GStreamer passing command line argument
    Gst.init(sys.argv)

    loop = GLib.MainLoop()


if len(sys.argv) != 2:
    print("Missing <output-location> parametter")
    sys.exit(1)

# Create the empty pipeline
pipeline = Gst.Pipeline.new("video-player")

# Create the elements
source = Gst.ElementFactory.make("filesrc", "file-source")
sink = Gst.ElementFactory.make("waylandsink", "video-output")
decoder = Gst.ElementFactory.make("avdec_h264", "h264-decoder")
demuxer = Gst.ElementFactory.make("qtdemux", "qt-demuxer")
Videoconv = Gst.ElementFactory.make("videoconvert", "converter")

if not pipeline or not source or not demuxer or not decoder or not sink or not Videoconv:
    logger.error("Element not created.")
    sys.exit(1)
bus = pipeline.get_bus()
bus_id = bus.add_signal_watch()
Gst.Object.unref(bus)

# Build the pipeline
pipeline.add(source)
pipeline.add(demuxer)
pipeline.add(decoder)
pipeline.add(Videoconv)
pipeline.add(sink)


if not source.link(demuxer):
    print("ERROR: Could not link source to demux")
    sys.exit(1)

if not decoder.link(Videoconv):
    print("ERROR: Could not link decoder to video_convert")
    sys.exit(1)

if not Videoconv.link(sink):
    print("ERROR: Could not link scaler to sink")
    sys.exit(1)

source.set_property("location", sys.argv[1])

demuxer.connect("pad-added", on_pad_added, decoder)

try:
    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    print("Video playback started...")
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Video playback stopped...")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)