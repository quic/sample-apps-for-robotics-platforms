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

if len(sys.argv) != 4:
    print("Missing <output-location> parameter")
    sys.exit(1)
else:
    camera_id = int(sys.argv[1])
    if (camera_id < 0 or camera_id > 3):
        print("camera id incorrect")
        sys.exit(1)
    port = int(sys.argv[3])
    if (port < 0):
        print("port incorrect.")
        sys.exit(1)

# Create the empty pipeline
pipeline = Gst.Pipeline.new("video-streaming")

# Create the elements
source = Gst.ElementFactory.make("qtiqmmfsrc", "qmmf-source")
framefilter = Gst.ElementFactory.make("capsfilter",   "frame-filter")
parse = Gst.ElementFactory.make("h264parse",    "h264-parse")
muxer = Gst.ElementFactory.make("mpegtsmux",    "mpegts-muxer")
queue = Gst.ElementFactory.make("queue",        "queue")
sink = Gst.ElementFactory.make("tcpserversink",  "tcp-server")

if not pipeline or not source or not framefilter or not parse or not queue or not sink:
    print("Create element failed.")
    sys.exit(1)

# set properties to the elements
source.set_property("camera", camera_id)

if camera_id == 0:
    framefilter.set_property("caps", Gst.Caps.from_string(
        "video/x-h264, framerate=30/1,width=1920, height=1080"))
else:
    framefilter.set_property("caps", Gst.Caps.from_string(
        "video/x-h264, framerate=30/1,width=1280, height=720"))

parse.set_property("config-interval", 1)
sink.set_property("host", sys.argv[2])
sink.set_property("port", port)

# Build the pipeline
pipeline.add(source)
pipeline.add(framefilter)
pipeline.add(parse)
pipeline.add(muxer)
pipeline.add(queue)
pipeline.add(sink)

if not Gst.Element.link(source, framefilter):
    print("ERROR: Could not link source to framefilter")
    sys.exit(1)
if not Gst.Element.link(framefilter, parse):
    print("ERROR: Could not link framefilter to parse")
    sys.exit(1)
if not Gst.Element.link(parse, muxer):
    print("ERROR: Could not link parse to muxer")
    sys.exit(1)
if not Gst.Element.link(muxer, queue):
    print("ERROR: Could not link muxer to queue")
    sys.exit(1)
if not Gst.Element.link(queue, sink):
    print("ERROR: Could not link queue to sink")
    sys.exit(1)

try:
    pipeline.set_state(Gst.State.PLAYING)
    print("Streaming started...")
    print("vlc tcp://%s:%d/" % (sys.argv[2], port))
    print('Press Ctrl+C to stop')
    loop.run()
except KeyboardInterrupt:
    print("Streaming stopped due to keyboard Interrupt")
    pipeline.set_state(Gst.State.NULL)
    sys.exit(0)
