import sys
import gi
import logging
import os

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

if len(sys.argv) < 1:
    print("Missing <output-location> parameter")
    sys.exit(1)

file_format = os.path.splitext(sys.argv[1])[1]
print(file_format)

# Create the empty pipeline
pipeline = Gst.Pipeline.new("audio playback")
filesrc = Gst.ElementFactory.make("filesrc", "file_src")
pulsesink = Gst.ElementFactory.make("pulsesink",  "pulseaudio_sink")

if not pipeline or not filesrc or not pulsesink:
    print("Create element failed.")
    sys.exit(1)

filesrc.set_property('location', sys.argv[1])

if file_format == ".wav":
    wavparse = Gst.ElementFactory.make("wavparse", "wav_parse")

    if not wavparse:
        print("Create element failed.\n")
        sys.exit(1)

    pipeline.add(filesrc)
    pipeline.add(wavparse)
    pipeline.add(pulsesink)

    if not filesrc.link(wavparse):
        print("ERROR: Could not link filesrc to wavparse")
        sys.exit(1)

    if not wavparse.link(pulsesink):
        print("ERROR: Could not link decoder to video_convert")
        sys.exit(1)

elif file_format == ".aac":

    aacparse = Gst.ElementFactory.make("aacparse",  "aac_parse")
    avdec_aac = Gst.ElementFactory.make("avdec_aac", "avdec_aac")

    if not aacparse or not avdec_aac:
        print("Create element failed.\n")
        sys.exit(1)

    pipeline.add(filesrc)
    pipeline.add(aacparse)
    pipeline.add(avdec_aac)
    pipeline.add(pulsesink)

    if not filesrc.link(aacparse):
        print("ERROR: Could not link filesrc to aacparse")
        sys.exit(1)

    if not aacparse.link(avdec_aac):
        print("ERROR: Could not link aacparse to avdec_aac")
        sys.exit(1)

    if not avdec_aac.link(pulsesink):
        print("ERROR: Could not link avdec_aac to pulsesink")
        sys.exit(1)

elif file_format == ".mp3":

    mpegaudioparse = Gst.ElementFactory.make(
        "mpegaudioparse", "mpegaudio_parse")
    avdec_mp3 = Gst.ElementFactory.make("avdec_mp3",      "avdec_mp3")

    if not mpegaudioparse or not avdec_mp3:
        print("Create element failed.\n")
        sys.exit(1)

    pipeline.add(filesrc)
    pipeline.add(mpegaudioparse)
    pipeline.add(avdec_mp3)
    pipeline.add(pulsesink)

    if not filesrc.link(mpegaudioparse):
        print("ERROR: Could not link filesrc to mpegaudioparse")
        sys.exit(1)

    if not mpegaudioparse.link(avdec_mp3):
        print("ERROR: Could not link mpegaudioparse to avdec_mp3")
        sys.exit(1)

    if not avdec_mp3.link(pulsesink):
        print("ERROR: Could not link avdec_mp3 to pulsesink")
        sys.exit(1)
else:
    print("Format not supported\n")
    sys.exit(1)


pipeline.set_state(Gst.State.PLAYING)
print("audio playback started")
bus = pipeline.get_bus()
bus_id = bus.add_signal_watch()
msg = bus.timed_pop_filtered(
    Gst.CLOCK_TIME_NONE,
    Gst.MessageType.ERROR | Gst.MessageType.EOS)

if msg:
    t = msg.type
    if t == Gst.MessageType.ERROR:
        err, dbg = msg.parse_error()
        print("ERROR:", msg.src.get_name(), ":", err.message)
        if dbg:
            print("debugging info:", dbg)
    elif t == Gst.MessageType.EOS:
        print("End-Of-Stream reached stoped")
    else:
        print("ERROR: Unexpected message received.")

pipeline.set_state(Gst.State.NULL)
Gst.Object.unref(bus)
Gst.deinit()
