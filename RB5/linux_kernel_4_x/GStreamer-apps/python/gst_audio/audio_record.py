import sys
import gi
import logging
import os

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib, GObject

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

# Create the empty pipeline
pipeline = Gst.Pipeline.new("audio record")
pulsesrc = Gst.ElementFactory.make("pulsesrc", "pulseaudio_src");
audioconvert = Gst.ElementFactory.make("audioconvert",    "audio_convert");
capsfilter = Gst.ElementFactory.make("capsfilter",      "caps_filter");
queue = Gst.ElementFactory.make("queue",           "queue");
filesink = Gst.ElementFactory.make("filesink",        "file_sink");


if not pipeline or not pulsesrc or not audioconvert or not capsfilter or not queue or not filesink:
    print("Create element failed")
    sys.exit(1)


pulsesrc.set_property('num-buffers', 1000)
pulsesrc.set_property('buffer-time', 30000)

caps=Gst.Caps.new_empty_simple("audio/x-raw")
capsfilter.set_property('caps', caps)

filesink.set_property('location', sys.argv[1])

if file_format == ".wav":
    wavenc=Gst.ElementFactory.make("wavenc", "wav_enc");
    if not wavenc:
        print("Create element failed.\n");
        sys.exit(1)

    pipeline.add(pulsesrc)
    pipeline.add(capsfilter)
    pipeline.add(audioconvert)
    pipeline.add(wavenc)
    pipeline.add(queue)
    pipeline.add(filesink)

    if not pulsesrc.link(capsfilter):
        print("ERROR: Could not link pulsesrc to capsfilter")
        sys.exit(1)
    if not capsfilter.link(audioconvert):
        print("ERROR: Could not link capsfilter to audioconvert")
        sys.exit(1)
    if not audioconvert.link(wavenc):
        print("ERROR: Could not link audioconvert to wavenc")
        sys.exit(1)
    if not wavenc.link(queue):
        print("ERROR: Could not link wavenc to queue")
        sys.exit(1)
    if not queue.link(filesink):
        print("ERROR: Could not link queue to filesink")
        sys.exit(1)

elif file_format == ".aac":
    avenc_aac=Gst.ElementFactory.make("avenc_aac",     "aac_enc");
    aacparse=Gst.ElementFactory.make("aacparse",      "aac_parse");
    aac_caps=Gst.ElementFactory.make("capsfilter",    "aac_caps_filter");

    if not avenc_aac or not aacparse or not aac_caps:
        print("Create element failed.\n");
        sys.exit(1)

    caps=Gst.Caps.new_empty_simple('audio/mpeg')
    aac_caps.set_property('caps', caps)
    pipeline.add(pulsesrc)
    pipeline.add(capsfilter)
    pipeline.add(audioconvert)
    pipeline.add(avenc_aac)
    pipeline.add(aacparse)
    pipeline.add(aac_caps)
    pipeline.add(queue)
    pipeline.add(filesink)
    if not pulsesrc.link(capsfilter):
        print("ERROR: Could not link pulsesrc to capsfilter")
        sys.exit(1)

    if not capsfilter.link(audioconvert):
        print("ERROR: Could not link capsfilter to audioconvert")
        sys.exit(1)

    if not audioconvert.link(avenc_aac):
        print("ERROR: Could not link audioconvert to avenc_aac")
        sys.exit(1)

    if not avenc_aac.link(aacparse):
        print("ERROR: Could not link avenc_aac to aacparse")
        sys.exit(1)

    if not aacparse.link(aac_caps):
        print("ERROR: Could not link aacparse to aac_caps")
        sys.exit(1)

    if not aac_caps.link(queue):
        print("ERROR: Could not link aac_caps to queue")
        sys.exit(1)

    if not queue.link(filesink):
        print("ERROR: Could not link queue to filesink")
        sys.exit(1)

elif file_format == ".mp3":
    lamemp3enc=Gst.ElementFactory.make("lamemp3enc", "mp3_enc");

    if not lamemp3enc:
        print("Create element failed.\n");
        sys.exit(1)

    pipeline.add(pulsesrc)
    pipeline.add(capsfilter)
    pipeline.add(audioconvert)
    pipeline.add(lamemp3enc)
    pipeline.add(queue)
    pipeline.add(filesink)

    if not pulsesrc.link(capsfilter):
        print("ERROR: Could not link pulsesrc to capsfilter")
        sys.exit(1)

    if not capsfilter.link(audioconvert):
        print("ERROR: Could not link capsfilter to audioconvert")
        sys.exit(1)
    if not audioconvert.link(lamemp3enc):
        print("ERROR: Could not link audioconvert to lamemp3enc")
        sys.exit(1)
    
    if not lamemp3enc.link(queue):
        print("ERROR: Could not link lamemp3enc to queue")
        sys.exit(1)
    if not queue.link(filesink):
        print("ERROR: Could not link queue to filesink")
        sys.exit(1)

else:
    print("Format %s not supported\n", format);
    sys.exit(1)


pipeline.set_state(Gst.State.PLAYING)
print("Audio record started");
bus = pipeline.get_bus()
bus_id = bus.add_signal_watch()
msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE,Gst.MessageType.ERROR | Gst.MessageType.EOS)
if msg:
    t = msg.type
    if t == Gst.MessageType.ERROR:
        err, dbg = msg.parse_error()
        print("ERROR:", msg.src.get_name(), ":", err.message)
        if dbg:
            print("debugging info:", dbg)
    elif t == Gst.MessageType.EOS:
            print("stopped recording")
    else:
        print("ERROR: Unexpected message received.")

pipeline.set_state(Gst.State.NULL)
Gst.Object.unref(bus)
Gst.deinit()
