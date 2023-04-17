import sys
import gi
import logging
import os
gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib, GObject
from numpy import ndarray


#Playback sample code 
def gst_sample_playback(content):
    # Create the empty pipeline 
    pipeline = Gst.Pipeline.new("audio playback")
    filesrc = Gst.ElementFactory.make("filesrc", "file_src")
    wavparse = Gst.ElementFactory.make("wavparse", "wav_parse");
    alsasink = Gst.ElementFactory.make("alsasink", "alsa_sink");

    if not pipeline or not filesrc or not wavparse or not alsasink:
        print("Create pipeline/filesrc/wavparse/alsasink element failed.")
        sys.exit(1)

    #set properties to the elements
    filesrc.set_property('location',content)
    alsasink.set_property('device',"hw:0,0")

    #Build pipeline
    #Add elements to the pipeline and link them
    pipeline.add(filesrc)
    pipeline.add(wavparse)
    pipeline.add(alsasink)

    if not filesrc.link(wavparse):
        print("ERROR: Could not link filesrc to wavparse")
        sys.exit(1)
    if not wavparse.link(alsasink):
        print("ERROR: Could not link wavparse to alsasink")
        sys.exit(1)

    pipeline.set_state(Gst.State.PLAYING)
    print("Start");
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
            print("Stop end-Of-Stream reached")
        else:
            print("ERROR: Unexpected message received.")

    pipeline.set_state(Gst.State.NULL)
    Gst.Object.unref(bus)
    Gst.deinit()
    sys.exit(1)


#Capture sample code 
def gst_sample_capture(content):
    #create the empty pipeline
    alsasrc = Gst.ElementFactory.make("alsasrc", "alsa_src");
    capsfilter = Gst.ElementFactory.make("capsfilter", "caps_filter");
    wavenc = Gst.ElementFactory.make("wavenc", "wav_enc");
    filesink = Gst.ElementFactory.make("filesink", "file_sink");
    pipeline = Gst.Pipeline.new("audio_capture_stream");

    if not pipeline or not alsasrc or not capsfilter or not wavenc or not filesink:
        print("Create pipeline/alsasrc/capsfilter/wavenc/filesink element failed.")
        sys.exit(1)

    #set properties to the elements
    alsasrc.set_property("device","hw:0,0")
    alsasrc.set_property("num-buffers",1000)
    alsasrc.set_property("buffer-time",10000) 
    caps=Gst.Caps.new_empty_simple("audio/x-raw")
    capsfilter.set_property("caps",caps)
    filesink.set_property("location",content)
    
    #Build the pipeline. Add elements to the pipeline and link them
    pipeline.add(alsasrc)
    pipeline.add(capsfilter)
    pipeline.add(wavenc)
    pipeline.add(filesink)

    if not alsasrc.link(capsfilter):
        print("ERROR: Could not link alsasrc to capsfilter")
        sys.exit(1)
    if not capsfilter.link(wavenc):
        print("ERROR: Could not link capsfilter to wavenc")
        sys.exit(1)
    if not wavenc.link(filesink):
        print("ERROR: Could not link wavenc to filesink")
        sys.exit(1)

    pipeline.set_state(Gst.State.PLAYING)
    print("Start");

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
            print("Stop end-Of-Stream reached")
        else:
            print("ERROR: Unexpected message received.")

    pipeline.set_state(Gst.State.NULL)
    Gst.Object.unref(bus)
    Gst.deinit()
    sys.exit(1)


def main():
    Gst.init(sys.argv) 
    if len(sys.argv) < 2:
        print("Missing <output-location> parameter")
        sys.exit(1)
    else:
        if (sys.argv[1]=="playback"):
            gst_sample_playback(sys.argv[2]);
        elif(sys.argv[1]=="capture"):
            gst_sample_capture(sys.argv[2]);
        else:
            print("Unexpected operation %s", sys.argv[1]);


if __name__ == "__main__":
    main()
