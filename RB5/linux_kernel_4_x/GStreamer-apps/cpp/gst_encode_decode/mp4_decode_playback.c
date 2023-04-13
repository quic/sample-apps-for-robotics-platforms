/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * mp4_decode_playback.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * mp4 file decoding to playback on weston with gstreamer on the
 * Qualcomm platform through this sample app.
 */

#include <gst/gst.h>
#include <glib.h>


static gboolean bus_callback(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    int type = GST_MESSAGE_TYPE(msg);
    
    if (type == GST_MESSAGE_EOS) {
        g_main_loop_quit (loop);
    } else if (type == GST_MESSAGE_ERROR) {
        gchar  *debug;
        GError *error;
        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        g_printerr("Error: %s\n", error->message);
        g_error_free(error);
        g_main_loop_quit(loop);
    }

    return TRUE;
}


static void on_pad_added(GstElement *bus, GstPad *pad, gpointer data)
{
    GstElement *srcpad = (GstElement *)data;
    GstPad *sinkpad;
  
    sinkpad = gst_element_get_static_pad(srcpad, "sink");
    gst_pad_link(pad, sinkpad);
    gst_object_unref(sinkpad);
}



int main(int argc, char *argv[])
{
    GMainLoop *loop;
    GstElement *pipeline, *source, *demuxer, *decoder, *videoconv, *sink;
    GstBus *bus;
    guint bus_id;

    gst_init(&argc, &argv);

    loop = g_main_loop_new(NULL, FALSE);

    if (argc != 2) {
        g_printerr("Usage: %s <mp4 filename>\n", argv[0]);
        return -1;
    }

    pipeline    = gst_pipeline_new("video-player");
    source      = gst_element_factory_make("filesrc",      "file-source");
    demuxer     = gst_element_factory_make("qtdemux",      "qt-demuxer");
    decoder     = gst_element_factory_make("avdec_h264",   "h264-decoder");
    videoconv   = gst_element_factory_make("videoconvert", "converter");
    sink        = gst_element_factory_make("waylandsink",  "video-output");

    if (!pipeline || !source || !demuxer || !decoder || !videoconv || !sink) {
        g_printerr("Create element failed.\n");
        return -1;
    }

    g_object_set(G_OBJECT(source), "location", argv[1], NULL);

    g_object_set(G_OBJECT(sink), "x", 100, "y", 100, "width", 1280, "height", 720, NULL);

    // Or we set the output fullscreen to the sink element
    // g_object_set(G_OBJECT(sink), "fullscreen", 1, NULL);

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_id = gst_bus_add_watch(bus, bus_callback, loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), source, demuxer, decoder, videoconv, sink, NULL);

    gst_element_link(source, demuxer);
    gst_element_link_many(decoder, videoconv, sink, NULL);
    g_signal_connect(demuxer, "pad-added", G_CALLBACK(on_pad_added), decoder);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("Start\n");
    g_main_loop_run(loop);
    g_print("Stop\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);

    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(bus_id);
    g_main_loop_unref(loop);

    gst_deinit();

    return 0;
}
