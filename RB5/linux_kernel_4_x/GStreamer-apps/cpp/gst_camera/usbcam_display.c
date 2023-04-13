/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * usbcam_display.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * USB camera display on weston with gstreamer on the
 * Qualcomm platform through this sample app.
 *
 * This sample should be run in weston display.
 */

#include <gst/gst.h>
#include <glib.h>


static gboolean bus_callback(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    int type = GST_MESSAGE_TYPE(msg);

    if (type == GST_MESSAGE_ERROR) {
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


int main(int argc, char *argv[])
{
    GMainLoop *loop;

    GstElement *pipeline, *source, *videoconv, *videorate, *framefilter, *sink;
    GstBus *bus;
    guint bus_id;

    gst_init (&argc, &argv);

    loop = g_main_loop_new(NULL, FALSE);

    if (argc != 2) {
        g_printerr("Usage: %s </dev/videoX>\n", argv[0]);
        return -1;
    }

    pipeline    = gst_pipeline_new("video-display");
    source      = gst_element_factory_make("v4l2src",      "v4l2-source");
    videoconv   = gst_element_factory_make("videoconvert", "video-convert");
    videorate   = gst_element_factory_make("videorate",    "video-rate");
    framefilter = gst_element_factory_make("capsfilter",   "frame-filter");
    sink        = gst_element_factory_make("waylandsink",  "display");

    // videoconvert: Convert video frames between a great variety of video formats.
    // we add videoconvert to convert the format from different usb camera input.


    if (!pipeline || !source || !videoconv || !videorate || !framefilter || !sink) {
        g_printerr ("Create element failed.\n");
        return -1;
    }

    g_object_set(G_OBJECT(source), "device", argv[1], NULL);

    g_object_set(G_OBJECT(framefilter), "caps", 
                        gst_caps_from_string("video/x-raw,framerate=15/1,width=1280,height=720"), NULL);

    g_object_set (G_OBJECT(sink), "x", 0, "y", 100, "width", 640, "height", 360, NULL);

    bus = gst_pipeline_get_bus (GST_PIPELINE(pipeline));
    bus_id = gst_bus_add_watch(bus, bus_callback, loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), source, videoconv, videorate, framefilter, sink, NULL);
    gst_element_link_many(source, videoconv, videorate, framefilter, sink, NULL);

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
