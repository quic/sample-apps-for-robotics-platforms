/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * tcp_server.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * tcp streaming server of ISP camera with gstreamer on the
 * Qualcomm platform through this sample app.
 * 
 * Client should use VLC player to play the url:
 *    tcp://<ip>:<port>/
 */

#include <gst/gst.h>
#include <glib.h>


static void usage(const char* argv0)
{
    g_print("Usage: %s [camera id: 0|1|2|3] IP port\n", argv0);
    g_print(" IP: [x.x.x.x] : IP address of the board. It maybe through eht0 or wlan0.\n");
    g_print("Example:\n");
    g_print("    %s 0 192.168.3.5 5000\n", argv0);
}

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
    GstElement *pipeline, *source, *framefilter, *parse, *muxer, *queue, *sink;
    GstBus *bus;
    guint bus_id;
    gint8 camera_id;
    gint port;

    gst_init(&argc, &argv);

    loop = g_main_loop_new(NULL, FALSE);

    if (argc != 4) {
        usage(argv[0]);
        return -1;
    }

    camera_id = atoi(argv[1]);
    if (camera_id < 0 || camera_id > 3) {
        g_printerr("camera id incorrect.\n");
        usage(argv[0]);
        return -1;
    }

    port = atoi(argv[3]);
    if (port < 0) {
        g_printerr("port incorrect.\n");
        usage(argv[0]);
        return -1;
    }

    pipeline        = gst_pipeline_new("video-streaming");
    source          = gst_element_factory_make("qtiqmmfsrc",   "qmmf-source");
    framefilter     = gst_element_factory_make("capsfilter",   "frame-filter");
    parse           = gst_element_factory_make("h264parse",    "h264-parse");
    muxer           = gst_element_factory_make("mpegtsmux",    "mpegts-muxer");
    queue           = gst_element_factory_make("queue",        "queue");
    sink            = gst_element_factory_make("tcpserversink","tcp-server");


    if (!pipeline || !source || !framefilter || !parse || !queue || !sink) {
        g_printerr("Create element failed.\n");
        return -1;
    }

    g_object_set (G_OBJECT(source), "camera", camera_id, NULL);

    if (camera_id == 0) {
        g_object_set(G_OBJECT(framefilter), "caps", 
                            gst_caps_from_string("video/x-h264,framerate=30/1,width=1920,height=1080"), NULL);
    } else {
        g_object_set(G_OBJECT(framefilter), "caps", 
                            gst_caps_from_string("video/x-h264,framerate=30/1,width=1280,height=720"), NULL);
    }

    g_object_set (G_OBJECT(parse), "config-interval", 1, NULL);
    g_object_set (G_OBJECT(sink), "host", argv[2], "port", port, NULL);


    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_id = gst_bus_add_watch(bus, bus_callback, loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), source, framefilter, parse, muxer, queue, sink, NULL);
    gst_element_link_many(source, framefilter, parse, muxer, queue, sink, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("Start\n");
    g_print("  vlc tcp://%s:%d/\n", argv[2], port);
    g_main_loop_run(loop);
    g_print("Stop\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);

    gst_object_unref(GST_OBJECT (pipeline));
    g_source_remove(bus_id);
    g_main_loop_unref(loop);

    gst_deinit();

    return 0;
}
