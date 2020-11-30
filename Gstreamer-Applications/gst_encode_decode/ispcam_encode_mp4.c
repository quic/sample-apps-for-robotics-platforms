/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * ispcam_encode_mp4.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * mp4 file encoding from ISP camera with gstreamer on the
 * Qualcomm platform through this sample app.
 */

#include <gst/gst.h>
#include <glib.h>
#include <glib-unix.h>

#define NO_ERROR  0
#define ERROR     1
#define INTERRUPT 2

static guint signal_interrupt_id;

gboolean interrupt_handler(gpointer data)
{
    GstElement *pipeline = (GstElement *)data;

    gst_element_post_message (GST_ELEMENT(pipeline),
        gst_message_new_application(GST_OBJECT(pipeline),
        gst_structure_new("UserInterrupt", "message", G_TYPE_STRING, "Interrupted", NULL)));

    signal_interrupt_id = 0;
    return G_SOURCE_REMOVE;
}

static int run(GstElement *pipeline)
{
    GstBus *bus;
    GstMessage *msg;
    int res = NO_ERROR;

    bus = gst_element_get_bus(GST_ELEMENT(pipeline));

    signal_interrupt_id = g_unix_signal_add(SIGINT, (GSourceFunc)interrupt_handler, pipeline);

    while (TRUE) {
        msg = gst_bus_poll(bus, GST_MESSAGE_ANY, -1);

        if (msg == NULL) {
            break;
        }

        int type = GST_MESSAGE_TYPE(msg);

        if (type == GST_MESSAGE_EOS) {
            break;
        } else if (type == GST_MESSAGE_APPLICATION) {
            const GstStructure *st = gst_message_get_structure(msg);

            if (gst_structure_has_name(st, "UserInterrupt")) {
                res = INTERRUPT;
                break;
            }
        } else if (type == GST_MESSAGE_ERROR) {
            gchar  *debug;
            GError *error;

            gst_message_parse_error(msg, &error, &debug);
            g_free(debug);
            g_printerr("Error: %s\n", error->message);
            g_error_free(error);
            res = ERROR;
            break;
        }
        if (msg) {
            gst_message_unref (msg);
        }
    }
   
    if (msg) {
        gst_message_unref (msg);
    }
    gst_object_unref (bus);

    if (signal_interrupt_id > 0) {
        g_source_remove (signal_interrupt_id);
    }

    return res;
}


int main(int argc, char *argv[])
{
    GstElement *pipeline, *source, *framefilter, *parse, *muxer, *queue, *sink;
    int res;
    gint8 camera_id;

    gst_init(&argc, &argv);

    if (argc != 3) {
        g_printerr("Usage: %s [camera id: 0|1|2|3] <mp4 filename>\n", argv[0]);
        return -1;
    }

    camera_id = atoi(argv[1]);
    if (camera_id < 0 || camera_id > 3) {
        g_printerr("Usage: %s [camera id: 0|1|2|3] <mp4 filename>\n", argv[0]);
        return -1;
    }

    /* Create gstreamer elements */
    pipeline    = gst_pipeline_new("video-encoder");
    source      = gst_element_factory_make("qtiqmmfsrc",   "qmmfsource");
    framefilter = gst_element_factory_make("capsfilter",   "frame-filter");
    parse       = gst_element_factory_make("h264parse",    "h264-parse");
    muxer       = gst_element_factory_make("mp4mux",       "mp4-muxer");
    queue       = gst_element_factory_make("queue",        "queue");
    sink        = gst_element_factory_make("filesink",     "file-output");

    if (!pipeline || !source || !framefilter || !parse || !muxer || !queue || !sink) {
        g_printerr("Create element failed.\n");
        return -1;
    }

    g_object_set(G_OBJECT(source), "camera", camera_id, NULL);

    if (camera_id == 0) {
        g_object_set(G_OBJECT(framefilter), "caps", 
                            gst_caps_from_string("video/x-h264,format=NV12,framerate=30/1,width=1920,height=1080"), NULL);
    } else {
        g_object_set(G_OBJECT(framefilter), "caps", 
                            gst_caps_from_string("video/x-h264,format=NV12,framerate=30/1,width=1280,height=720"), NULL);
    }

    g_object_set(G_OBJECT(sink), "location", argv[2], NULL);

    gst_bin_add_many(GST_BIN(pipeline), source, framefilter, parse, muxer, queue, sink, NULL);
    gst_element_link_many(source, framefilter, parse, muxer, queue, sink, NULL);

    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    g_print("Start\n");
    g_print("\nPress Ctrl-C to stop.\n\n");

    res = run(pipeline);

    // check keyboard interrupt
    if (res == INTERRUPT) {
        gst_element_send_event(pipeline, gst_event_new_eos());

        // Wait EOS
        while (TRUE) {
            res = run(pipeline);
            if (res == NO_ERROR) {
                g_print("EOS - stop\n");
                break;
            } else if (res == INTERRUPT) {
                g_print("Interrupt when waiting for EOS - stop\n");
                break;
            } else if (res == ERROR) {
                g_print("Error when waiting for EOS\n");
                break;
            }
        }
    }

    g_print("Stop\n");

    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline));

    gst_deinit ();

    return 0;
}
