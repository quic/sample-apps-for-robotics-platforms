/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * transform_display.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * transform funtions of ISP camera with gstreamer on the
 * Qualcomm platform through this sample app.
 * 
 * This sample should be run in weston display.
 */

#include <gst/gst.h>
#include <glib.h>


static void usage(const char *argv0)
{
    g_print("Usage: %s [camera id: 0|1|2|3] [Operation]\n", argv0);
    g_print(" Operation:\n");
    g_print("    flip v|h (v:vertical, h:horizontal)\n");
    g_print("    rotate 1|2|3  (1:90CW, 2:90CCW, 3:180)\n");
    g_print("    crop x y width height\n");
    g_print(" Example:\n");
    g_print("    %s 0 flip v\n", argv0);
    g_print("    %s 0 flip h\n", argv0);
    g_print("    %s 0 rotate 2\n", argv0);
    g_print("    %s 0 crop 10 10 200 100\n", argv0);
}

static gboolean bus_callback(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    int type = GST_MESSAGE_TYPE(msg);
    
    if (type == GST_MESSAGE_ERROR) {
        gchar *debug;
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
    GstElement *pipeline, *source, *framefilter, *transform, *sink, *filter;
    GstBus *bus;
    guint bus_id;
    gint8 camera_id;
    gint8 rotate_type;

    gst_init (&argc, &argv);

    loop = g_main_loop_new(NULL, FALSE);

    if (argc < 3) {
        usage(argv[0]);
        return -1;
    }

    camera_id = atoi(argv[1]);
    if (camera_id < 0 || camera_id > 3) {
        g_printerr("camera id incorrect.\n");
        usage(argv[0]);
        return -1;
    }

    if (!strncmp(argv[2], "flip", 8)) {
        if (argc == 4) {
            if (strncmp(argv[3], "v", 8) && strncmp(argv[3], "h", 8)) {
                g_printerr("flip should specify v or h.\n");
                usage(argv[0]);
                return -1;
            }
            
        } else {
            g_printerr("flip should specify v or h.\n");
            usage(argv[0]);
            return -1;
        }
    } else if (!strncmp(argv[2], "rotate", 8)) {
        if (argc == 4) {
            rotate_type = atoi(argv[3]);
            if (rotate_type < 1 || rotate_type > 3) {
                g_printerr("rotate type incorrect. (1|2|3)\n");
                usage(argv[0]);
                return -1;
            }
        } else {
            g_printerr("rotate type (1|2|3) needs to be specified.\n");
            usage(argv[0]);
            return -1;
        }
    } else if (!strncmp(argv[2], "crop", 8)) {
        if (argc != 7) {
            g_printerr("crop position and size (x y w h) need to be specified.\n");
            usage(argv[0]);
            return -1;
        }
    } else {
        g_printerr("Unknown operation: %s.\n", argv[2]);
        usage(argv[0]);
        return -1;
    }


    pipeline        = gst_pipeline_new("video-display");
    source          = gst_element_factory_make("qtiqmmfsrc",   "qmmf-source");
    framefilter     = gst_element_factory_make("capsfilter",   "frame-filter");
    filter          = gst_element_factory_make("capsfilter",   "filter");
    transform       = gst_element_factory_make("qtivtransform","transform");
    sink            = gst_element_factory_make("waylandsink",  "display");


    if (!pipeline || !source || !framefilter || !transform || !filter || !sink) {
        g_printerr("Element create failed.\n");
        return -1;
    }

    g_object_set(G_OBJECT(source), "camera", camera_id, NULL);

    if (camera_id == 0) {
        g_object_set(G_OBJECT(framefilter), "caps", 
                            gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1920,height=1080"), NULL);
    } else {
        g_object_set(G_OBJECT(framefilter), "caps", 
                            gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1280,height=720"), NULL);
    }

    if (camera_id == 0) {
        g_object_set(G_OBJECT(filter), "caps", 
                            gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1920,height=1080"), NULL);
    } else {
        g_object_set(G_OBJECT(filter), "caps", 
                            gst_caps_from_string("video/x-raw,format=NV12,framerate=30/1,width=1280,height=720"), NULL);
    }

    if (!strncmp(argv[2], "flip", 8)) {
        if (!strncmp(argv[3], "v", 8)) {
            g_object_set (G_OBJECT(transform), "flip-vertical", 1, NULL);
        } else if (!strncmp(argv[3], "h", 8)) {
            g_object_set (G_OBJECT(transform), "flip-horizontal", 1, NULL);
        }
    } else if (!strncmp(argv[2], "rotate", 8)) {
        g_object_set (G_OBJECT(transform), "rotate", atoi(argv[3]), NULL);
    } else if (!strncmp(argv[2], "crop", 8)) {
        GArray *array = g_array_new(FALSE, TRUE, sizeof(GValue));;
        GValue cropX = G_VALUE_INIT;
        GValue cropY = G_VALUE_INIT;
        GValue cropWidth = G_VALUE_INIT;
        GValue cropHeight = G_VALUE_INIT;
        g_value_init (&cropX, G_TYPE_INT);
        g_value_init (&cropY, G_TYPE_INT);
        g_value_init (&cropWidth, G_TYPE_INT);
        g_value_init (&cropHeight,G_TYPE_INT);
        g_value_set_int (&cropX, atoi(argv[3]));
        g_value_set_int (&cropY, atoi(argv[4]));
        g_value_set_int (&cropWidth, atoi(argv[5]));
        g_value_set_int (&cropHeight,atoi(argv[6]));
        g_array_append_val(array, cropX);
        g_array_append_val(array, cropY);
        g_array_append_val(array, cropWidth);
        g_array_append_val(array, cropHeight);
        g_object_set (G_OBJECT(transform), "crop", array, NULL);
        g_array_free(array, TRUE);
    }

    g_object_set(G_OBJECT(sink), "x", 0, "y", 0, "width", 640, "height", 480, NULL);

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_id = gst_bus_add_watch(bus, bus_callback, loop);
    gst_object_unref(bus);


    gst_bin_add_many(GST_BIN(pipeline), source, framefilter, transform, filter, sink, NULL);
    gst_element_link_many(source, framefilter, transform, filter, sink, NULL);

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
