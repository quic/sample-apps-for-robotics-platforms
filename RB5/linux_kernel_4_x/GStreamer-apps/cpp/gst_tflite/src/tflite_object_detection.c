/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/
#include <gst/gst.h>

int main(int argc, char *argv[]) {

    GstElement *pipeline, *source, *sink, *capsfilter;
    GstElement *qtimletflite, *queue, *qtioverlay;
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;

    /* Initialize GStreamer */
    gst_init (&argc, &argv);

    /* Create the elements */
    source          =   gst_element_factory_make ("qtiqmmfsrc", "source");
    capsfilter      =   gst_element_factory_make ("capsfilter", "caps-filter");
    qtimletflite    =   gst_element_factory_make ("qtimletflite", "qti-mletflite");
    queue           =   gst_element_factory_make ("queue", "queue");
    qtioverlay      =   gst_element_factory_make ("qtioverlay", "qti-overlay");
    sink            =   gst_element_factory_make ("waylandsink", "sink");

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new ("test-pipeline");

    if (!pipeline || !source || !sink || !capsfilter || !qtimletflite || !queue || !qtioverlay) {
        g_printerr ("Not all elements could be created.\n");
        return -1;
    }

    /* Build the pipeline */
    gst_bin_add_many (GST_BIN (pipeline), source, capsfilter, qtimletflite, queue, qtioverlay, sink, NULL);
    if (gst_element_link_many (source, capsfilter, qtimletflite, queue, qtioverlay, sink, NULL) != TRUE) {
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (pipeline);
        return -1;
    }

    /* Modify the properties for caps-fitler */
    g_object_set (capsfilter, "caps",
            gst_caps_from_string ("video/x-raw,format=NV12,width=1280,height=720,framerate=30/1,camera=0"), NULL);

    /* Modify the properties for qtimle-tflite */
    g_object_set (qtimletflite, "config", "/data/misc/camera/mle_tflite.config", 
        "model", "/data/misc/camera/detect.tflite", "labels", 
        "/data/misc/camera/labelmap.txt", "postprocessing", "detection", NULL);

    /* Modify the properties for sink */
    g_object_set (sink, "fullscreen", TRUE, "async", TRUE, "enable-last-sample", FALSE, NULL);

    /* Start playing */
    ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the pipeline to the playing state.\n");
        gst_object_unref (pipeline);
        return -1;
    }

    /* Wait until error or EOS */
    bus = gst_element_get_bus (pipeline);
    msg = gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    /* Parse message */
    if (msg != NULL) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error (msg, &err, &debug_info);
            g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
            g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
            g_clear_error (&err);
            g_free (debug_info);
            break;
        case GST_MESSAGE_EOS:
            g_print ("End-Of-Stream reached.\n");
            break;
        default:
            /* We should not reach here because we only asked for ERRORs and EOS */
            g_printerr ("Unexpected message received.\n");
            break;
        }
        gst_message_unref (msg);
    }

    /* Free resources */
    gst_object_unref (bus);
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (pipeline);
    return 0;
}
