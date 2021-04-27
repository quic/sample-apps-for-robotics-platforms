/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <gst/gst.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

char card[64] = "hw:0";

static void usage(void)
{
    g_print("Usage:\n");
    g_print("  gst_audio_sample OPERATION /FILE/PATH/NAME.WAV\n");
    g_print("\n");
    g_print("Operation:\n");
    g_print("  playback\t\tPlayback /FILE/PATH/NAME.WAV via speaker\n");
    g_print("  Capture\t\tCapture and output at /FILE/PATH/NAME.WAV via dmic0\n");
    g_print("\n");

    return;
}

/* Playback sample code */
static int gst_sample_playback(const char* content)
{
    GstElement *pipeline, *filesrc, *wavparse, *alsasink;
    GstBus *bus;
    GstMessage *msg;
    GstCaps *caps;
    FILE *file;

    g_print("%s: Start\n", __func__);

    if (access(content, F_OK)) {
        g_printerr("%s: Failed, %s not exist\n", __func__, content); 
        return GST_STATE_NULL;
    }

    /* create pipeline */
    filesrc = gst_element_factory_make("filesrc", "file_src");
    wavparse = gst_element_factory_make("wavparse", "wav_parse");
    alsasink = gst_element_factory_make("alsasink", "alsa_sink");
    pipeline = gst_pipeline_new("audio_playback_stream");

    if (!pipeline || !filesrc || !wavparse || !alsasink) {
        g_printerr("%s: Not all elements could be created\n", __func__);
        return GST_STATE_NULL;
    }

    /* The playback path can using arcv at here */
    g_object_set(G_OBJECT(filesrc), "location", content, NULL);
    g_object_set(G_OBJECT(alsasink), "device", "hw:0,0", NULL);

    gst_bin_add_many(GST_BIN(pipeline), filesrc, wavparse, alsasink, NULL);
    if (gst_element_link_many(filesrc, wavparse, alsasink, NULL) != TRUE) {
        g_printerr("Elements could not be linked\n");
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    /* Start playing */
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    /* Wait until error or EOS */
    bus = gst_element_get_bus(pipeline);

    /* wait until we either get an EOS or an ERROR message. Note that in a real
     * program you would probably not use gst_bus_poll(), but rather set up an
     * async signal watch on the bus and run a main loop and connect to the
     * bus's signals to catch certain messages or all messages */
    msg = gst_bus_poll(bus, GST_MESSAGE_EOS | GST_MESSAGE_ERROR, -1);

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS: {
            g_print("EOS\n");
            break;
        }
        case GST_MESSAGE_ERROR: {
            GError *err = NULL; /* error to show to users                 */
            gchar *dbg = NULL;  /* additional debug string for developers */

            gst_message_parse_error(msg, &err, &dbg);
            if (err) {
                g_printerr("ERROR: %s\n", err->message);
                g_error_free(err);
            }
            if (dbg) {
                g_printerr("[Debug details: %s]\n", dbg);
                g_free(dbg);
            }
        }
        default: {
            g_printerr("Unexpected message of type %d", GST_MESSAGE_TYPE (msg));
            break;
        }
    }

    gst_message_unref(msg);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(bus);

    g_print("%s: Stop\n", __func__);

    return 0;
}

/* Capture sample code */
static int gst_sample_capture(const char* content)
{
    GstElement *pipeline, *alsasrc, *audioconvert, *wavenc, *filesink, *capsfilter;
    GstBus *bus;
    GstMessage *msg;
    GstCaps *caps;

    g_print("%s: Start\n", __func__);

    /* create pipeline */
    alsasrc = gst_element_factory_make("alsasrc", "alsa_src");
    capsfilter = gst_element_factory_make("capsfilter", "caps_filter");
    wavenc = gst_element_factory_make("wavenc", "wav_enc");
    filesink = gst_element_factory_make("filesink", "file_sink");
    pipeline = gst_pipeline_new("audio_capture_stream");

    if (!pipeline || !alsasrc || !capsfilter || !wavenc || !filesink) {
        g_printerr("%s: Not all elements could be created\n", __func__);
        return GST_STATE_NULL;
    }

    g_object_set(G_OBJECT(alsasrc), "device", "hw:0,0", "num-buffers", 1000, "buffer-time", 30000, NULL);

    caps = gst_caps_new_simple("audio/x-raw",
        "format",G_TYPE_STRING,"S16LE",
        "width", G_TYPE_INT, 16,
        "channels", G_TYPE_INT, 1,
        "rate", G_TYPE_INT, 48000,
        NULL);
    g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
    gst_caps_unref(caps);

    g_object_set(G_OBJECT(filesink), "location", content, NULL);

    gst_bin_add_many(GST_BIN(pipeline), alsasrc, capsfilter, wavenc, filesink, NULL);
    if (gst_element_link_many(alsasrc, capsfilter, wavenc, filesink, NULL) != TRUE) {
        g_printerr("%s: Elements could not be linked\n", __func__);
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    /* Start playing */
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    /* Wait until error or EOS */
    bus = gst_element_get_bus(pipeline);

    /* wait until we either get an EOS or an ERROR message. Note that in a real
     * program you would probably not use gst_bus_poll(), but rather set up an
     * async signal watch on the bus and run a main loop and connect to the
     * bus's signals to catch certain messages or all messages */
    msg = gst_bus_poll(bus, GST_MESSAGE_EOS | GST_MESSAGE_ERROR, -1);

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS: {
            g_print("EOS\n");
            break;
        }
        case GST_MESSAGE_ERROR: {
            GError *err = NULL; /* error to show to users                 */
            gchar *dbg = NULL;  /* additional debug string for developers */

            gst_message_parse_error(msg, &err, &dbg);
            if (err) {
                g_printerr("ERROR: %s\n", err->message);
                g_error_free(err);
            }
            if (dbg) {
                g_printerr("[Debug details: %s]\n", dbg);
                g_free(dbg);
            }
        }
        default: {
            g_printerr("Unexpected message of type %d", GST_MESSAGE_TYPE (msg));
            break;
        }
    }

    gst_message_unref(msg);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(bus);

    g_print("Get captured file from: %s\n", content);
    g_print("%s: Stop\n", __func__);

    return 0;
}

int main (int argc, char *argv[])
{
    /* Initialize GStreamer */
    gst_init (&argc, &argv);

    if (argc != 3) {
        g_printerr("Unexpected argc\n");
        usage();
        return -EINVAL;
    }

    /* Check input function */
    if (!strncmp(argv[1], "playback", 8)) {
        gst_sample_playback(argv[2]);
    } else if (!strncmp(argv[1], "capture", 6)) {
        gst_sample_capture(argv[2]);
    } else {
        g_printerr("Unexpected operation %s\n", argv[1]);
        usage();
        return -EINVAL;
    }

    return 0;
}
