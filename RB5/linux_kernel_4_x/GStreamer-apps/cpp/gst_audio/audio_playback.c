/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * audio_playback.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * playback functions of gstreamer+PulseAudio on the
 * Qualcomm platform through this sample app.
 */

#include <gst/gst.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

static void usage(void)
{
    g_print("Usage:\n");
    g_print("  audio_playback /FILE/PATH/NAME.wav\n");
    g_print("  audio_playback /FILE/PATH/NAME.aac\n");
    g_print("  audio_playback /FILE/PATH/NAME.mp3\n");
    g_print("\n");

    return;
}

static int playback(const char *format, const char *content)
{
    GstElement *pipeline, *filesrc, *pulsesink;
    GstElement *wavparse;
    GstElement *aacparse, *avdec_aac;
    GstElement *mpegaudioparse, *avdec_mp3;
    GstBus *bus;
    GstMessage *msg;


    if (access(content, F_OK)) {
        g_printerr("File %s not exist\n", content); 
        return GST_STATE_NULL;
    }

    pipeline =  gst_pipeline_new("audio_playback_stream");
    filesrc =   gst_element_factory_make("filesrc",   "file_src");
    pulsesink = gst_element_factory_make("pulsesink", "pulseaudio_sink");

    if (!pipeline || !filesrc || !pulsesink) {
        g_printerr("Create element failed.n");
        return GST_STATE_NULL;
    }

    g_object_set(G_OBJECT(filesrc), "location", content, NULL);

    if (!strncmp(format, "wav", 8)) {
        wavparse = gst_element_factory_make("wavparse", "wav_parse");

        if (!wavparse) {
            g_printerr("Create element failed.\n");
            return GST_STATE_NULL;
        }

        gst_bin_add_many(GST_BIN(pipeline), filesrc, wavparse, pulsesink, NULL);
        if (gst_element_link_many(filesrc, wavparse, pulsesink, NULL) != TRUE) {
            g_printerr("Linked elements failed.\n");
            gst_object_unref(pipeline);
            return GST_STATE_NULL;
        }
    } else if (!strncmp(format, "aac", 8)) {
        aacparse    = gst_element_factory_make("aacparse",  "aac_parse");
        avdec_aac   = gst_element_factory_make("avdec_aac", "avdec_aac");

        if (!aacparse || !avdec_aac) {
            g_printerr("Create element failed.\n");
            return GST_STATE_NULL;
        }

        gst_bin_add_many(GST_BIN(pipeline), filesrc, aacparse, avdec_aac, pulsesink, NULL);
        if (gst_element_link_many(filesrc, aacparse, avdec_aac, pulsesink, NULL) != TRUE) {
            g_printerr("Linked elements failed.\n");
            gst_object_unref(pipeline);
            return GST_STATE_NULL;
        }
    } else if (!strncmp(format, "mp3", 8)) {
        mpegaudioparse = gst_element_factory_make("mpegaudioparse", "mpegaudio_parse");
        avdec_mp3 =      gst_element_factory_make("avdec_mp3",      "avdec_mp3");

        if (!mpegaudioparse || !avdec_mp3) {
            g_printerr("Create element failed.\n");
            return GST_STATE_NULL;
        }

        gst_bin_add_many(GST_BIN(pipeline), filesrc, mpegaudioparse, avdec_mp3, pulsesink, NULL);
        if (gst_element_link_many(filesrc, mpegaudioparse, avdec_mp3, pulsesink, NULL) != TRUE) {
            g_printerr("Linked elements failed.\n");
            gst_object_unref(pipeline);
            return GST_STATE_NULL;
        }
    } else {
        g_printerr("Format %s not supported\n", format);
        return GST_STATE_NULL;
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("Start\n");
    
    bus = gst_element_get_bus(pipeline);

    msg = gst_bus_poll(bus, GST_MESSAGE_EOS | GST_MESSAGE_ERROR, -1);

    int type = GST_MESSAGE_TYPE(msg);
    
    if (type == GST_MESSAGE_EOS) {
        // Do nothing
    } else if (type == GST_MESSAGE_ERROR) {
        GError *error;
        gchar *debug;

        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        g_printerr("Error: %s\n", error->message);
        g_error_free(error);
    }

    gst_message_unref(msg);

    g_print("Stop\n");
    
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(bus);

    return 0;
}

int main(int argc, char *argv[])
{
    const char* ext;

    gst_init(&argc, &argv);

    if (argc != 2) {
        usage();
        return -EINVAL;
    }

    ext = strrchr(argv[1], '.');
    if (ext && (!strncmp(ext+1, "wav", 8) || !strncmp(ext+1, "aac", 8) || !strncmp(ext+1, "mp3", 8))) {
        playback(ext+1, argv[1]);
    } else {
        usage();
        return -EINVAL;
    }

    gst_deinit();

    return 0;
}
