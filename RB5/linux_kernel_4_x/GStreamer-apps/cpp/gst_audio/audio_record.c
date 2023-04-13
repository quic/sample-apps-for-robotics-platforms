/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

/*
 * audio_record.c:
 *
 * A sample app based on gstreamer
 * The purpose is helping users to learn how to implement the
 * recording functions of gstreamer+PulseAudio on the
 * Qualcomm platform through this sample app.
 */

#include <gst/gst.h>
#include <glib-unix.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#define NO_ERROR  0
#define ERROR     1
#define INTERRUPT 2

static guint signal_interrupt_id;

static void usage(void)
{
    g_print("Usage:\n");
    g_print("  audio_record /FILE/PATH/FILENAME.[wav|aac|mp3]\n");
    g_print("\n");
    g_print("Example:\n");
    g_print("  audio_record /FILE/PATH/NAME.wav\n");
    g_print("  audio_record /FILE/PATH/NAME.aac\n");
    g_print("  audio_record /FILE/PATH/NAME.mp3\n");
    g_print("\n");

    return;
}


gboolean interrupt_handler(gpointer data)
{
    GstElement *pipeline = (GstElement *)data;

    gst_element_post_message(GST_ELEMENT(pipeline),
        gst_message_new_application(GST_OBJECT(pipeline),
        gst_structure_new ("UserInterrupt", "message", G_TYPE_STRING, "Interrupted", NULL)));

    signal_interrupt_id = 0;
    return G_SOURCE_REMOVE;
}


int run(GstElement *pipeline)
{
    GstBus *bus;
    GstMessage *msg = NULL;
    int res = NO_ERROR;
    
    bus = gst_element_get_bus(GST_ELEMENT(pipeline));

    signal_interrupt_id = g_unix_signal_add (SIGINT, (GSourceFunc)interrupt_handler, pipeline);

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
            gst_message_unref(msg);
        }

    }
    if (msg) {
        gst_message_unref(msg);
    }
    gst_object_unref(bus);

    if (signal_interrupt_id > 0) {
          g_source_remove (signal_interrupt_id);
    }

    return res;
}


/** Record sample code 
 *  format: wav | aac | mp3 
 *  content: file name
*/
static int record(const char *format, const char *content)
{
    GstElement *pipeline, *pulsesrc, *audioconvert, *capsfilter, *queue, *filesink;
    GstElement *wavenc;
    GstElement *avenc_aac, *aacparse, *aac_caps;
    GstElement *lamemp3enc;
    GstCaps *caps;
    int res;

    pipeline =      gst_pipeline_new("audio_record_stream");
    pulsesrc =      gst_element_factory_make("pulsesrc",        "pulseaudio_src");
    audioconvert =  gst_element_factory_make("audioconvert",    "audio_convert");
    capsfilter =    gst_element_factory_make("capsfilter",      "caps_filter");
    queue =         gst_element_factory_make("queue",           "queue");
    filesink =      gst_element_factory_make("filesink",        "file_sink");
    
    if (!pipeline || !pulsesrc || !audioconvert || !capsfilter || !filesink) {
        g_printerr("Create element failed.\n");
        return GST_STATE_NULL;
    }

    g_object_set(G_OBJECT(pulsesrc), "num-buffers", 1000, "buffer-time", 30000, NULL);

    caps = gst_caps_new_simple("audio/x-raw",
        "format",G_TYPE_STRING,"S16LE",
        "width", G_TYPE_INT, 16,
        "channels", G_TYPE_INT, 1,
        "rate", G_TYPE_INT, 48000,
        NULL);
    g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
    gst_caps_unref(caps);

    g_object_set(G_OBJECT(filesink), "location", content, NULL);

    if (!strncmp(format, "wav", 8)) {
        wavenc = gst_element_factory_make("wavenc", "wav_enc");

        if (!wavenc) {
            g_printerr("Create element failed.\n");
            return GST_STATE_NULL;
        }

        gst_bin_add_many(GST_BIN(pipeline), pulsesrc, capsfilter, audioconvert, wavenc, queue, filesink, NULL);
        if (gst_element_link_many(pulsesrc, capsfilter, audioconvert, wavenc, queue, filesink, NULL) != TRUE) {
            g_printerr("Linked elements failed.\n");
            gst_object_unref(pipeline);
            return GST_STATE_NULL;
        }
    } else if (!strncmp(format, "aac", 8)) {
        avenc_aac   = gst_element_factory_make("avenc_aac",     "aac_enc");
        aacparse    = gst_element_factory_make("aacparse",      "aac_parse");
        aac_caps    = gst_element_factory_make("capsfilter",    "aac_caps_filter");

        if (!avenc_aac || !aacparse || !aac_caps) {
            g_printerr("Create element failed.\n");
            return GST_STATE_NULL;
        }

        caps = gst_caps_new_simple("audio/mpeg",
            "mpegversion", G_TYPE_INT, 4,
            "stream-format",G_TYPE_STRING,"adts",
            NULL);
        g_object_set(G_OBJECT(aac_caps), "caps", caps, NULL);
        gst_caps_unref(caps);

        gst_bin_add_many(GST_BIN(pipeline), pulsesrc, capsfilter, audioconvert, avenc_aac, aacparse, aac_caps, queue, filesink, NULL);
        if (gst_element_link_many(pulsesrc, capsfilter, audioconvert, avenc_aac, aacparse, aac_caps, queue, filesink, NULL) != TRUE) {
            g_printerr("Linked elements failed.\n");
            gst_object_unref(pipeline);
            return GST_STATE_NULL;
        }
    } else if (!strncmp(format, "mp3", 8)) {
        lamemp3enc = gst_element_factory_make("lamemp3enc", "mp3_enc");

        if (!lamemp3enc) {
            g_printerr("Create element failed.\n");
            return GST_STATE_NULL;
        }

        gst_bin_add_many(GST_BIN(pipeline), pulsesrc, capsfilter, audioconvert, lamemp3enc, queue, filesink, NULL);
        if (gst_element_link_many(pulsesrc, capsfilter, audioconvert, lamemp3enc, queue, filesink, NULL) != TRUE) {
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

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}

int main (int argc, char *argv[])
{
    const char* ext;

    gst_init(&argc, &argv);

    if (argc != 2) {
        usage();
        return -EINVAL;
    }

    /* Check input function */
    ext = strrchr(argv[1], '.');
    if (ext && (!strncmp(ext+1, "wav", 8) || !strncmp(ext+1, "aac", 8) || !strncmp(ext+1, "mp3", 8))) {
        record(ext+1, argv[1]);
    } else {
        usage();
        return -EINVAL;
    }

    gst_deinit();

    return 0;
}
