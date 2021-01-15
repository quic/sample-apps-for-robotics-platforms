/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <errno.h>
#include <gst/gst.h>
#include <string.h>

/* patterns */
char inputfile[512] = "\0";
const char *encode_outputfile = "/data/OpenMAX-Video/test_video/output_encode.mp4";
const char *decode_outputfile = "/data/OpenMAX-Video/test_video/output_decode.yuv";

int error_handle(GstElement *pipeline);
int gst_encode_yuv();
int gst_decode_file();
int check_input();
static void qtdemux_pad_added_cb (GstElement *qtdemux, GstPad *pad, GstElement *queue);
void Usage(void);

void Usage (void) {
    g_print("Usage: \n");
    g_print("./gst-omx_codec_h264 <OPERATION> <FILE>\n");
    g_print("OPERATION: encode OR decode\n");
    g_print("FILE: the absolute file path of inputfile + inputfile name\n");
    g_print("   encode:\t encode <FILE> to h264 stream, and ouput to");
    g_print("   decode:\t decode <FILE> to yuv raw data and output to");
    g_print("\n");

    return;
}

int gst_encode_yuv () {
    int raw_width, raw_height, raw_format, raw_framerate;
    GstElement *pipeline, *source, *videoparse, *omxh264enc, *h264parse, *sink, *capsfilter;
    GstCaps *caps;
    GstStateChangeReturn ret;

    /* Take users' configuration */
    printf("Enter raw clip width, height, format, and framerate of images in raw stream in order: \n");
    scanf("%d %d %d %d", &raw_width, &raw_height, &raw_format, &raw_framerate);
    /* Create the element */
    source = gst_element_factory_make("filesrc", "InputFile");
    videoparse = gst_element_factory_make("videoparse", "VideoParse");
    omxh264enc = gst_element_factory_make("omxh264enc", "OmxH264Enc");
    h264parse = gst_element_factory_make("h264parse", "H264Parse");
    sink = gst_element_factory_make("filesink", "OutputFile");
    capsfilter = gst_element_factory_make("capsfilter", "CapsFilter");

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new("encode-pipeline");

    if (!pipeline || !source || !videoparse || !omxh264enc || !h264parse || !sink) {
        g_printerr("Not all elements could be created.\n");
        return GST_STATE_NULL;
    }

    /* Modify element properties */
    g_object_set(G_OBJECT(source), "location", inputfile, NULL);
    //g_object_set(G_OBJECT(videoparse), "width", 352, "height", 288, "format", 23,
    //        "framerate", 25, 1, NULL);
    g_object_set(G_OBJECT(videoparse), "width", raw_width, "height", raw_height,
            "format", raw_format, "framerate", raw_framerate, 1, NULL);
    g_object_set(G_OBJECT(omxh264enc), "target-bitrate", 60000000, NULL);
    g_object_set(G_OBJECT(sink), "location", encode_outputfile, NULL);

    caps = gst_caps_new_simple("video/x-h264",
            "stream-format", G_TYPE_STRING, "byte-stream",
            "profile", G_TYPE_STRING, "main",
            "level", G_TYPE_STRING, "3",
            NULL);
    g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
    gst_caps_unref(caps);

    /* Build the pipeline */
    gst_bin_add_many(GST_BIN(pipeline), source, videoparse, omxh264enc, capsfilter,
            h264parse, sink, NULL);
    if (gst_element_link_many(source, videoparse, omxh264enc, capsfilter, h264parse, sink, NULL) != TRUE ) {
        g_printerr("Elements could not be linked\n");
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    /* Start playing */
    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    error_handle(pipeline);

    /* Free pipeline */
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}

int gst_decode_file () {
    GstElement *pipeline, *source, *qtdemux, *queue, *h264parse, *omxh264dec, *sink;
    GstStateChangeReturn ret;

    /* Create the element */
    source = gst_element_factory_make("filesrc", "InputFile");
    qtdemux = gst_element_factory_make("qtdemux", "QtDemux");
    queue = gst_element_factory_make("queue", "Queue");
    h264parse = gst_element_factory_make("h264parse", "H264Parse");
    omxh264dec = gst_element_factory_make("omxh264dec", "OmxH264Enc");
    sink = gst_element_factory_make("filesink", "OutputFile");

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new("decode-pipeline");

    if (!pipeline || !source || !qtdemux || !queue || !omxh264dec || !h264parse || !sink) {
        g_printerr("Not all elements could be created.\n");
        return GST_STATE_NULL;
    }

    /* Modify element properties */
    g_object_set(G_OBJECT(source), "location", inputfile, NULL);
    g_object_set(G_OBJECT(qtdemux), "name", "demux", NULL);
    g_object_set(G_OBJECT(sink), "location", decode_outputfile, NULL);

    /* Build the pipeline */
    gst_bin_add_many(GST_BIN(pipeline), source, qtdemux, queue, h264parse,
            omxh264dec, sink, NULL);

   if (gst_element_link(source, qtdemux) != TRUE ) {
        g_printerr("source and qtdemux could not be linked\n");
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    if (gst_element_link_many(queue, h264parse, omxh264dec, sink, NULL) != TRUE ) {
        g_printerr("queue, h264parse, omxh264dec, and sink could not be linked\n");
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    g_signal_connect (qtdemux, "pad-added", (GCallback) qtdemux_pad_added_cb, queue);

    /* Start playing */
    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return GST_STATE_NULL;
    }

    error_handle(pipeline);

    /* Free pipeline */
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}

static void qtdemux_pad_added_cb (GstElement *qtdemux, GstPad *pad, GstElement *queue)
{
      gst_element_link_pads(qtdemux, GST_PAD_NAME (pad), queue, NULL);
}

int error_handle (GstElement *pipeline) {
    GstBus *bus;
    GstMessage *msg;

    /* Wait until error or EOS */
    bus = gst_element_get_bus (pipeline);
    msg = gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
                                      GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    /* Message handling */
    if (msg != NULL) {
          GError *err;
          gchar *debug_info;

          switch (GST_MESSAGE_TYPE (msg)) {
              case GST_MESSAGE_ERROR:
                  gst_message_parse_error (msg, &err, &debug_info);
                  g_printerr ("Error received: %s\n", err->message);
                  if (debug_info) {
                      g_printerr ("Debugging information: %s\n", debug_info);
                  }
                  g_clear_error (&err);
                  g_free (debug_info);
                  break;
              case GST_MESSAGE_EOS:
                  g_print ("End-Of-Stream reached.\n");
                  break;
              default:
                  g_printerr ("Unexpected message received.\n");
                  break;
          }
          gst_message_unref (msg);
    }

    /* Free resources */
    gst_object_unref (bus);
    return 0;
}

int check_input ()
{
    FILE *file = fopen(inputfile, "r");
    if (file != NULL) {
        fclose(file);
        return 0;
    } else {
        return -1;
    }
}

int main (int argc, char *argv[])
{
    /* Initializing GStreamer */
    gst_init (&argc, &argv);

    int res = 0;

    if (argc != 3) {
          g_printerr ("Unexpected argc\n");
          Usage();
          return -EINVAL;
    }

    if (!strcmp(argv[1], "encode")) {
        strncpy(inputfile, argv[2], strlen(argv[2]));
        res = check_input();
        if (res == -1) {
            g_printerr ("Invalid inputfile\n");
            Usage();
            return -EINVAL;
        } else {
            gst_encode_yuv();
        }
    } else if (!strcmp(argv[1], "decode")) {
        strncpy(inputfile, argv[2], strlen(argv[2]));
        res = check_input();
        if (res == -1) {
            g_printerr ("Invalid inputfile\n");
            Usage();
            return -EINVAL;
        } else {
            gst_decode_file();
        }
    } else {
        g_printerr ("Unexpected operation %s\n", argv[1]);
        Usage();
        return -EINVAL;
    }

    return 0;
}
