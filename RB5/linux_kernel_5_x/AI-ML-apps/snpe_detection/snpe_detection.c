/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <gst/gst.h>
#include <gnu/libc-version.h>
#define PARSER

int main(int argc, char *argv[]) {

    GstElement *pipeline;
    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;

    /* Initialize GStreamer */
    gst_init (&argc, &argv);

#ifdef PARSER
    /* Create the pipeline */
    pipeline = gst_parse_launch("qtiqmmfsrc name=camsrc ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! \
                                 queue ! tee name=split ! queue ! qtimetamux name=metamux ! queue ! qtioverlay text-font-size=30 ! \
				 queue ! waylandsink sync=false fullscreen=true split. ! queue ! qtimlvconverter ! queue ! \
				 qtimlsnpe delegate=dsp model=/data/mobilenet_v1_ssd_2017_quantized.dlc layers=\"<Postprocessor/BatchMultiClassNonMaxSuppression>\" ! \
				 queue ! qtimlvdetection threshold=50.0 results=5 module=ssd-mobilenet labels=/data/ssd-mobilenet.labels ! \
				 text/x-raw ! queue ! metamux.", NULL);

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

#else
    GstElement *source, *sink, *capsfilter;
    GstElement *qtimlsnpe, *queue, *qtioverlay;
    GstElement *tee, *qtimetamux, *qtimlvconverter;
    GstElement *qtimlvdetection;
    GstElement *infqueue;
    GstPad *tee_src_pad, *queue_src_pad;
    GstPad *tee_inf_pad, *queue_inf_pad;
    GstCaps* pad_filter;

    /* Create the elements */
    source               =   gst_element_factory_make ("qtiqmmfsrc", "source");
    capsfilter           =   gst_element_factory_make ("capsfilter", "capsfilter");
    tee                  =   gst_element_factory_make ("tee","tee");
    qtimetamux           =   gst_element_factory_make ("qtimetamux","qtimetamux");
    qtimlvconverter      =   gst_element_factory_make ("qtimlvconverter","qtimlvconverter");
    qtimlsnpe            =   gst_element_factory_make ("qtimlsnpe", "qtimlsnpe");
    qtimlvdetection      =   gst_element_factory_make ("qtimlvdetection","qtimlvdetection");
    queue                =   gst_element_factory_make ("queue", "queue");
    infqueue             =   gst_element_factory_make ("queue", "infqueue");
    qtioverlay           =   gst_element_factory_make ("qtioverlay", "qtioverlay");
    sink                 =   gst_element_factory_make ("waylandsink", "sink");

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new ("test-pipeline");

    if (!pipeline || !source || !capsfilter || !tee || !qtimetamux ||
	!qtimlvconverter || !qtimlsnpe || !qtimlvdetection ||
	!queue || !infqueue || !qtioverlay || !sink) {
        g_printerr ("Not all elements could be created.\n");
        return -1;
    }

    /* Build the pipeline */
    gst_bin_add_many (GST_BIN (pipeline), source, capsfilter, tee, qtimetamux,
		      qtimlvconverter, qtimlsnpe, qtimlvdetection,
		      queue, infqueue, qtioverlay, sink, NULL);

    /* Set filter for pad link */
    pad_filter = gst_caps_from_string("text/x-raw");

    /* Link elements */
    if (!gst_element_link_many (source, capsfilter, tee, NULL) ||
        !gst_element_link_many (queue, qtimetamux, NULL) ||
        !gst_element_link_many (infqueue, qtimlvconverter, qtimlsnpe, qtimlvdetection, NULL) ||
        !gst_element_link_filtered (qtimlvdetection, qtimetamux, pad_filter) ||
        !gst_element_link_many (qtimetamux, qtioverlay, sink, NULL)){
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (pipeline);
        return -1;
    }

    /* Modify elements' properties */
    /* For more info & modifiable properties, run gst-inspect-1.0 <element> to check */
    g_object_set (capsfilter, "caps",
            gst_caps_from_string ("video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1"), NULL);

    /* Create a GValue for layers propertie of qtimlsnpe */
    gchar *str = "Postprocessor/BatchMultiClassNonMaxSuppression";
    GValue layer = G_VALUE_INIT;
    g_value_init (&layer, G_TYPE_STRING);
    g_value_set_string (&layer, str);

    GValue *snpe_array;
    gst_value_array_init(&snpe_array,1);
    gst_value_array_append_value(&snpe_array,&layer);

    /* Modify elements' properties */
    g_object_set (qtimlsnpe, "model", "/data/mobilenet_v1_ssd_2017_quantized.dlc", "delegate", 1, NULL);
    g_object_set_property (G_OBJECT(qtimlsnpe), "layers", &snpe_array);
    g_object_set (qtimlvdetection, "threshold", 50.0, "results", 10, "module", 2,
		  "labels", "/data/ssd-mobilenet.labels", NULL);
    g_object_set (sink, "fullscreen", TRUE, "sync", FALSE, NULL);

    /* Link Tee which has On Request src pads */
    tee_src_pad = gst_element_get_request_pad (tee, "src_%u");
    g_print ("Obtained request pad %s for camera source branch.\n", gst_pad_get_name (tee_src_pad));
    queue_src_pad = gst_element_get_static_pad (queue, "sink");

    tee_inf_pad = gst_element_get_request_pad (tee, "src_%u");
    g_print ("Obtained request pad %s for camera source branch.\n", gst_pad_get_name (tee_inf_pad));
    queue_inf_pad = gst_element_get_static_pad (infqueue, "sink");

    if (gst_pad_link (tee_src_pad, queue_src_pad) != GST_PAD_LINK_OK ||
        gst_pad_link (tee_inf_pad, queue_inf_pad) != GST_PAD_LINK_OK){
	g_printerr ("Tee could not be linked 0.\n");
	gst_object_unref (pipeline);
	return -1;
    }
    gst_object_unref (queue_src_pad);
    gst_object_unref (queue_inf_pad);

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

    /* Release the request pads from the Tee, and unref them */
    gst_element_release_request_pad (tee, tee_src_pad);
    gst_element_release_request_pad (tee, tee_inf_pad);
    gst_object_unref (tee_src_pad);
    gst_object_unref (tee_inf_pad);

    g_value_unset(&layer);
    g_value_unset(&snpe_array);
#endif
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
