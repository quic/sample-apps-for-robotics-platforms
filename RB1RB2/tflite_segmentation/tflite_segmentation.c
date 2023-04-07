/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <gst/gst.h>
//#define PARSER

int main(int argc, char *argv[]) {

    GstElement *pipeline;
    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;

    /* Initialize GStreamer */
    gst_init (&argc, &argv);

#ifdef PARSER
    /* Create the pipeline */
    pipeline = gst_parse_launch("qtivcomposer name=mixer ! queue ! video/x-raw ! \
		    waylandsink fullscreen=true enable-last-sample=false sync=false qtiqmmfsrc camera=0 ! \
		    video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! tee name=t ! queue ! mixer. t. ! \
		    queue ! qtimlvconverter ! queue ! qtimltflite delegate=nnapi-dsp model=/data/dv3_argmax_int32.tflite ! \
		    queue ! qtimlvsegmentation module=deeplab-argmax labels=/data/dv3-argmax.labels ! \
		    video/x-raw,width=256,height=144 ! queue ! mixer.", NULL);

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
    GstElement *qtimltflite, *queue;
    GstElement *tee, *qtivcomposer, *qtimlvconverter;
    GstElement *qtimlvsegmentation;
    GstElement *infqueue;
    GstPad *tee_src_pad, *queue_src_pad;
    GstPad *tee_inf_pad, *queue_inf_pad;
    GstCaps *pad_filter1, *pad_filter2;

    /* Create the elements */
    source               =   gst_element_factory_make ("qtiqmmfsrc", "source");
    capsfilter           =   gst_element_factory_make ("capsfilter", "capsfilter");
    tee                  =   gst_element_factory_make ("tee","tee");
    qtivcomposer         =   gst_element_factory_make ("qtivcomposer","qtivcomposer");
    qtimlvconverter      =   gst_element_factory_make ("qtimlvconverter","qtimlvconverter");
    qtimltflite          =   gst_element_factory_make ("qtimltflite", "qtimltflite");
    qtimlvsegmentation   =   gst_element_factory_make ("qtimlvsegmentation","qtimlvsegmentation");
    queue                =   gst_element_factory_make ("queue", "queue");
    infqueue             =   gst_element_factory_make ("queue", "infqueue");
    sink                 =   gst_element_factory_make ("waylandsink", "sink");

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new ("test-pipeline");

    if (!pipeline || !source || !capsfilter || !tee || !qtivcomposer ||
	!qtimlvconverter || !qtimltflite || !qtimlvsegmentation ||
	!queue || !infqueue || !sink) {
        g_printerr ("Not all elements could be created.\n");
        return -1;
    }

    /* Build the pipeline */
    gst_bin_add_many (GST_BIN (pipeline), source, capsfilter, tee, qtivcomposer,
		      qtimlvconverter, qtimltflite, qtimlvsegmentation,
		      queue, infqueue, sink, NULL);

    /* Set filter for pad link */
    pad_filter1 = gst_caps_from_string("video/x-raw");
    pad_filter2 = gst_caps_from_string("video/x-raw,width=256,height=144");

    /* Link elements */
    if (!gst_element_link_many (source, capsfilter, tee, NULL) ||
        !gst_element_link_many (queue, qtivcomposer, NULL) ||
        !gst_element_link_many (infqueue, qtimlvconverter, qtimltflite, qtimlvsegmentation, NULL) ||
        !gst_element_link_filtered (qtimlvsegmentation, qtivcomposer, pad_filter2) ||
        !gst_element_link_filtered (qtivcomposer, sink, pad_filter1)){
        g_printerr ("Elements could not be linked.\n");
        gst_object_unref (pipeline);
        return -1;
    }

    /* Modify elements' properties*/
    /* For more info & modifiable properties, run gst-inspect-1.0 <element> to check */
    g_object_set (capsfilter, "caps",
            gst_caps_from_string ("video/x-raw,format=NV12,width=1280,height=720,framerate=30/1,camera=0"), NULL);
    g_object_set (qtimltflite, "model", "/data/dv3_argmax_int32.tflite", "delegate", 1, NULL);
    g_object_set (qtimlvsegmentation, "module",1, "labels", "/data/dv3-argmax.labels", NULL);
    g_object_set (sink, "fullscreen", TRUE, "sync", FALSE, "enable-last-sample", FALSE, NULL);

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
