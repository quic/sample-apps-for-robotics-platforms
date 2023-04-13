/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "qgstpipeline.h"
#include <QDebug>
#include <QThread>

/**
 * @brief QGstPipeline::QGstPipeline
 * @param parent
 *
 * Constructor
 *
 * Initialize gstreamer.
 */
QGstPipeline::QGstPipeline(QObject *parent) : QObject(parent)
{
    gst_init(NULL, NULL);
}

/**
 * @brief QGstPipeline::~QGstPipeline
 *
 * Destructor
 *
 * Deinitialize gstreamer.
 */
QGstPipeline::~QGstPipeline()
{
    qDebug() << "gst_deinit()";
    gst_deinit();
}

/**
 * @brief QGstPipeline::bus_call
 * @param msg
 * @param data
 * @return
 *
 * Private Static
 *
 * Callback of GstBus.
 *
 * According to the message, apply an action to the pipeline.
 */
GstBusSyncReply QGstPipeline::bus_callback(GstBus *, GstMessage *msg, gpointer data)
{
    QGstPipeline *gstPipeline = (QGstPipeline*)data;
    GMainLoop *loop = gstPipeline->mainLoop();

    int type = GST_MESSAGE_TYPE(msg);

    if (type == GST_MESSAGE_ERROR) {
        gchar  *debug;
        GError *error;
        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        gstPipeline->emitError(error->message);
        g_error_free(error);
        g_main_loop_quit(loop);
    } else if (type == GST_MESSAGE_APPLICATION) {
        const GstStructure *st;

        st = gst_message_get_structure (msg);

        if (gst_structure_has_name(st, "UserInterrupt")) {
            g_main_loop_quit (loop);
        }
    } else if (type == GST_MESSAGE_EOS) {
            g_main_loop_quit (loop);
    }

    return GST_BUS_DROP;
}

/**
 * @brief QGstPipeline::start
 * @param op
 * @param param
 *
 * Public
 *
 * Start the pipeline according to the operation.
 */
void QGstPipeline::start(QString op, QStringList param)
{
    if (op == DISPLAY) {
        startIspCamera(param);
    } else if (op == RECORD) {
        startRecord(param);
    } else if (op == PLAYBACK) {
        startPlayback(param);
    }
}

/**
 * @brief QGstPipeline::stop
 *
 * Public
 *
 * Post a message to pipeline to stop it.
 */
void QGstPipeline::stop()
{
    qDebug() << "QGstPipeline stop slot";

    gst_element_post_message (GST_ELEMENT(pipeline),
        gst_message_new_application (GST_OBJECT(pipeline),
        gst_structure_new ("UserInterrupt", "message", G_TYPE_STRING, "Interrupted", NULL)));
}

/**
 * @brief QGstPipeline::startIspCamera
 * @param param
 *
 * Private
 *
 * Use gstreamer API to display camera on wayland.
 */
void QGstPipeline::startIspCamera(QStringList param)
{
    GstBus *bus;
    GstCaps *disp_caps;
    qint8 camera_id = -1;
    qint16 width = -1;
    qint16 height = -1;
    qint16 framerate = -1;
    bool fullscreen = false;
    qint16 start_x = -1;
    qint16 start_y = -1;
    qint16 disp_w = -1;
    qint16 disp_h = -1;
    gchar *caps_str;
    bool flip_v = false;
    bool flip_h = false;
    qint8 rotate = -1;
    bool is_crop_x = false;
    bool is_crop_y = false;
    bool is_crop_w = false;
    bool is_crop_h = false;
    qint16 crop_x = -1;
    qint16 crop_y = -1;
    qint16 crop_w = -1;
    qint16 crop_h = -1;
    GstStateChangeReturn ret;

    for (int i = 0; i < param.length(); i++) {
        QStringList p = param[i].split("=");
        if (p[0] == "camera") {
            camera_id = p[1].toInt();
        } else if (p[0] == "width") {
            width = p[1].toInt();
        } else if (p[0] == "height") {
            height = p[1].toInt();
        } else if (p[0] == "framerate") {
            framerate = p[1].toInt();
        } else if (p[0] == "fullscreen") {
            fullscreen = p[1].toInt();
        } else if (p[0] == "start_x") {
            start_x = p[1].toInt();
        } else if (p[0] == "start_y") {
            start_y = p[1].toInt();
        } else if (p[0] == "disp_w") {
            disp_w = p[1].toInt();
        } else if (p[0] == "disp_h") {
            disp_h = p[1].toInt();
        } else if (p[0] == "flip_h") {
            flip_h = p[1].toInt();
        } else if (p[0] == "flip_v") {
            flip_v = p[1].toInt();
        } else if (p[0] == "rotate") {
            rotate = p[1].toInt();
        } else if (p[0] == "crop_x") {
            crop_x = p[1].toInt();
            is_crop_x = true;
        } else if (p[0] == "crop_y") {
            crop_y = p[1].toInt();
            is_crop_y = true;
        } else if (p[0] == "crop_w") {
            crop_w = p[1].toInt();
            is_crop_w = true;
        } else if (p[0] == "crop_h") {
            crop_h = p[1].toInt();
            is_crop_h = true;
        }
    }

    if (camera_id < 0 || width <= 0 || height <= 0 || framerate <= 0 ||
        (!fullscreen && (start_x < 0  || start_y < 0 || disp_w <= 0 || disp_h <= 0))) {
        emit reportError("Wrong parameter input");
        return;
    }

    loop = g_main_loop_new (NULL, FALSE);

    pipeline    = gst_pipeline_new("video-display");
    qmmfsource  = gst_element_factory_make("qtiqmmfsrc",   "qmmf-source");
    transform   = gst_element_factory_make("qtivtransform","transform");
    waylandsink = gst_element_factory_make("waylandsink",  "display");

    if (!pipeline || !qmmfsource || !transform || !waylandsink) {
        emit reportError("Create element failed.");
        goto exit;
    }

    video_pad0 = gst_element_get_request_pad(qmmfsource, "video_%u");
    disp_pad = gst_element_get_static_pad(transform, "sink");

    if (!video_pad0 || !disp_pad) {
        emit reportError("Create pad failed.");
        goto exit;
    }

    g_object_set (G_OBJECT(qmmfsource), "camera", camera_id, NULL);

    caps_str = g_strdup_printf("video/x-raw(memory:GBM),format=NV12,framerate=%d/1,width=%d,height=%d", framerate, width, height);
    disp_caps = gst_caps_from_string(caps_str);

    if (!gst_pad_set_caps(video_pad0, disp_caps)) {
        emit reportError("gst_pad_set_caps failed");
        goto exit;
    }

    if (flip_h) {
        g_object_set(G_OBJECT(transform), "flip-horizontal", 1, NULL);
    }
    if (flip_v) {
        g_object_set(G_OBJECT(transform), "flip-vertical", 1, NULL);
    }
    if (rotate != -1) {
        g_object_set(G_OBJECT(transform), "rotate", rotate, NULL);
    }
    if (is_crop_x && is_crop_y && is_crop_w && is_crop_h) {
        g_object_set(G_OBJECT(transform), "crop-x", crop_x, "crop-y", crop_y,
                                          "crop-width", crop_w, "crop-height", crop_h, NULL);
    }

    if (!fullscreen) {
        g_object_set(G_OBJECT(waylandsink), "x", start_x, "y", start_y, "width", disp_w, "height", disp_h, NULL);
    } else {
        g_object_set(G_OBJECT(waylandsink), "fullscreen", fullscreen, NULL);
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));
    gst_bus_set_sync_handler(bus, bus_callback, this, nullptr);
    gst_object_unref(bus);

    gst_bin_add_many (GST_BIN(pipeline), qmmfsource, transform, waylandsink, NULL);

    if (gst_pad_link(video_pad0, disp_pad) != GST_PAD_LINK_OK) {
        emit reportError("Pad0 could not be linked.");
        goto exit;
    }

    gst_element_link_many(transform, waylandsink, NULL);

    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    ret = gst_element_get_state(pipeline, NULL, NULL, 0);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        emit reportError("Gstreamer changing state failed. Some settings may be incorrect.");
        goto exit;
    }

    emit started(DISPLAY);

    g_main_loop_run(loop);

    gst_element_send_event(pipeline, gst_event_new_eos());
    gst_element_set_state(pipeline, GST_STATE_NULL);

exit:
    gst_object_unref(GST_OBJECT(pipeline));

    gst_object_unref(GST_OBJECT(waylandsink));
    gst_object_unref(GST_OBJECT(transform));
    gst_object_unref(GST_OBJECT(qmmfsource));

    g_main_loop_unref(loop);

    emit finished();
}

/**
 * @brief QGstPipeline::startRecord
 * @param param
 *
 * Private
 *
 * Use gstreamer API to record camera to mp4.
 * Also display camera on wayland.
 */
void QGstPipeline::startRecord(QStringList param)
{
    GstBus *bus;
    GstCaps *encode_caps, *disp_caps;
    qint8 camera_id = -1;
    qint16 width = -1;
    qint16 height = -1;
    qint16 framerate = -1;
    bool fullscreen = false;
    qint16 start_x = -1;
    qint16 start_y = -1;
    qint16 disp_w = -1;
    qint16 disp_h = -1;
    gchar *caps_str;
    bool flip_v = false;
    bool flip_h = false;
    qint8 rotate = -1;
    bool is_crop_x = false;
    bool is_crop_y = false;
    bool is_crop_w = false;
    bool is_crop_h = false;
    qint16 crop_x = -1;
    qint16 crop_y = -1;
    qint16 crop_w = -1;
    qint16 crop_h = -1;
    QString file_path;
    GstStateChangeReturn ret;

    for (int i = 0; i < param.length(); i++) {
        QStringList p = param[i].split("=");
        if (p[0] == "camera") {
            camera_id = p[1].toInt();
        } else if (p[0] == "width") {
            width = p[1].toInt();
        } else if (p[0] == "height") {
            height = p[1].toInt();
        } else if (p[0] == "framerate") {
            framerate = p[1].toInt();
        } else if (p[0] == "fullscreen") {
            fullscreen = p[1].toInt();
        } else if (p[0] == "start_x") {
            start_x = p[1].toInt();
        } else if (p[0] == "start_y") {
            start_y = p[1].toInt();
        } else if (p[0] == "disp_w") {
            disp_w = p[1].toInt();
        } else if (p[0] == "disp_h") {
            disp_h = p[1].toInt();
        } else if (p[0] == "flip_h") {
            flip_h = p[1].toInt();
        } else if (p[0] == "flip_v") {
            flip_v = p[1].toInt();
        } else if (p[0] == "rotate") {
            rotate = p[1].toInt();
        } else if (p[0] == "crop_x") {
            crop_x = p[1].toInt();
            is_crop_x = true;
        } else if (p[0] == "crop_y") {
            crop_y = p[1].toInt();
            is_crop_y = true;
        } else if (p[0] == "crop_w") {
            crop_w = p[1].toInt();
            is_crop_w = true;
        } else if (p[0] == "crop_h") {
            crop_h = p[1].toInt();
            is_crop_h = true;
        } else if (p[0] == "file_path") {
            file_path = p[1];
        }
    }
    if (camera_id < 0 || width <= 0 || height <= 0 || framerate <= 0 ||
        (!fullscreen && (start_x < 0  || start_y < 0 || disp_w <= 0 || disp_h <= 0)) ||
        file_path.isEmpty()) {
        emit reportError("Wrong parameter input");
        return;
    }

    loop = g_main_loop_new (NULL, FALSE);

    pipeline    = gst_pipeline_new("video-display");
    qmmfsource  = gst_element_factory_make("qtiqmmfsrc",   "qmmf-source");
    transform   = gst_element_factory_make("qtivtransform","transform");
    waylandsink = gst_element_factory_make("waylandsink",  "display");
    parse       = gst_element_factory_make("h264parse",    "h264-parse");
    muxer       = gst_element_factory_make("mp4mux",       "mp4-muxer");
    queue       = gst_element_factory_make("queue",        "queue");
    filesink    = gst_element_factory_make("filesink",     "file-output");

    if (!pipeline || !qmmfsource || !transform || !waylandsink ||
        !parse || !muxer || !queue || !filesink) {
        emit reportError("Create element failed.");
        goto exit;
    }

    video_pad0 = gst_element_get_request_pad(qmmfsource, "video_%u");
    video_pad1 = gst_element_get_request_pad(qmmfsource, "video_%u");
    encode_pad = gst_element_get_static_pad(parse, "sink");
    disp_pad   = gst_element_get_static_pad(transform, "sink");

    if (!video_pad0 || !disp_pad || !video_pad1 || !encode_pad) {
        emit reportError("Create pad failed.");
        goto exit;
    }

    g_object_set(G_OBJECT(qmmfsource), "camera", camera_id, NULL);

    if (flip_h) {
        g_object_set(G_OBJECT(transform), "flip-horizontal", 1, NULL);
    }
    if (flip_v) {
        g_object_set(G_OBJECT(transform), "flip-vertical", 1, NULL);
    }
    if (rotate != -1) {
        g_object_set(G_OBJECT(transform), "rotate", rotate, NULL);
    }
    if (is_crop_x && is_crop_y && is_crop_w && is_crop_h) {
        g_object_set(G_OBJECT(transform), "crop-x", crop_x, "crop-y", crop_y,
                                          "crop-width", crop_w, "crop-height", crop_h, NULL);
    }

    g_object_set(G_OBJECT(filesink), "location", file_path.toLocal8Bit().data(), NULL);

    if (!fullscreen) {
        g_object_set(G_OBJECT(waylandsink), "x", start_x, "y", start_y, "width", disp_w, "height", disp_h, NULL);
    } else {
        g_object_set(G_OBJECT(waylandsink), "fullscreen", fullscreen, NULL);
    }

    caps_str = g_strdup_printf("video/x-h264,format=NV12,framerate=%d/1,width=%d,height=%d", framerate, width, height);
    encode_caps = gst_caps_from_string(caps_str);
    caps_str = g_strdup_printf("video/x-raw(memory:GBM),format=NV12,framerate=%d/1,width=%d,height=%d", framerate, width, height);
    disp_caps = gst_caps_from_string(caps_str);

    if (!gst_pad_set_caps(video_pad0, encode_caps) ||
          !gst_pad_set_caps(video_pad1, disp_caps)) {
        emit reportError("gst_pad_set_caps failed");
        goto exit;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));
    gst_bus_set_sync_handler(bus, bus_callback, this, nullptr);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN (pipeline), qmmfsource, transform, waylandsink, parse, muxer, queue, filesink, NULL);

    // source pad0 -> parse -> muxer -> queue -> file
    // source pad1 -> transform -> display

    if (gst_pad_link(video_pad0, encode_pad) != GST_PAD_LINK_OK) {
        emit reportError("Pad0 could not be linked.");
        goto exit;
    }

    if (gst_pad_link(video_pad1, disp_pad) != GST_PAD_LINK_OK) {
        emit reportError("Pad1 could not be linked\n");
        goto exit;
    }

    gst_element_link_many(parse, muxer, queue, filesink, NULL);
    gst_element_link_many(transform, waylandsink, NULL);


    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    ret = gst_element_get_state(pipeline, NULL, NULL, 0);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        emit reportError("Gstreamer changing state failed. Some settings may be incorrect.");
        goto exit;
    }

    emit started(RECORD);

    g_main_loop_run(loop);

    gst_element_send_event(pipeline, gst_event_new_eos ());
    gst_element_set_state(pipeline, GST_STATE_NULL);

exit:
    gst_object_unref(GST_OBJECT (pipeline));

    gst_object_unref(GST_OBJECT (waylandsink));
    gst_object_unref(GST_OBJECT (transform));
    gst_object_unref(GST_OBJECT (filesink));
    gst_object_unref(GST_OBJECT (queue));
    gst_object_unref(GST_OBJECT (muxer));
    gst_object_unref(GST_OBJECT (parse));
    gst_object_unref(GST_OBJECT (qmmfsource));

    g_main_loop_unref(loop);

    emit finished();
}

/**
 * @brief QGstPipeline::on_pad_added
 * @param pad
 * @param data
 *
 * Private Static
 *
 * Callback of pad added.
 *
 * Pads are linked when pad added.
 */
void QGstPipeline::on_pad_added(GstElement *, GstPad *pad, gpointer data)
{
    GstElement *srcpad = (GstElement *)data;
    GstPad *sinkpad;

    sinkpad = gst_element_get_static_pad(srcpad, "sink");
    gst_pad_link(pad, sinkpad);
    gst_object_unref(sinkpad);
}

/**
 * @brief QGstPipeline::startPlayback
 * @param param
 *
 * Private
 *
 * Use gstreamer API to decode mp4 to wayland.
 */
void QGstPipeline::startPlayback(QStringList param)
{
    GstBus *bus;
    qint8 camera_id = -1;
    bool fullscreen = false;
    qint16 start_x = -1;
    qint16 start_y = -1;
    qint16 disp_w = -1;
    qint16 disp_h = -1;
    QString file_path;
    GstStateChangeReturn ret;

    for (int i = 0; i < param.length(); i++) {
        QStringList p = param[i].split("=");
        if (p[0] == "camera") {
            camera_id = p[1].toInt();
        } else if (p[0] == "fullscreen") {
            fullscreen = p[1].toInt();
        } else if (p[0] == "start_x") {
            start_x = p[1].toInt();
        } else if (p[0] == "start_y") {
            start_y = p[1].toInt();
        } else if (p[0] == "disp_w") {
            disp_w = p[1].toInt();
        } else if (p[0] == "disp_h") {
            disp_h = p[1].toInt();
        } else if (p[0] == "file_path") {
            file_path = p[1];
        }
    }
    if (camera_id < 0 || (!fullscreen && (start_x < 0  || start_y < 0 || disp_w <= 0 || disp_h <= 0)) ||
        file_path.isEmpty() ) {
        emit reportError("Wrong parameter input");
        return;
    }

    loop = g_main_loop_new (NULL, FALSE);

    pipeline    = gst_pipeline_new("video-player");
    filesource  = gst_element_factory_make("filesrc",      "file-source");
    demuxer     = gst_element_factory_make("qtdemux",      "qt-demuxer");
    decoder     = gst_element_factory_make("avdec_h264",   "h264-decoder");
    convert     = gst_element_factory_make("videoconvert", "converter");
    waylandsink = gst_element_factory_make("waylandsink",  "video-output");

    if (!pipeline || !filesource || !demuxer || !decoder || !convert || !waylandsink) {
        emit reportError("Create element failed.");
        goto exit;
    }

    g_object_set(G_OBJECT(filesource), "location", file_path.toLocal8Bit().data(), NULL);

    if (!fullscreen) {
        g_object_set(G_OBJECT(waylandsink), "x", start_x, "y", start_y, "width", disp_w, "height", disp_h, NULL);
    } else {
        g_object_set(G_OBJECT(waylandsink), "fullscreen", fullscreen, NULL);
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_set_sync_handler(bus, bus_callback, this, nullptr);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), filesource, demuxer, decoder, convert, waylandsink, NULL);

    gst_element_link(filesource, demuxer);
    gst_element_link_many(decoder, convert, waylandsink, NULL);
    g_signal_connect(demuxer, "pad-added", G_CALLBACK(on_pad_added), decoder);


    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    ret = gst_element_get_state(pipeline, NULL, NULL, 0);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        emit reportError("Gstreamer changing state failed. Some settings may be incorrect.");
        goto exit;
    }

    emit started(PLAYBACK);

    g_main_loop_run(loop);

    gst_element_send_event(pipeline, gst_event_new_eos());
    gst_element_set_state(pipeline, GST_STATE_NULL);

 exit:
    gst_object_unref(GST_OBJECT(pipeline));

    gst_object_unref(GST_OBJECT(waylandsink));
    gst_object_unref(GST_OBJECT(convert));
    gst_object_unref(GST_OBJECT(decoder));
    gst_object_unref(GST_OBJECT(demuxer));
    gst_object_unref(GST_OBJECT(filesource));

    g_main_loop_unref(loop);

    emit finished();
}
