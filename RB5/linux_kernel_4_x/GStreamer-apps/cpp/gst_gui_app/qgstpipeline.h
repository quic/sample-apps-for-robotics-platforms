/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef QGSTPIPELINE_H
#define QGSTPIPELINE_H

#include <QObject>
#include <gst/gst.h>
#include <glib.h>


/**
 * Global constants to define operations
 */
const QString DISPLAY = "DISPLAY";
const QString RECORD = "RECORD";
const QString PLAYBACK = "PLAYBACK";


/**
 * @brief The QGstPipeline class
 *
 * Use gstreamer API to start pipeline.
 */
class QGstPipeline : public QObject
{
    Q_OBJECT

public:
    explicit QGstPipeline(QObject *parent = nullptr);
    ~QGstPipeline();

    GMainLoop * mainLoop() { return loop; }
    void emitError(const QString error) { emit reportError(error); }

public slots:
    void start(const QString op, const QStringList param);
    void stop();


signals:
    void started(const QString op);
    void reportError(const QString);
    void finished();

private:
    static GstBusSyncReply
    bus_callback (GstBus     *bus,
                  GstMessage *msg,
                  gpointer    data);
    static void
    on_pad_added (GstElement *element,
                  GstPad     *pad,
                  gpointer    data);

    GMainLoop *loop;
    GstElement *pipeline;
    GstElement *qmmfsource, *transform, *waylandsink;
    GstElement *parse, *muxer, *queue, *filesink;
    GstPad *video_pad0, *disp_pad, *video_pad1, *encode_pad;
    GstElement *filesource, *demuxer, *decoder, *convert;

    void startIspCamera(QStringList param);
    void startRecord(QStringList param);
    void startPlayback(QStringList param);


};

#endif // QGSTPIPELINE_H
