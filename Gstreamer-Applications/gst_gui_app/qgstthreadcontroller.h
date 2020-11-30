/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef QGSTTHREADCONTROLLER_H
#define QGSTTHREADCONTROLLER_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QSize>
#include <QRect>

#include "qgstpipeline.h"


/**
 * @brief The QGstThreadController class
 *
 * Accept parameters from main Window and
 * start to run QGstPipeline instance in
 * a thread.
 */
class QGstThreadController : public QObject
{
    Q_OBJECT

    QThread workerThread;
public:
    explicit QGstThreadController(QObject *parent = nullptr);
    ~QGstThreadController();

    void start(QString op, QString device,
               QSize cameraSize, qint16 framerate, QRect display,
               bool fullscreen=false, QString filePath="");
    void start(QString op, QString device,
               QSize cameraSize, qint16 framerate, QRect display,
               bool flipH, bool flipV, quint8 rotate,
               bool fullscreen=false, QString filePath="");
    void start(QString op, QString device,
               QSize cameraSize, qint16 framerate, QRect display, QRect crop,
               bool fullscreen=false, QString filePath="");
    void start(QString op, QString device,
               QSize cameraSize, qint16 framerate, QRect display,
               bool flipH, bool flipV, quint8 rotate, QRect crop,
               bool fullscreen=false, QString filePath="");
    void start(QString op, QString device, QRect display, bool fullscreen, QString filePath);
    void stop();
    bool isStart() { return bStart; }

public slots:
    void pipelineStarted(const QString op);
    void pipelineFinished();
    void errorHandler(const QString error);


signals:
    // to QGstPipeline
    void startPipeline(const QString op, const QStringList param);
    void stopPipeline();
    // to main window
    void started(const QString op);
    void finished();
    void pipelineError(const QString error);

private:
    QGstPipeline *gstPipeline;
    bool bStart;

    void startThread(const QString op, const QStringList param);

};

#endif // QGSTTHREADCONTROLLER_H
