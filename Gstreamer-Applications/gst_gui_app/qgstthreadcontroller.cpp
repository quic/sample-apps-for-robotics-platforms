/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "qgstthreadcontroller.h"

/**
 * @brief QGstThreadController::QGstThreadController
 * @param parent
 *
 * Constructor
 *
 * Create a QGstPipeline instance.
 * Put QGstPipeline to a thread.
 */
QGstThreadController::QGstThreadController(QObject *parent) : QObject(parent)
{
    bStart = false;

    // Move QGstPipeline to workerThread
    gstPipeline = new QGstPipeline;
    gstPipeline->moveToThread(&workerThread);

    connect(this, SIGNAL(startPipeline(const QString, const QStringList)),
            gstPipeline, SLOT(start(const QString, const QStringList)));
    connect(this, SIGNAL(stopPipeline()), gstPipeline, SLOT(stop()), Qt::DirectConnection);

    connect(gstPipeline, SIGNAL(started(const QString)), this, SLOT(pipelineStarted(const QString)));
    connect(gstPipeline, SIGNAL(reportError(const QString)), this, SLOT(errorHandler(const QString)));
    connect(gstPipeline, SIGNAL(finished()), this, SLOT(pipelineFinished()));
}

/**
 * @brief QGstThreadController::~QGstThreadController
 *
 * Destructor
 *
 * Quit and wait thread in destructor.
 */
QGstThreadController::~QGstThreadController()
{
    emit stopPipeline();
    workerThread.quit();
    workerThread.wait();
    if (gstPipeline) {
        delete gstPipeline;
    }
}

/**
 * @brief QGstThreadController::start
 * @param op
 * @param device
 * @param cameraSize
 * @param framerate
 * @param displayRect
 * @param fullscreen
 * @param filePath
 *
 * Public
 *
 * Arrange parameters to internal data.
 */
void QGstThreadController::start(QString op, QString device,
                                 QSize cameraSize, qint16 framerate,
                                 QRect displayRect, bool fullscreen,
                                 QString filePath)
{
    QStringList param;
    param << "camera="+device;
    param << "width="+QString::number(cameraSize.width());
    param << "height="+QString::number(cameraSize.height());
    param << "framerate="+QString::number(framerate);
    if (!fullscreen) {
        param << "start_x="+QString::number(displayRect.x());
        param << "start_y="+QString::number(displayRect.y());
        param << "disp_w="+QString::number(displayRect.width());
        param << "disp_h="+QString::number(displayRect.height());
    } else {
        param << "fullscreen="+QString::number(fullscreen);
    }
    if (!filePath.isEmpty()) {
        param << "file_path="+filePath;
    }

    startThread(op, param);
}

/**
 * @brief QGstThreadController::start
 * @param op
 * @param device
 * @param cameraSize
 * @param framerate
 * @param displayRect
 * @param flipH
 * @param flipV
 * @param rotate
 * @param fullscreen
 * @param filePath
 *
 * Public
 *
 * Arrange parameters to internal data.
 */
void QGstThreadController::start(QString op, QString device,
                                 QSize cameraSize, qint16 framerate, QRect displayRect,
                                 bool flipH, bool flipV, quint8 rotate,
                                 bool fullscreen, QString filePath)
{
    QStringList param;
    param << "camera="+device;
    param << "width="+QString::number(cameraSize.width());
    param << "height="+QString::number(cameraSize.height());
    param << "framerate="+QString::number(framerate);
    if (!fullscreen) {
        param << "start_x="+QString::number(displayRect.x());
        param << "start_y="+QString::number(displayRect.y());
        param << "disp_w="+QString::number(displayRect.width());
        param << "disp_h="+QString::number(displayRect.height());
    } else {
        param << "fullscreen="+QString::number(fullscreen);
    }
    param << "flip_h="+QString::number(flipH);
    param << "flip_v="+QString::number(flipV);
    param << "rotate="+QString::number(rotate);
    if (!filePath.isEmpty()) {
        param << "file_path="+filePath;
    }

    startThread(op, param);
}

/**
 * @brief QGstThreadController::start
 * @param op
 * @param device
 * @param cameraSize
 * @param framerate
 * @param displayRect
 * @param cropRect
 * @param fullscreen
 * @param filePath
 *
 * Public
 *
 * Arrange parameters to internal data.
 */
void QGstThreadController::start(QString op, QString device,
                                 QSize cameraSize, qint16 framerate, QRect displayRect,
                                 QRect cropRect,
                                 bool fullscreen, QString filePath)
{
    QStringList param;
    param << "camera="+device;
    param << "width="+QString::number(cameraSize.width());
    param << "height="+QString::number(cameraSize.height());
    param << "framerate="+QString::number(framerate);
    if (!fullscreen) {
        param << "start_x="+QString::number(displayRect.x());
        param << "start_y="+QString::number(displayRect.y());
        param << "disp_w="+QString::number(displayRect.width());
        param << "disp_h="+QString::number(displayRect.height());
    } else {
        param << "fullscreen="+QString::number(fullscreen);
    }
    param << "crop_x="+QString::number(cropRect.x());
    param << "crop_y="+QString::number(cropRect.y());
    param << "crop_w="+QString::number(cropRect.width());
    param << "crop_h="+QString::number(cropRect.height());
    if (!filePath.isEmpty()) {
        param << "file_path="+filePath;
    }

    startThread(op, param);
}

/**
 * @brief QGstThreadController::start
 * @param op
 * @param device
 * @param cameraSize
 * @param framerate
 * @param displayRect
 * @param flipH
 * @param flipV
 * @param rotate
 * @param cropRect
 * @param fullscreen
 * @param filePath
 *
 * Public
 *
 * Arrange parameters to internal data.
 */
void QGstThreadController::start(QString op, QString device,
                                 QSize cameraSize, qint16 framerate, QRect displayRect,
                                 bool flipH, bool flipV, quint8 rotate, QRect cropRect,
                                 bool fullscreen, QString filePath)
{
    QStringList param;
    param << "camera="+device;
    param << "width="+QString::number(cameraSize.width());
    param << "height="+QString::number(cameraSize.height());
    param << "framerate="+QString::number(framerate);
    if (!fullscreen) {
        param << "start_x="+QString::number(displayRect.x());
        param << "start_y="+QString::number(displayRect.y());
        param << "disp_w="+QString::number(displayRect.width());
        param << "disp_h="+QString::number(displayRect.height());
    } else{
        param << "fullscreen="+QString::number(fullscreen);
    }
    param << "flip_h="+QString::number(flipH);
    param << "flip_v="+QString::number(flipV);
    param << "rotate="+QString::number(rotate);
    param << "crop_x="+QString::number(cropRect.x());
    param << "crop_y="+QString::number(cropRect.y());
    param << "crop_w="+QString::number(cropRect.width());
    param << "crop_h="+QString::number(cropRect.height());
    if (!filePath.isEmpty()) {
        param << "file_path="+filePath;
    }

    startThread(op, param);
}

/**
 * @brief QGstThreadController::start
 * @param op
 * @param device
 * @param displayRect
 * @param fullscreen
 * @param filePath
 *
 * Public
 *
 * Arrange parameters to internal data.
 */
void QGstThreadController::start(QString op, QString device,
                                 QRect displayRect,
                                 bool fullscreen,
                                 QString filePath)
{
    QStringList param;
    param << "camera="+device;
    if (!fullscreen) {
        param << "start_x="+QString::number(displayRect.x());
        param << "start_y="+QString::number(displayRect.y());
        param << "disp_w="+QString::number(displayRect.width());
        param << "disp_h="+QString::number(displayRect.height());
    } else {
        param << "fullscreen="+QString::number(fullscreen);
    }
    param << "file_path="+filePath;

    startThread(op, param);
}

/**
 * @brief QGstThreadController::startThread
 * @param op
 * @param param
 *
 * Private
 *
 * Start the thread with internal parameter format.
 */
void QGstThreadController::startThread(QString op, QStringList param)
{
    if (bStart) {
        qDebug() << "QGstController already started";
        return;
    }

    workerThread.start(); // Start thread

    emit startPipeline(op, param);

    bStart = true;
}

/**
 * @brief QGstThreadController::stop
 *
 * Public
 *
 * Stop the thread.
 */
void QGstThreadController::stop()
{
    if (bStart) {
        emit stopPipeline();
    } else {
        qDebug() << "GstController not started yet";
    }
}

/*
 *  gstreamer pipeline slots
 */


/**
 * @brief QGstThreadController::pipelineStarted
 * @param op
 *
 * Public slot
 *
 * Triggered when gstreamer pipeline started.
 *
 * Emit a signal to main window.
 */
void QGstThreadController::pipelineStarted(const QString op)
{
    emit started(op);
}

/**
 * @brief QGstThreadController::pipelineFinished
 *
 * Public slot
 *
 * Trigger when gstreamer pipeline finished.
 *
 * Quit and wait the thread.
 * Emit a signal to main window.
 */
void QGstThreadController::pipelineFinished()
{
    // make sure workerThread stopped and cleared
    workerThread.quit();
    workerThread.wait();
    bStart = false;

    emit finished();
}

/**
 * @brief QGstThreadController::errorHandler
 * @param error
 *
 * Public slot
 *
 * Trigger when gstreamer pipeline raise an error.
 *
 * Emit a signal to main window.
 */
void QGstThreadController::errorHandler(const QString error)
{
    bStart = false;
    emit pipelineError(error);
}
