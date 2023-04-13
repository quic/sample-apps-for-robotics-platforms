/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/


#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include "qgstthreadcontroller.h"
#include "qgstpipeline.h"

QT_BEGIN_NAMESPACE
class QComboBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QCheckBox;
QT_END_NAMESPACE

/**
 * @brief The Window class
 *
 * Main Window
 *
 * UI interface to set parameters.
 *
 * Use QGstThreadController to start a gstreamer pipeline
 * in another thread.
 */
class Window : public QWidget
{
    Q_OBJECT

public:
    Window();

public slots:
    void sizeChanged(int);
    void cropCheckChanged(int);
    void fullscreenCheckChanged(int);
    void buttonClicked();
    void gstStarted(const QString op);
    void gstErrorHandle(const QString error);
    void gstFinished();

private:
    void createCameraControls(const QString &title);
    void createDisplayControls(const QString &title);
    void createTransformControls(const QString &title);
    void setControlEnable(bool en, QPushButton* except=nullptr);

    QGroupBox *cameraControlsGroup;
    QGroupBox *displayControlsGroup;
    QGroupBox *transformControlsGroup;
    QLabel *cameraIdLabel;
    QLabel *cameraWidthLabel;
    QLabel *cameraHeightLabel;
    QLabel *cameraResolutionLabel;
    QLabel *cameraFramerateLabel;
    QLabel *displayStartXLabel;
    QLabel *displayStartYLabel;
    QLabel *displayWidthLabel;
    QLabel *displayHeightLabel;
    QLabel *transformRotateLabel;
    QLabel *transformCropXLabel;
    QLabel *transformCropYLabel;
    QLabel *transformCropWLabel;
    QLabel *transformCropHLabel;


    QComboBox *cameraIdCombo;
    QLineEdit *cameraWidthEdit;
    QLineEdit *cameraHeightEdit;
    QComboBox *cameraResolutionCombo;
    QSpinBox  *cameraFramerateSpin;
    QCheckBox *displayFullscreenCheck;
    QLineEdit *displayStartXEdit;
    QLineEdit *displayStartYEdit;
    QLineEdit *displayWidthEdit;
    QLineEdit *displayHeightEdit;

    QCheckBox *transformFlipVCheck;
    QCheckBox *transformFlipHCheck;
    QComboBox *transformRotateCombo;
    QCheckBox *transformCropCheck;
    QLineEdit *transformCropXEdit;
    QLineEdit *transformCropYEdit;
    QLineEdit *transformCropWEdit;
    QLineEdit *transformCropHEdit;

    QPushButton *displayButton;
    QPushButton *recordButton;
    QPushButton *playButton;

private:
    QGstThreadController *gstThreadController;
};

#endif
