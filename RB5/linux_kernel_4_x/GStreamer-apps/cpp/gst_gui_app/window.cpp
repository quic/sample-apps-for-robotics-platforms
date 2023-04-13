/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <QtWidgets>
#include <QSize>
#include <QRect>
#include <QFileDialog>
#include <QDebug>

#include "window.h"

/**
 *  Constants
 */
const qint16 CameraWidthMin = 16;
const qint16 CameraHeightMin = 16;
const qint16 CameraWidthMax = 3840;
const qint16 CameraHeightMax = 2160;
const qint16 CameraFramerateMin = 0;
const qint16 CameraFramerateMax = 30;

const qint16 DisplayWidthMin = 0;
const qint16 DisplayHeightMin = 0;
const qint16 DisplayWidthMax = 1920;
const qint16 DisplayHeightMax = 1080;

const qint16 DisplayStartXMin = 0;
const qint16 DisplayStartYMin = 0;
const qint16 DisplayStartXMax = 1920;
const qint16 DisplayStartYMax = 1080;

const qint16 DefaultCropWidth = 640;
const qint16 DefaultCropHeight = 360;
const qint16 DefaultFramerate = 30;
const qint16 DefaultStartX = 40;
const qint16 DefaultStartY = 20;

/**
 * @brief Window::Window
 *
 * Constructor of Window
 */
Window::Window()
{
    createCameraControls(tr("Camera Control (for Display and Record)"));
    createTransformControls(tr("Transform Control (for Display only)"));
    createDisplayControls(tr("View Control (for Display and Playback)"));

    displayButton = new QPushButton("Display");
    recordButton = new QPushButton("Record");
    playButton = new QPushButton("Playback");

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(cameraControlsGroup);
    layout->addSpacing(12);
    layout->addWidget(transformControlsGroup);
    layout->addSpacing(12);
    layout->addWidget(displayControlsGroup);
    layout->addSpacing(12);
    QHBoxLayout *hlayout = new QHBoxLayout;
    hlayout->addWidget(displayButton);
    hlayout->addWidget(recordButton);
    hlayout->addWidget(playButton);
    layout->addLayout(hlayout);
    setLayout(layout);

    connect(displayButton, SIGNAL(clicked()),
            this, SLOT(buttonClicked()));
    connect(recordButton, SIGNAL(clicked()),
            this, SLOT(buttonClicked()));
    connect(playButton, SIGNAL(clicked()),
            this, SLOT(buttonClicked()));

    setWindowTitle(tr("Camera App"));

    resize(480, 360);

    gstThreadController = new QGstThreadController;
    connect(gstThreadController, SIGNAL(started(const QString)),
            this, SLOT(gstStarted(const QString)));
    connect(gstThreadController, SIGNAL(finished()),
            this, SLOT(gstFinished()));
    connect(gstThreadController, SIGNAL(pipelineError(const QString)),
            this, SLOT(gstErrorHandle(const QString)));
}

/**
 * @brief Window::createCameraControls
 * @param title
 *
 * Private
 *
 * Create controls that set camera paramters,
 * such as camera id, resolution, and framerate.
 */
void Window::createCameraControls(const QString &title)
{
    cameraControlsGroup = new QGroupBox(title);

    cameraIdLabel = new QLabel(tr("Camera ID:"));
    cameraIdCombo = new QComboBox;
    QStringList cameraId;
    cameraId << "0" << "1" << "2" << "3";
    cameraIdCombo->addItems(cameraId); 

    QIntValidator *pIntValidatorW = new QIntValidator(CameraWidthMin, CameraWidthMax, this);
    QIntValidator *pIntValidatorH = new QIntValidator(CameraHeightMin, CameraHeightMax, this);
    cameraWidthLabel = new QLabel(tr("Camera Width:"));
    cameraHeightLabel = new QLabel(tr("Camera Height:"));
    cameraWidthEdit = new QLineEdit;
    cameraHeightEdit = new QLineEdit;
    cameraWidthEdit->setMaxLength(8);
    cameraHeightEdit->setMaxLength(8);
    cameraWidthEdit->setValidator(pIntValidatorW);
    cameraHeightEdit->setValidator(pIntValidatorH);

    cameraResolutionLabel = new QLabel(tr("Resolution:"));
    cameraResolutionCombo = new QComboBox;
    cameraResolutionCombo->addItem(tr("Custom Size"), QSize(0,0));
    cameraResolutionCombo->addItem(tr("QCIF (176,144)"), QSize(176,144));
    cameraResolutionCombo->addItem(tr("QVGA (320,240)"), QSize(320,240));
    cameraResolutionCombo->addItem(tr("CIF (352,288)"), QSize(352,288));
    cameraResolutionCombo->addItem(tr("VGA (640,480)"), QSize(640,480));
    cameraResolutionCombo->addItem(tr("SVGA (800,600)"), QSize(800,600));
    cameraResolutionCombo->addItem(tr("XGA (1024,768)"), QSize(1024,768));
    cameraResolutionCombo->addItem(tr("XGA+ (1280,960)"), QSize(1280,960));
    cameraResolutionCombo->addItem(tr("360p (640,360)"), QSize(640,360));
    cameraResolutionCombo->addItem(tr("480p (720,480)"), QSize(720,480));
    cameraResolutionCombo->addItem(tr("720p (1280,720)"), QSize(1280,720));
    cameraResolutionCombo->addItem(tr("1080p (1920,1080)"), QSize(1920,1080));
    cameraResolutionCombo->addItem(tr("4K (3840,2160)"), QSize(3840,2160));
    // Default size
    int defaultIndex = cameraResolutionCombo->count() - 2;
    cameraResolutionCombo->setCurrentIndex(defaultIndex);
    QSize size = qvariant_cast<QSize>(cameraResolutionCombo->itemData(defaultIndex));
    cameraWidthEdit->setText(QString::number(size.width()));
    cameraHeightEdit->setText(QString::number(size.height()));

    cameraFramerateLabel = new QLabel(tr("Framerate(fps)"));
    cameraFramerateSpin = new QSpinBox;
    cameraFramerateSpin->setValue(DefaultFramerate);
    cameraFramerateSpin->setMinimum(CameraFramerateMin);
    cameraFramerateSpin->setMaximum(CameraFramerateMax);


    QGridLayout *controlsLayout = new QGridLayout;

    controlsLayout->addWidget(cameraIdLabel, 0, 0, Qt::AlignRight);
    controlsLayout->addWidget(cameraIdCombo, 0, 1);
    controlsLayout->addWidget(cameraWidthLabel, 1, 0, Qt::AlignRight);
    controlsLayout->addWidget(cameraWidthEdit, 1, 1);
    controlsLayout->addWidget(cameraHeightLabel, 1, 2, Qt::AlignRight);
    controlsLayout->addWidget(cameraHeightEdit, 1, 3);
    controlsLayout->addWidget(cameraResolutionLabel, 2, 0, Qt::AlignRight);
    controlsLayout->addWidget(cameraResolutionCombo, 2, 1, 1, 3);
    controlsLayout->addWidget(cameraFramerateLabel, 3, 0, Qt::AlignRight);
    controlsLayout->addWidget(cameraFramerateSpin, 3, 1);
    cameraControlsGroup->setLayout(controlsLayout);

    connect(cameraResolutionCombo, SIGNAL(currentIndexChanged(int)),
            this, SLOT(sizeChanged(int)));
}

/**
 * @brief Window::createDisplayControls
 * @param title
 *
 * Private
 *
 * Create controls that set display view paramters,
 * such as location(x,y), size, and fullscreen.
 */
void Window::createDisplayControls(const QString &title)
{
    displayControlsGroup = new QGroupBox(title);

    displayFullscreenCheck = new QCheckBox(tr("Fullscreen"));

    QIntValidator *pIntValidatorX = new QIntValidator(DisplayStartXMin, DisplayStartXMax, this);
    QIntValidator *pIntValidatorY = new QIntValidator(DisplayStartYMin, DisplayStartYMax, this);
    displayStartXLabel = new QLabel(tr("Start X:"));
    displayStartYLabel = new QLabel(tr("Start Y:"));
    displayStartXEdit = new QLineEdit;
    displayStartYEdit = new QLineEdit;

    displayStartXEdit->setMaxLength(8);
    displayStartYEdit->setMaxLength(8);
    displayStartXEdit->setValidator(pIntValidatorX);
    displayStartYEdit->setValidator(pIntValidatorY);
    displayStartXEdit->setText(QString::number(DefaultStartX));
    displayStartYEdit->setText(QString::number(DefaultStartY));

    QIntValidator *pIntValidatorW = new QIntValidator(DisplayWidthMin, DisplayWidthMax, this);
    QIntValidator *pIntValidatorH = new QIntValidator(DisplayHeightMin, DisplayHeightMax, this);
    displayWidthLabel = new QLabel(tr("View Width:"));
    displayHeightLabel = new QLabel(tr("View Height:"));
    displayWidthEdit = new QLineEdit;
    displayHeightEdit = new QLineEdit;
    displayWidthEdit->setMaxLength(8);
    displayHeightEdit->setMaxLength(8);
    displayWidthEdit->setValidator(pIntValidatorW);
    displayHeightEdit->setValidator(pIntValidatorH);
    // Default display size based on camera size
    QSize size = qvariant_cast<QSize>(cameraResolutionCombo->currentData());
    displayWidthEdit->setText(QString::number(size.width()));
    displayHeightEdit->setText(QString::number(size.height()));


    QGridLayout *controlsLayout = new QGridLayout;

    controlsLayout->addWidget(displayFullscreenCheck, 0, 1);
    controlsLayout->addWidget(displayStartXLabel, 1, 0, Qt::AlignRight);
    controlsLayout->addWidget(displayStartXEdit, 1, 1);
    controlsLayout->addWidget(displayStartYLabel, 1, 2, Qt::AlignRight);
    controlsLayout->addWidget(displayStartYEdit, 1, 3);
    controlsLayout->addWidget(displayWidthLabel, 2, 0, Qt::AlignRight);
    controlsLayout->addWidget(displayWidthEdit, 2, 1);
    controlsLayout->addWidget(displayHeightLabel, 2, 2, Qt::AlignRight);
    controlsLayout->addWidget(displayHeightEdit, 2, 3);

    displayControlsGroup->setLayout(controlsLayout);

    connect(displayFullscreenCheck, SIGNAL(stateChanged(int)), this, SLOT(fullscreenCheckChanged(int)));
}


/**
 * @brief Window::createTransformControls
 * @param title
 *
 * Private
 *
 * Create controls that set transform paramters,
 * such as flip, rotate, and crop.
 */
void Window::createTransformControls(const QString &title)
{
    transformControlsGroup = new QGroupBox(title);

    transformFlipHCheck = new QCheckBox("Flip Horizontal");
    transformFlipVCheck = new QCheckBox("Flip Vertical");

    transformRotateLabel = new QLabel(tr("Rotate:"));
    transformRotateCombo = new QComboBox;
    transformRotateCombo->addItem(tr("0: none - No rotation"), 0);
    transformRotateCombo->addItem(tr("1: 90CW - Rotate 90 degrees clockwise"), 1);
    transformRotateCombo->addItem(tr("2: 90CCW - Rotate 90 degrees counter-clockwise"), 2);
    transformRotateCombo->addItem(tr("3: 180 - Rotate 180 degrees"), 3);

    transformCropCheck = new QCheckBox(tr("Crop"));

    QIntValidator *pIntValidatorX = new QIntValidator(CameraWidthMin, CameraWidthMax, this);
    QIntValidator *pIntValidatorY = new QIntValidator(CameraHeightMin, CameraHeightMax, this);
    transformCropXLabel = new QLabel(tr("Crop X:"));
    transformCropYLabel = new QLabel(tr("Crop Y:"));
    transformCropXEdit = new QLineEdit;
    transformCropYEdit = new QLineEdit;

    transformCropXEdit->setMaxLength(8);
    transformCropYEdit->setMaxLength(8);
    transformCropXEdit->setValidator(pIntValidatorX);
    transformCropYEdit->setValidator(pIntValidatorY);
    transformCropXEdit->setText(QString::number(0));
    transformCropYEdit->setText(QString::number(0));

    transformCropWLabel = new QLabel(tr("Crop Width:"));
    transformCropHLabel = new QLabel(tr("Crop Height:"));
    transformCropWEdit = new QLineEdit;
    transformCropHEdit = new QLineEdit;
    transformCropWEdit->setMaxLength(8);
    transformCropHEdit->setMaxLength(8);
    transformCropWEdit->setValidator(pIntValidatorX);
    transformCropHEdit->setValidator(pIntValidatorY);
    transformCropWEdit->setText(QString::number(DefaultCropWidth));
    transformCropHEdit->setText(QString::number(DefaultCropHeight));

    // Crop default disabled
    transformCropXEdit->setEnabled(false);
    transformCropYEdit->setEnabled(false);
    transformCropWEdit->setEnabled(false);
    transformCropHEdit->setEnabled(false);

    QGridLayout *controlsLayout = new QGridLayout;

    controlsLayout->addWidget(transformFlipHCheck, 0, 1);
    controlsLayout->addWidget(transformFlipVCheck, 0, 2);
    controlsLayout->addWidget(transformRotateLabel, 2, 0, Qt::AlignRight);
    controlsLayout->addWidget(transformRotateCombo, 2, 1, 1, 3);
    controlsLayout->addWidget(transformCropCheck, 3, 1);
    controlsLayout->addWidget(transformCropXLabel, 4, 0, Qt::AlignRight);
    controlsLayout->addWidget(transformCropXEdit, 4, 1);
    controlsLayout->addWidget(transformCropYLabel, 4, 2, Qt::AlignRight);
    controlsLayout->addWidget(transformCropYEdit, 4, 3);
    controlsLayout->addWidget(transformCropWLabel, 5, 0, Qt::AlignRight);
    controlsLayout->addWidget(transformCropWEdit, 5, 1);
    controlsLayout->addWidget(transformCropHLabel, 5, 2, Qt::AlignRight);
    controlsLayout->addWidget(transformCropHEdit, 5, 3);

    transformControlsGroup->setLayout(controlsLayout);

    connect(transformCropCheck, SIGNAL(stateChanged(int)), this, SLOT(cropCheckChanged(int)));
}

/**
 * @brief Window::sizeChanged
 * @param index
 *
 * Public slot
 *
 * Triggered when camera resolution changed.
 *
 * Selected resolution will set to coresponding TextEdit.
 */
void Window::sizeChanged(int index)
{
    if (index == 0) {
        cameraWidthEdit->clear();
        cameraHeightEdit->clear();
        return;
    }

    QComboBox *combo = dynamic_cast<QComboBox *>(QObject::sender());
    QSize size = qvariant_cast<QSize>(combo->currentData());
    qint16 cameraWidth = size.width();
    qint16 cameraHeight = size.height();
    qint16 displayWidth;
    qint16 displayHeight;

    cameraWidthEdit->setText(QString::number(cameraWidth));
    cameraHeightEdit->setText(QString::number(cameraHeight));
    displayWidth = (cameraWidth > DisplayWidthMax) ? DisplayWidthMax : cameraWidth;
    displayHeight = (cameraHeight > DisplayHeightMax) ? DisplayHeightMax : cameraHeight;

    displayWidthEdit->setText(QString::number(displayWidth));
    displayHeightEdit->setText(QString::number(displayHeight));
}

/**
 * @brief Window::cropCheckChanged
 * @param state
 *
 * Public slot
 *
 * Triggered when crop checkbox changed.
 *
 * Crop TextEdits are enabled only when crop checkbox checked.
 */
void Window::cropCheckChanged(int state)
{
    bool checked = (state == Qt::Checked);
    transformCropXEdit->setEnabled(checked);
    transformCropYEdit->setEnabled(checked);
    transformCropWEdit->setEnabled(checked);
    transformCropHEdit->setEnabled(checked);
}

/**
 * @brief Window::fullscreenCheckChanged
 * @param state
 *
 * Public slot
 *
 * Triggered when fullscreen checkbox changed.
 *
 * Display location and size TextEdits are disabbled
 * when fullscreen checkbox checked.
 */
void Window::fullscreenCheckChanged(int state)
{
    bool checked = (state == Qt::Checked);
    displayStartXEdit->setEnabled(!checked);
    displayStartYEdit->setEnabled(!checked);
    displayWidthEdit->setEnabled(!checked);
    displayHeightEdit->setEnabled(!checked);
}

/**
 * @brief Window::setControlEnable
 * @param en
 * @param except
 *
 * Private
 *
 * Set controls enabled or disabled.
 *
 * When gstreamer pipeline starting, all controls
 * should be disabled to avoid changing parameters.
 * Especially buttons should be disabled to avoid
 * clicked again before entering stable state.
 *
 * Once gstreamer pipeline started, a 'stop' button
 * should be enabled for ending the pipeline later.
 * Argument 'except' specifys the 'stop' button.
 *
 */
void Window::setControlEnable(bool en, QPushButton *except)
{
    cameraIdCombo->setEnabled(en);
    cameraWidthEdit->setEnabled(en);
    cameraHeightEdit->setEnabled(en);
    cameraResolutionCombo->setEnabled(en);
    cameraFramerateSpin->setEnabled(en);
    displayFullscreenCheck->setEnabled(en);
    if (en && displayFullscreenCheck->checkState() != Qt::Checked) {
        displayStartXEdit->setEnabled(true);
        displayStartYEdit->setEnabled(true);
        displayWidthEdit->setEnabled(true);
        displayHeightEdit->setEnabled(true);
    } else{
        displayStartXEdit->setEnabled(false);
        displayStartYEdit->setEnabled(false);
        displayWidthEdit->setEnabled(false);
        displayHeightEdit->setEnabled(false);
    }
    transformFlipHCheck->setEnabled(en);
    transformFlipVCheck->setEnabled(en);
    transformRotateCombo->setEnabled(en);
    transformCropCheck->setEnabled(en);
    if (en && transformCropCheck->checkState() == Qt::Checked) {
        transformCropXEdit->setEnabled(true);
        transformCropYEdit->setEnabled(true);
        transformCropWEdit->setEnabled(true);
        transformCropHEdit->setEnabled(true);
    } else {
        transformCropXEdit->setEnabled(false);
        transformCropYEdit->setEnabled(false);
        transformCropWEdit->setEnabled(false);
        transformCropHEdit->setEnabled(false);
    }
    // --
    displayButton->setEnabled(en);
    recordButton->setEnabled(en);
    playButton->setEnabled(en);

    if (en) {
        displayButton->setText(tr("Display"));
        recordButton->setText(tr("Record"));
        playButton->setText(tr("Playback"));
    } else {
        if (except == displayButton) {
            displayButton->setText(tr("Stop"));
            displayButton->setEnabled(true);
        } else if (except == recordButton) {
            recordButton->setText(tr("Stop"));
            recordButton->setEnabled(true);
        } else if (except == playButton) {
            playButton->setText(tr("Stop"));
            playButton->setEnabled(true);
        }
    }
}

/**
 * @brief Window::buttonClicked
 *
 * Public slot
 *
 * Triggered when button clicked
 *
 * Do the procedure according to the clicked button
 * and the state of gstreamer pipeline.
 */
void Window::buttonClicked()
{
    QPushButton *source = dynamic_cast<QPushButton *>(QObject::sender());
    QString op = DISPLAY;

    if (source == displayButton) {
        op = DISPLAY;
    } else if (source == recordButton) {
        op = RECORD;
    } else if (source == playButton) {
        op = PLAYBACK;
    }

    if (!gstThreadController->isStart()) {

        if (cameraWidthEdit->text().isEmpty() ||
            cameraHeightEdit->text().isEmpty() ||
            displayStartXEdit->text().isEmpty() ||
            displayStartYEdit->text().isEmpty() ||
            displayWidthEdit->text().isEmpty() ||
            displayHeightEdit->text().isEmpty()) {
            QMessageBox msgBox;
            msgBox.setText("Some fields are empty.");
            msgBox.exec();
            return;
        }

        QString filePath;
        if (op == RECORD) {
            filePath = QFileDialog::getSaveFileName(this, "Save to", "/data/record.mp4", "*.mp4");
            if (filePath.isEmpty()) {
                return;
            }
        } else if (op == PLAYBACK) {
            filePath = QFileDialog::getOpenFileName(this, "Open", "/data", "*.mp4");
            if (filePath.isEmpty()) {
                return;
            }
        }

        // Disable controls
        // Some controls may be enabled in gstFinished or gstErrorHandle slots.
        setControlEnable(false);

        if (op == DISPLAY || op == RECORD) {
            QString device = cameraIdCombo->currentText();
            QSize cameraSize(cameraWidthEdit->text().toInt(), cameraHeightEdit->text().toInt());
            qint16 framerate = cameraFramerateSpin->value();
            bool displayFullscreen = (displayFullscreenCheck->checkState() == Qt::Checked);
            QRect displayRect(displayStartXEdit->text().toInt(), displayStartYEdit->text().toInt(),
                              displayWidthEdit->text().toInt(),  displayHeightEdit->text().toInt());
            bool isFlipH = (transformFlipHCheck->checkState() == Qt::Checked);
            bool isFlipV = (transformFlipVCheck->checkState() == Qt::Checked);
            int rotateValue = qvariant_cast<int>(transformRotateCombo->currentData());
            bool isTransformFlipRotate = (isFlipH || isFlipV || rotateValue != 0);
            bool isTransformCrop = (transformCropCheck->checkState() == Qt::Checked);
            QRect cropRect;
            if (isTransformCrop) {
                cropRect.setX(transformCropXEdit->text().toInt());
                cropRect.setY(transformCropYEdit->text().toInt());
                cropRect.setWidth(transformCropWEdit->text().toInt());
                cropRect.setHeight(transformCropHEdit->text().toInt());
            }

            if (!isTransformFlipRotate && !isTransformCrop) {
                if (op == DISPLAY) {
                    gstThreadController->start(op, device, cameraSize, framerate,
                                               displayRect, displayFullscreen);
                } else if (op == RECORD) {
                    gstThreadController->start(op, device, cameraSize, framerate,
                                               displayRect, displayFullscreen, filePath);
                }
            } else if (isTransformFlipRotate && !isTransformCrop) {
                if (op == DISPLAY) {
                    gstThreadController->start(op, device, cameraSize, framerate, displayRect,
                                                isFlipH, isFlipV, rotateValue, displayFullscreen);
                } else if (op == RECORD) {
                    gstThreadController->start(op, device, cameraSize, framerate, displayRect,
                                                isFlipH, isFlipV, rotateValue, displayFullscreen, filePath);
                }
            } else if (!isTransformFlipRotate && isTransformCrop) {
                if (op == DISPLAY) {
                    gstThreadController->start(op, device, cameraSize, framerate,
                                               displayRect, cropRect, displayFullscreen);
                } else if (op == RECORD) {
                    gstThreadController->start(op, device, cameraSize, framerate,
                                               displayRect, cropRect, displayFullscreen, filePath);
                }
            } else {
                if (op == DISPLAY) {
                    gstThreadController->start(op, device, cameraSize, framerate, displayRect,
                                             isFlipH, isFlipV, rotateValue, cropRect, displayFullscreen);
                } else if (op == RECORD) {
                    gstThreadController->start(op, device, cameraSize, framerate, displayRect,
                                             isFlipH, isFlipV, rotateValue, cropRect, displayFullscreen, filePath);
                }
            }
        } // if (op == DISPLAY || op == RECORD)
        else if (op == PLAYBACK) {
            QString device = cameraIdCombo->currentText();
            QRect displayRect(displayStartXEdit->text().toInt(), displayStartYEdit->text().toInt(),
                              displayWidthEdit->text().toInt(),  displayHeightEdit->text().toInt());
            bool displayFullscreen = (displayFullscreenCheck->checkState() == Qt::Checked);

            gstThreadController->start(op, device, displayRect, displayFullscreen, filePath);
        }

    } else {     // if (!gstThreadController->isStart())
        // Disable controls
        setControlEnable(false);
        gstThreadController->stop();
        ///
        /// Clear resource and reset UI in gstFinished slot
        ///
    }
}

/*
 * gstreamer slots
 */

/**
 * @brief Window::gstStarted
 * @param op
 *
 * Public slot
 *
 * Triggered when gstreamer pipeline started
 *
 * Set the control according to the current operation.
 */
void Window::gstStarted(const QString op)
{
    qDebug() << "Window" << op << "started slot";

    if (op == DISPLAY) {
        // set controls disabled but displayButton should be enabled.
        setControlEnable(false, displayButton);
    } else if (op == RECORD) {
        // set controls disabled but recordButton should be enabled.
        setControlEnable(false, recordButton);
    } else if (op == PLAYBACK) {
        // set controls disabled but playButton should be enabled.
        setControlEnable(false, playButton);
    }

    /**
     * This is a workaround.
     * If the main window is totally covered by camera view,
     * there is no way to bring the main window to the top.
     * This workaround makes the main window hidden and brough back.
     * The side effect is that the position of main window is
     * different to the original position.
     */
    if (displayFullscreenCheck->checkState() == Qt::Checked ||
        displayWidthEdit->text().toDouble() >= (double)DisplayWidthMax * 0.6 ||
        displayHeightEdit->text().toDouble() >= (double)DisplayHeightMax* 0.6) {
        setVisible(false);
        QThread::sleep(2);
        setVisible(true);
    }
}

/**
 * @brief Window::gstFinished
 *
 * Public slot
 *
 * Triggered when gstreamer pipeline finished.
 *
 * Reset control state.
 */
void Window::gstFinished()
{
    qDebug() << "Window gstFinished slot";

    setControlEnable(true);
}

/**
 * @brief Window::gstErrorHandle
 * @param error
 *
 * Public slot
 *
 * Triggered when getting errors from gstreamer pipeline.
 *
 * Popup a message box and reset control state.
 */
void Window::gstErrorHandle(const QString error)
{
    qDebug() << "Window gstErrorHandle slot: " << error;

    QMessageBox msgBox;
    msgBox.setText(error);
    msgBox.exec();


    // Once error happens, some resources are disordered.
    // Force to quit the application.
    qApp->quit();
}
