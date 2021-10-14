import gi
from gi.repository import Gst, GLib
from PyQt5.QtWidgets import QAction, QApplication, QGridLayout, QWidget, QComboBox, QGroupBox, QLabel, QLineEdit, QPushButton, QSpinBox, QCheckBox, QVBoxLayout, QHBoxLayout, QBoxLayout, QFileDialog
from PyQt5.QtCore import Qt, QSize, QThread
import sys
import os
import signal


gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")


# Constants
CameraWidthMin = 16
CameraHeightMin = 16
CameraWidthMax = 3840
CameraHeightMax = 2160
CameraFramerateMin = 0
CameraFramerateMax = 30
CameraId = 0
CameraWidth = 1920
CameraHeight = 1080
CameraFrameRate = 30

DisplayWidthMin = 0
DisplayHeightMin = 0
DisplayWidthMax = 1920
DisplayHeightMax = 1080
DisplayFlipHorizontal = 0
DisplayFlipVertical = 0
DisplayRotation = False
DisplayRotate = 0
DisplayCrop = False
DisplayCropX = 0
DisplayCropY = 0
DisplayCropWidth = 0
DisplayCropHeight = 0

DisplayStartX = 200
DisplayStartY = 200
DisplayHeight = 1080
DisplayWidth = 1920
DisplayStartXMin = 0
DisplayStartYMin = 0
DisplayStartXMax = 1920
DisplayStartYMax = 1080

DefaultCropWidth = 640
DefaultCropHeight = 360
DefaultFramerate = 30
DefaultStartX = 40
DefaultStartY = 20
DisplayFullScreen = False

# Display with transform  
def liveView(window, btn, recbtn, playbackbtn):

    if(btn.text() == "Display"):
        btn.setEnabled(True)
        btn.setText("Stop")
        setButtons([], [recbtn, playbackbtn])
        Gst.init()
        loop = GLib.MainLoop()
        pipeline = Gst.Pipeline.new("video-display")

        # Creating the elements
        source = Gst.ElementFactory.make("qtiqmmfsrc",   "qmmf-source")
        framefilter = Gst.ElementFactory.make("capsfilter",   "frame-filter")
        transform = Gst.ElementFactory.make("qtivtransform", "transform")
        sink = Gst.ElementFactory.make("waylandsink",  "display")

        # Linking the created element 
        if not pipeline or not source or not framefilter or not transform or not sink:
            print("Element create failed.")

        source.set_property("camera", CameraId)

        # Adding the properties for framefilter
        cap_string = f"video/x-raw,format=NV12,framerate={CameraFrameRate}/1,width={CameraWidth},height={CameraHeight}"
        framefilter.set_property("caps",
                                 Gst.caps_from_string(cap_string))
        transform.set_property("flip-vertical", DisplayFlipVertical)
        transform.set_property("flip-horizontal", DisplayFlipHorizontal)
        transform.set_property("rotate", DisplayRotate)
        if DisplayCrop == True:
            transform.set_property("crop-x", DisplayCropX)
            transform.set_property("crop-y", DisplayCropY)
            transform.set_property("crop-width", DisplayCropWidth)
            transform.set_property("crop-height", DisplayCropHeight)

        # Modify the properties for sink
        if(DisplayFullScreen):
            sink.set_property("fullscreen", "true")
        else:
            sink.set_property("x", DisplayStartX)
            sink.set_property("y", DisplayStartY)
            sink.set_property("width", DisplayWidth)
            sink.set_property("height", DisplayHeight)

        # Build the pipeline
        pipeline.add(source)
        pipeline.add(framefilter)
        pipeline.add(transform)
        pipeline.add(sink)

        if not source.link(framefilter):
            print("ERROR: Could not link source to framefilter")
            return
        if not framefilter.link(transform):
            print("ERROR: Could not link framefilter to transform")
            return
        if not transform.link(sink):
            print("ERROR: Could not link transform to sink")
            return
        try:
            # Start playing
            pipeline.set_state(Gst.State.PLAYING)
            print("Display Started...")
            loop.run()
        except:
            btn.setText("Display")
            print("Display Stopped...")
            pipeline.set_state(Gst.State.NULL)
            del pipeline
            setButtons([recbtn, playbackbtn, btn], [])
    else:
        os.kill(os.getegid(), signal.SIGINT)


def on_pad_added(bus, pad, data):
    srcpad = data
    sinkpad = Gst.Element.get_static_pad(srcpad, "sink")
    Gst.Pad.link(pad, sinkpad)
    return True

# Video Recording 
def record(win, btn, btnplayback, btndisplay):
    if(btn.text() == "Record"):
        widget = QWidget()
        fileDialog = QFileDialog(widget)
        filepath = fileDialog.getSaveFileName(
            btn, "Save to", "/data/record.mp4", "*.mp4")
        if filepath[0]:
            setButtons([btn], [btndisplay, btnplayback])
            btn.setText("Stop")
            Gst.init()
            loop = GLib.MainLoop()
            pipeline = Gst.Pipeline.new("video-display")

        # Create the elements
            cap_string = f"video/x-raw,format=NV12,framerate={CameraFrameRate}/1,width={CameraWidth},height={CameraHeight}"

            source = Gst.ElementFactory.make("qtiqmmfsrc",   "qmmf-source")
            display_framefilter = Gst.ElementFactory.make(
                "capsfilter", "frame-filter")
            transform = Gst.ElementFactory.make("qtivtransform", "transform")
            sink = Gst.ElementFactory.make("waylandsink",  "display")
            record_framefilter = Gst.ElementFactory.make(
                "capsfilter", "frame-filter")
            parse = Gst.ElementFactory.make("h264parse", "h264-parse")
            muxer = Gst.ElementFactory.make("mpegtsmux", "mpegts-muxer")
            queue = Gst.ElementFactory.make("queue", "queue")
            filesink = Gst.ElementFactory.make("filesink", "file-output")

            if not pipeline or not source or not transform or not sink or not parse or not muxer or not queue or not filesink:
                print("Element create failed.")

            source.set_property("name", "qmmf")
            filesink.set_property("location", filepath[0])
            record_framefilter.set_property(
                "caps", Gst.caps_from_string(cap_string))
            display_framefilter.set_property(
                "caps", Gst.caps_from_string(cap_string))
            sink.set_property("qmmf.")

            if(DisplayFullScreen):
                sink.set_property("fullscreen", "true")
            else:
                sink.set_property("x", DisplayStartX)
                sink.set_property("y", DisplayStartY)
                sink.set_property("width", DisplayWidth)
                sink.set_property("height", DisplayHeight)
            
            # Adding element to pipeline 
            pipeline.add(source)
            pipeline.add(display_framefilter)
            pipeline.add(transform)
            pipeline.add(sink)
            pipeline.add(record_framefilter)
            pipeline.add(parse)
            pipeline.add(muxer)
            pipeline.add(queue)
            pipeline.add(filesink)

            # Linking the elements 
            if not source.link(display_framefilter):
                print("ERROR: Could not link source to demux")
                return
            if not decoder.link(Videoconv):
                print("ERROR: Could not link decoder to video_convert")
                return
            if not Videoconv.link(sink):
                print("ERROR: Could not link scaler to sink")
                return

            try:
                # Start playing
                pipeline.set_state(Gst.State.PLAYING)
                print("Video playback started...")
                loop.run()
                win.setVisible(False)
                QThread.sleep(2)
                win.setVisible(True)
            except KeyboardInterrupt:
                print("Video playback stopped...")
                pipeline.set_state(Gst.State.NULL)
                del pipeline
                btn.setText("Playback")
                setButtons([btn, btndisplay, btnrecord], [])
    else:
        os.kill(os.getegid(), signal.SIGINT)


def playback(win, btn, btnrecord, btndisplay):
    if(btn.text() == "Playback"):
        widget = QWidget()
        fileDialog = QFileDialog(widget)
        filepath = fileDialog.getOpenFileName(
            fileDialog, "Open", "/data", "*.mp4")
        if filepath[0]:
            setButtons([btn], [btndisplay, btnrecord])
            btn.setEnabled(True)
            btn.setText("Stop")
            Gst.init()
            loop = GLib.MainLoop()
            pipeline = Gst.Pipeline.new("video-display")

        # Create the elements
            source = Gst.ElementFactory.make("filesrc", "file-source")
            sink = Gst.ElementFactory.make("waylandsink", "video-output")
            decoder = Gst.ElementFactory.make("avdec_h264", "h264-decoder")
            demuxer = Gst.ElementFactory.make("qtdemux", "qt-demuxer")
            Videoconv = Gst.ElementFactory.make("videoconvert", "converter")

            if not pipeline or not source or not demuxer or not decoder or not sink or not Videoconv:
                print("Element not created.")
                return

            bus = pipeline.get_bus()
            bus_id = bus.add_signal_watch()
            Gst.Object.unref(bus)

            # Build the pipeline
            pipeline.add(source)
            pipeline.add(demuxer)
            pipeline.add(decoder)
            pipeline.add(Videoconv)
            pipeline.add(sink)

            if not source.link(demuxer):
                print("ERROR: Could not link source to demux")
                sys.exit(1)

            if not decoder.link(Videoconv):
                print("ERROR: Could not link decoder to video_convert")
                sys.exit(1)

            if not Videoconv.link(sink):
                print("ERROR: Could not link scaler to sink")
                sys.exit(1)

            source.set_property("location", filepath[0])
            # Modify the properties for sink
            if(DisplayFullScreen):
                sink.set_property("fullscreen", "true")
            else:
                sink.set_property("x", DisplayStartX)
                sink.set_property("y", DisplayStartY)
                sink.set_property("width", DisplayWidth)
                sink.set_property("height", DisplayHeight)

            demuxer.connect("pad-added", on_pad_added, decoder)

            try:
                # Start playing
                pipeline.set_state(Gst.State.PLAYING)
                print("Video playback started...")

            except KeyboardInterrupt:
                print("Video playback stopped...")
                pipeline.set_state(Gst.State.NULL)
                del pipeline
                btn.setText("Playback")
                setButtons([btn, btndisplay, btnrecord], [])
    else:
        os.kill(os.getegid(), signal.SIGINT)


def window():
    app = QApplication(sys.argv)
    win = QWidget()
    win.setGeometry(0, 0, 480, 360)
    win.setWindowTitle("Camera App")
    # Camera Group
    grp_camera = createCameraControls(
        "Camera Control (for Display and Record)")
    grp_transform = createTranformControls(
        "Transform Control( for Display Only)")
    grp_view = createViewControls("View Control (for Display and Playback)")

    # Add Bottom Buttons
    btn_display = QPushButton("Display")
    btn_record = QPushButton("Record")
    btn_playback = QPushButton("Playback")
    btn_display.clicked.connect(
        lambda: liveView(win, btn_display, btn_record, btn_playback))
    btn_playback.clicked.connect(
        lambda: playback(win, btn_playback, btn_record, btn_display))
    btn_record.clicked.connect(
        lambda: record(win, btn_record, btn_playback, btn_display))

    # Adding component to layout
    lyt_vbox = QVBoxLayout()
    lyt_vbox.addWidget(grp_camera)
    lyt_vbox.addSpacing(12)
    lyt_vbox.addWidget(grp_transform)
    lyt_vbox.addSpacing(12)
    lyt_vbox.addWidget(grp_view)
    lyt_vbox.addSpacing(12)

    lyt_hbox = QHBoxLayout()
    lyt_hbox.addWidget(btn_display)
    lyt_hbox.addWidget(btn_record)
    lyt_hbox.addWidget(btn_playback)
    lyt_vbox.addLayout(lyt_hbox)

    win.setLayout(lyt_vbox)

    win.show()
    sys.exit(app.exec_())


# Creating Camera Controls
def createCameraControls(grp_title):
    grp_camera = QGroupBox(grp_title)
    lyt_grid = QGridLayout()

    # Camera ID
    lbl_camera_id = QLabel("Camera ID: ")
    cbbox_camera_id = QComboBox()
    cbbox_camera_id.addItems(["0", "1", "2", "3"])
    cbbox_camera_id.currentIndexChanged[str].connect(
        lambda: setCameraId(cbbox_camera_id))

    # Camera Height and Width
    lbl_camera_width = QLabel("Camera Width: ")
    lbl_camera_height = QLabel("Camera Height: ")
    edi_camera_width = QLineEdit("1920")
    edi_camera_height = QLineEdit("1080")
    edi_camera_height.setMaxLength(8)
    edi_camera_width.setMaxLength(8)

    # Camera Resolution
    lbl_camera_resolution = QLabel("Resolution: ")
    cbbox_camera_resolution = QComboBox()
    cbbox_camera_resolution.addItem("QCIF (176,144)", QSize(176, 144))
    cbbox_camera_resolution.addItem("QVGA (320,240)", QSize(320, 240))
    cbbox_camera_resolution.addItem("CIF (352,288)", QSize(352, 288))
    cbbox_camera_resolution.addItem("VGA (640,480)", QSize(640, 480))
    cbbox_camera_resolution.addItem("SVGA (800,600)", QSize(800, 600))
    cbbox_camera_resolution.addItem("XGA (1024,768)", QSize(1024, 768))
    cbbox_camera_resolution.addItem("XGA+ (1280,960)", QSize(1280, 960))
    cbbox_camera_resolution.addItem("360p (640,360)", QSize(640, 360))
    cbbox_camera_resolution.addItem("480p (720,480)", QSize(720, 480))
    cbbox_camera_resolution.addItem("720p (1280,720)", QSize(1280, 720))
    cbbox_camera_resolution.addItem("1080p (1920,1080)", QSize(1920, 1080))
    cbbox_camera_resolution.addItem("4K (3840,2160)", QSize(3840, 2160))
    cbbox_camera_resolution.setCurrentIndex(10)
    cbbox_camera_resolution.currentIndexChanged[str].connect(lambda: setCameraResolution(
        edi_camera_width, edi_camera_height, cbbox_camera_resolution))
    # Camera Framerate
    lbl_camera_framerate = QLabel("Framerate(fps): ")
    spn_camera_framerate = QSpinBox()
    spn_camera_framerate.setValue(DefaultFramerate)
    spn_camera_framerate.setMinimum(CameraFramerateMin)
    spn_camera_framerate.setMaximum(CameraFramerateMax)

    # Adding component to layout
    lyt_grid.addWidget(lbl_camera_id, 0, 0, Qt.AlignRight)
    lyt_grid.addWidget(cbbox_camera_id, 0, 1)
    lyt_grid.addWidget(lbl_camera_width, 1, 0, Qt.AlignRight)
    lyt_grid.addWidget(edi_camera_width, 1, 1)
    lyt_grid.addWidget(lbl_camera_height, 1, 2, Qt.AlignRight)
    lyt_grid.addWidget(edi_camera_height, 1, 3)
    lyt_grid.addWidget(lbl_camera_resolution, 2, 0, Qt.AlignRight)
    lyt_grid.addWidget(cbbox_camera_resolution, 2, 1, 1, 3)
    lyt_grid.addWidget(lbl_camera_framerate, 3, 0, Qt.AlignRight)
    lyt_grid.addWidget(spn_camera_framerate, 3, 1)
    grp_camera.setLayout(lyt_grid)

    return grp_camera


# Creating Transform Controls
def createTranformControls(grp_title):
    grp_tranform = QGroupBox(grp_title)
    lyt_grid = QGridLayout()

    # Camera Flip
    ckbox_flip_h = QCheckBox("Flip Horizontal")
    ckbox_flip_v = QCheckBox("Flip Vertical")
    ckbox_flip_h.stateChanged.connect(
        lambda: setDisplayFlipHorizontal(ckbox_flip_h.checkState()))
    ckbox_flip_v.stateChanged.connect(
        lambda: setDisplayFlipVertical(ckbox_flip_v.checkState()))

    # Camera Rotate
    lbl_rotate = QLabel("Rotate: ")
    cbbox_rotate = QComboBox()
    cbbox_rotate.addItem(("0: none - No rotation"), 0)
    cbbox_rotate.addItem(("1: 90CW - Rotate 90 degrees clockwise"), 1)
    cbbox_rotate.addItem(("2: 90CCW - Rotate 90 degrees counter-clockwise"), 2)
    cbbox_rotate.addItem(("3: 180 - Rotate 180 degrees"), 3)

    cbbox_rotate.currentIndexChanged.connect(
        lambda: setRotate(cbbox_rotate.currentData()))

    # Camera Crop
    ckbox_crop = QCheckBox("Crop")
    lbl_crop_x = QLabel("Crop X: ")
    lbl_crop_y = QLabel("Crop Y: ")
    lbl_crop_w = QLabel("Crop Width: ")
    lbl_crop_h = QLabel("Crop Height: ")
    edi_crop_x = QLineEdit("0")
    edi_crop_y = QLineEdit("0")
    edi_crop_w = QLineEdit("640")
    edi_crop_h = QLineEdit("360")

    edi_crop_x.textChanged.connect(
        lambda: setCropValue("x", int(edi_crop_x.text())))
    edi_crop_y.textChanged.connect(
        lambda: setCropValue("y", int(edi_crop_y.text())))
    edi_crop_w.textChanged.connect(
        lambda: setCropValue("w", int(edi_crop_w.text())))
    edi_crop_h.textChanged.connect(
        lambda: setCropValue("h", int(edi_crop_h.text())))

    edi_crop_x.setEnabled(False)
    edi_crop_y.setEnabled(False)
    edi_crop_w.setEnabled(False)
    edi_crop_h.setEnabled(False)

    ckbox_crop.stateChanged.connect(lambda: setCrop(
        ckbox_crop, [edi_crop_x, edi_crop_y, edi_crop_w, edi_crop_h]))

    # Adding Component to layout
    lyt_grid.addWidget(ckbox_flip_h, 0, 1)
    lyt_grid.addWidget(ckbox_flip_v, 0, 2)
    lyt_grid.addWidget(lbl_rotate, 2, 0, Qt.AlignRight)
    lyt_grid.addWidget(cbbox_rotate, 2, 1, 1, 3)
    lyt_grid.addWidget(ckbox_crop, 3, 1)
    lyt_grid.addWidget(lbl_crop_x, 4, 0, Qt.AlignRight)
    lyt_grid.addWidget(edi_crop_x, 4, 1)
    lyt_grid.addWidget(lbl_crop_y, 4, 2, Qt.AlignRight)
    lyt_grid.addWidget(edi_crop_y, 4, 3)
    lyt_grid.addWidget(lbl_crop_w, 5, 0, Qt.AlignRight)
    lyt_grid.addWidget(edi_crop_w, 5, 1)
    lyt_grid.addWidget(lbl_crop_h, 5, 2, Qt.AlignRight)
    lyt_grid.addWidget(edi_crop_h, 5, 3)

    grp_tranform.setLayout(lyt_grid)

    return grp_tranform


# Creating View Controls
def createViewControls(grp_title):
    grp_view = QGroupBox(grp_title)

    # Camera Fullscreen
    ckbox_fullscreen = QCheckBox("Fullscreen")

    # Starting X & Y points
    lbl_start_x = QLabel("Start X: ")
    lbl_start_y = QLabel("Start Y: ")
    edi_start_x = QLineEdit(str(DefaultStartX))
    edi_start_y = QLineEdit(str(DefaultStartY))
    edi_start_x.textChanged.connect(lambda: setDisplay("x", edi_start_x))
    edi_start_y.textChanged.connect(lambda: setDisplay("y", edi_start_y))

    # View Width & Height
    lbl_view_w = QLabel("View Width: ")
    lbl_view_h = QLabel("Start Height: ")
    edi_view_w = QLineEdit("1920")
    edi_view_h = QLineEdit("1080")
    edi_view_w.textChanged.connect(lambda: setDisplay("w", edi_view_w))
    edi_view_h.textChanged.connect(lambda: setDisplay("h", edi_view_h))

    ckbox_fullscreen.stateChanged.connect(lambda: setFullScreen(
        ckbox_fullscreen, [edi_start_x, edi_start_y, edi_view_w, edi_view_h]))

    # Adding component to Grid Layout
    lyt_grid = QGridLayout()
    lyt_grid.addWidget(ckbox_fullscreen, 0, 1)
    lyt_grid.addWidget(lbl_start_x, 1, 0, Qt.AlignRight)
    lyt_grid.addWidget(edi_start_x, 1, 1)
    lyt_grid.addWidget(lbl_start_y, 1, 2, Qt.AlignRight)
    lyt_grid.addWidget(edi_start_y, 1, 3)
    lyt_grid.addWidget(lbl_view_w, 2, 0, Qt.AlignRight)
    lyt_grid.addWidget(edi_view_w, 2, 1)
    lyt_grid.addWidget(lbl_view_h, 2, 2, Qt.AlignRight)
    lyt_grid.addWidget(edi_view_h, 2, 3)

    grp_view.setLayout(lyt_grid)

    return grp_view

# Setting the camera id for combobox
def setCameraId(cameraSelect):
    global CameraId
    CameraId = cameraSelect.currentText()

# Setting the camera resolution
def setCameraResolution(e_width, e_height, resolutionSelect):
    global CameraHeight, CameraWidth
    s_height = str(resolutionSelect.currentData().height())
    s_width = str(resolutionSelect.currentData().width())
    CameraHeight = s_height
    CameraWidth = s_width
    e_width.setText(s_width)
    e_height.setText(s_height)

# Setting the display parameters 
def setDisplay(value, field):
    if value == "x":
        global DisplayStartX
        DisplayStartX = int(field.text())
    if value == "y":
        global DisplayStartY
        DisplayStartY = int(field.text())
    if value == "w":
        global DisplayWidth
        DisplayWidth = int(field.text())
    if value == "h":
        global DisplayHeight
        DisplayHeight = int(field.text())

# Setting the Fullscreen parameters
def setFullScreen(checkbox, fields):
    global DisplayFullScreen
    if checkbox.checkState() == 2:
        DisplayFullScreen = True
    else:
        DisplayFullScreen = False
    for field in fields:
        field.setDisabled(DisplayFullScreen)

# Enabling & disabling the buttons when display is is process 
def setButtons(enableButtons, disableButtons):
    for btn in enableButtons:
        btn.setDisabled(False)
    for btn in disableButtons:
        btn.setDisabled(True)

# Setting the crop checkbox 
def setCrop(checkbox, fields):
    state = checkbox.checkState() == 2
    global DisplayCrop
    DisplayCrop = state
    for field in fields:
        field.setDisabled(not state)

# setting the crop values 
def setCropValue(field, value):
    if field == "x":
        global DisplayCropX
        DisplayCropX = value
    if field == "y":
        global DisplayCropY
        DisplayCropY = value
    if field == "w":
        global DisplayCropWidth
        DisplayCropWidth = value
    if field == "h":
        global DisplayCropHeight
        DisplayCropHeight = value

# setting the horizontal transform 
def setDisplayFlipHorizontal(value):
    global DisplayFlipHorizontal
    if(value == 2):
        DisplayFlipHorizontal = 1
    else:
        DisplayFlipHorizontal = 0

# setting the vertical transform 
def setDisplayFlipVertical(value):
    global DisplayFlipVertical
    if(value == 2):
        DisplayFlipVertical = 1
    else:
        DisplayFlipVertical = 0

# setting the rotation 
def setRotate(value):
    global DisplayRotate
    DisplayRotate = value


window()

