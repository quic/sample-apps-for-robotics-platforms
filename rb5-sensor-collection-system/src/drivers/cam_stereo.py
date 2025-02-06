import time
import os
import sys
import json
import cv2
import numpy as np

from common.idriver import *
from common.logging import *
from common.drv_types import *

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import  Gst, GstApp
from gstreamer import GstPipeline, GstVideo
import gstreamer.utils as utils

config_file = "/etc/rb5-scs.config"

class CamStereoDriver(IDriver):
	"""
	Driver implementation for an Tracking Camera logger
	"""
	def __init__(self, log_path, rate):
		super(CamStereoDriver, self).__init__(log_path, DriverTypes.CamStereo, rate)
		self._logger = Logging(log_path, self._drv_type, rate)
		self.cmd = ""
		self.path = self._logger._path + "/"
		self.pipeline = None
		self.camera = None
		self.width = 640
		self.height = 480
		self.rate = rate
		self.save_format = "png"
		self.channels = 1
		self.ts = 0 # timestamp

	def initialize(self):
		"""
		Initialize the driver.  
		:return: True if initialization is succssful, else false
		"""
		config = None
		with open(config_file) as json_file:
			config = json.load(json_file)

		# parsing config
		for sensor in config['sensors']:
			if sensor['name'] == 'cam_stereo':
				if sensor['config_resolution'] == 1:
					self.width = sensor['width']
					self.height = sensor['height']
				self.save_format = sensor['save-format']

		Gst.init(sys.argv)  # init gstreamer

		# iterating through different user save options
		if self.save_format == "png":
			self.cmd = "qtiqmmfsrc camera=1 name=stereo1 !"
			self.cmd += f" video/x-h264,format=NV12,width={self.width},height={self.height},framerate={self.rate}/1 !"
			self.cmd += " queue ! h264parse ! avdec_h264 ! videoconvert !"
			self.cmd += " appsink emit-signals=True"

		elif self.save_format == "mp4":
			self.cmd = "qtiqmmfsrc camera=1 name=stereo1 !"
			self.cmd += f" video/x-h264,format=NV12,width={self.width},height={self.height},framerate={self.rate}/1 !"
			self.cmd += " h264parse ! mp4mux ! queue !"
			self.cmd += " filesink location=" + self.path + str(time.time()) + ".mp4"

		elif self.save_format == "h264":
			self.cmd = "qtiqmmfsrc camera=1 name=stereo1 !"
			self.cmd += f" video/x-h264,format=NV12,width={self.width},height={self.height},framerate={self.rate}/1 !"
			self.cmd += " filesink location=" + self.path + str(time.time()) + ".h264"

		# Initialize GST pipeline
		self.pipeline = Gst.parse_launch(self.cmd)
		#self.pipeline.set_state(Gst.State.READY)

	def start(self, duration):
		"""
		Start running the driver
		To be implemented by child.
		:param duration: length in seconds to log, 0 to log forever
		:return: True if duration ran and exitted with success
		"""
		# Attempt to make log dir
		try:
			os.makedirs(self._logger._path)
		except:
			print("ERROR unable to create logging directory for cam stereo")
			return False

		#if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			start_time = time.time()
			while not super(CamStereoDriver, self).duration_ended(start_time, duration):
				#self._logger.log_file('0.0') # THIS DOES NOT WORK
				time.sleep(self._sleep_sec)
					
			# flush
			self._logger.log_file("", True) # THIS ALSO FAILS
			return True
			
		# running actual capture
		else:
			with GstPipeline(self.cmd) as pipeline:
				#Start pipeline
				self.pipeline.set_state(Gst.State.PLAYING)
				if self.save_format == "png":
					appsink = pipeline.get_by_cls(GstApp.AppSink).pop(0)
					appsink.connect("new-sample", self.on_buffer, None)
			
				start_time = time.time() #Capture starting time
				while not super(CamStereoDriver, self).duration_ended(start_time, duration):
					pass # wait until timer ends to end thread

				self.pipeline.send_event(Gst.Event.new_eos())
				time.sleep(1) # give time for eos cleanup

				self.pipeline.set_state(Gst.State.NULL) # clean pipeline
				time.sleep(1)
				return True

	def on_buffer(self, sink, data):
		sample = sink.emit("pull-sample")  # Gst.Sample
		buffer = sample.get_buffer()  # Gst.Buffer
		self.ts = time.time() # capture timestamp as close to image signal

		caps_format = sample.get_caps().get_structure(0)  # Gst.Structure

		# GstVideo.VideoFormat
		frmt_str = caps_format.get_value('format') 
		video_format = GstVideo.VideoFormat.from_string(frmt_str)

		array = np.ndarray(shape=(self.height, self.width, self.channels), \
		buffer=buffer.extract_dup(0, buffer.get_size()), \
		dtype=utils.get_np_dtype(video_format))

		# remove single dimensions (ex.: grayscale image)
		array = np.squeeze(array)

		timestamp = str(self.ts) + "." + str(self.save_format)
		logging_cp_path = os.path.join(self.path, timestamp)   # complete logging path

		cv2.imwrite(logging_cp_path , array)
		return 0
