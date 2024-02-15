from common.idriver import *
from common.logging import Logging
from common.drv_types import *

import cv2
import time
import os
import json

config_file = "/etc/rb5-scs.config"

class CamIrDriver(IDriver):
	"""
	Driver implementation for an IR Camera logger
	"""
	def __init__(self, log_path, rate):
		super(CamIrDriver, self).__init__(log_path, DriverTypes.CamIr, rate)
		self._logger = Logging(log_path, self._drv_type, rate)
		self.rate = rate
		self.width = 640
		self.height = 480
		self.config_resolution = False
		self.video_num = 2
		self.save_format = "png"

	def initialize(self):
		"""
		Initialize the driver.  
		:return: True if initialization is succssful, else false
		"""
		config = None
		with open(config_file) as json_file:
			config = json.load(json_file)

		#parsing config file for user requests
		for sensor in config['sensors']:
			if sensor['name'] == 'cam_ir':
				if sensor['config_resolution'] == 1:
					self.config_resolution = True
					self.width = sensor['width']
					self.height = sensor['height']
				self.save_format = sensor['save-format']
				if sensor['priority'] == 1:
					self.video_num = 0
				

	def start(self, duration):
		"""
		Start runningthe driver
		To be implemented by child.
		:param duration: length in seconds to log, 0 to log forever
		:return: True if duration ran and exitted with success
		"""
		# Attempt to make log dir
		try:
			os.makedirs(self._logger._path)
		except:
			print("ERROR unable to create logging directory for camIR")
			return False

		# if simulation enabled in config file
		if self.simulation:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			start_time = time.time()
			while not super(CamIrDriver,self).duration_ended(start_time, duration):
				self._logger.log_file('0.0')
				time.sleep(self._sleep_sec)	
			# flush
			self._logger.log_file("", True)
			return True
		
		# Not in simulation mode, running actual capture
		else:
			dim = (self.width, self.height)
            
            # attempt to open camera by ID
			try:
				camera = cv2.VideoCapture(self.video_num)
			except:
				self._logger.log_console("[Error]: UVC camera not detected")
				return False
			
			#Initialize time values
			start_time = time.time()
			while not super(CamIrDriver,self).duration_ended(start_time, duration):
				if not camera.isOpened():
					self._logger.log_console("[Error]: UVC camera not detected")
					camera.release()
					return False
				
				# Access camera frames
				_, frame = camera.read()
				if frame is None:
					#self._logger.log_console("[Error]: no image data")
					camera.release()
					return False
				
				# change camera resolution
				if self.config_resolution:
					frame = cv2.resize(frame, dim)
				
				# Logging results
				cur_time = time.time()
				path = self._logger._path + "/" + str(cur_time) + "." + self.save_format
				cv2.imwrite(path, frame)

				# Sleep for rate provided
				time.sleep(1/self.rate)

			camera.release()
			return True
			
