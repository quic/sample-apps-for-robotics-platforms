from io import TextIOWrapper,BufferedRWPair
import serial
import pynmea2

import os
import time

from common.idriver import *
from common.logging import *
from common.drv_types import *

class GpsDriver(IDriver):
	"""
	Driver implementation for an GPS logger
	"""
	def __init__(self, log_path, rate):
		super(GpsDriver, self).__init__(log_path, DriverTypes.Gps, rate)
		self._logger = Logging(log_path, self._drv_type, rate)
		self.rate = rate
		self.path = self._logger._path + "/" + str(int(time.time())) + ".csv"

	def initialize(self):
		"""
		Initialize the driver.
		:return: True if initialization is succssful, else false
		"""
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
		else:
			self.ser = serial.Serial('/dev/ttyMSM0', 38400, timeout=5.0)
			self.sio = TextIOWrapper(BufferedRWPair(self.ser,self.ser))

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
			print("ERROR unable to create logging directory for gps")
			return False

		# if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			start_time = time.time()
			while not super(GpsDriver,self).duration_ended(start_time, duration):
				# lat - floating point, latitude
				# long - floating point, longitude
				# alt - floating point, altitude in meters
				self._logger.log_file('32.7157, 17.1611, 33.3')
				time.sleep(self._sleep_sec)

			# flush
			self._logger.log_file("", True)
			return True
		
		# not simulation, running actual logging
		else:
			# Initialize CSV file for logging
			logfile = open(self.path,"a+")
			logfile.write("Timestamp,Latitude,Longitute,Speed Over Ground\n")
			
			start_time = time.time()
			while not super(GpsDriver,self).duration_ended(start_time, duration):
				# attempt to make a read
				try:
					line = self.sio.readline()
					cur_time = time.time() # capture timestamp as soon as line read
					msg = pynmea2.parse(line) # parse data

					if msg.sentence_type == 'RMC':
						if msg.lat is not None:
							lat = str(float(msg.lat)/100)
						else:
							lat = "NOLAT"
						if msg.lon is not None:
							lon = str(float(msg.lon)/100)
						else:
							lon = "NOLON"

						# structure output for writing
						output = str(cur_time) + "," + lat + " " + msg.lat_dir + "," + lon + " " + msg.lon_dir + "," + str(msg.spd_over_grnd) + "\n"
						logfile.write(output)

				# common exceptions
				except serial.SerialException as e:
					continue # serial exceptions are common and unpredictable- we just ignore them

				except pynmea2.ParseError as e:
					continue # parse errors too

				# sleep for rate
				time.sleep(self.rate/1000)

			# close logfile properly and exit thread
			logfile.close()
			return True
