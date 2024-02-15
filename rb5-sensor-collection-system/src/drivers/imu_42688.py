import time
import os

from common.idriver import *
from common.logging import *
from common.drv_types import *

class Imu42688Driver(IDriver):
	"""
	Driver implementation for an ICM 42688 logger
	"""
	def __init__(self, log_path, rate):
		super(Imu42688Driver, self).__init__(log_path, DriverTypes.Imu42688, rate)
		self._logger = Logging(log_path, self._drv_type, rate)
		self.path = self._logger._path + "/" + str(int(time.time())) + ".csv" 
		self.rate = rate

	def initialize(self):
		"""
		Initialize the driver.  
		:return: True if initialization is succssful, else false
		"""
		
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
			print("ERROR unable to create logging directory for imu 42688")
			return False

		# setup command for running binary
		cmd = "log_sensor_icm4x6xx -p " + self.path
		cmd += " -t " + str(duration) + " > /dev/null 2>&1"

		# if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			start_time = time.time()
			while not super(Imu42688Driver,self).duration_ended(start_time, duration):
				# accelX - floating point, acceleration X axis
				# accelY - floating point, acceleration Y axis
				# accelZ - floating point, acceleration Z axis
				# gyroX - floating point, gyro X axis
				# gyroY - floating point, gyro Y axis
				# gyroZ - floating point, gyro Z axis
				self._logger.log_file('0.01, 0.02, 0.03, 0.04, 0.05, 0.06')
				time.sleep(self._sleep_sec)
				
			# flush
			self._logger.log_file("", True)
			return True
		
		# running actual logging
		else:
			ret = os.system(cmd)
			if (ret != 0):
				return False # unsuccessful
			return True # success 
