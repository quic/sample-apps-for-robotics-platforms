import time

from common.idriver import *
from common.logging import *
from common.drv_types import *

class Imu4622xDriver(IDriver):
	"""
	Driver implementation for an ICM 4622x logger
	"""
	def __init__(self, log_path, rate):
		super(Imu4622xDriver, self).__init__(log_path, DriverTypes.Imu4622x, rate)
		self._logger = Logging(log_path, self._drv_type, rate)

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
		start_time = time.time()

		# if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			while not super(Imu4622xDriver,self).duration_ended(start_time, duration):
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
		else:
			return True
