import time

from common.logging import *
from common.drv_types import *

class IDriver:
	"""
	Interface" for a  class inmplementing a Driver
	"""
	simulation = False
	name =  ""

	def __init__(self, log_path, drv_type, rate):
		"""
		Creates a DeviceConfig , will fail if exists already
		:param log_path: string path of the log file to write to
		:param drv_type: enum type of driver
		:param drv_type: number representing the sensor update rate requested (e.g. 100 for 100Hz)
		:return: True if created OK
        """
		self._log_path  = log_path
		self._drv_type = drv_type
		self._rate = rate

		# rate is sameples per second, we want a millisecond interval
		if(self._rate > 0):
			self._sleep_sec = (1/rate)
		else:
			self._sleep_sec = 0
	
	def initialize(self):
		"""
		Initialize the driver.  
		To be implemented by child.
		:return: True if initialization is succssful, else false
		"""
		pass

	def start(self, duration):
		"""
		Start runningthe driver
		To be implemented by child.
		:param duration: length in seconds to log, 0 to log forever
		:return: True if duration ran and exitted with success
		"""
		pass

	def duration_ended(self, start_time, duration):
		"""
		Start runningthe driver
		To be implemented by child.
		:param start_time: start time of process
		:param duration: how long in seconds the process should run
		:return: True if duration has expired
		"""
		current_time = time.time()
		elapsed_time = current_time - start_time
		if duration > 0 and elapsed_time > duration:
			return True
		else:
			return False

