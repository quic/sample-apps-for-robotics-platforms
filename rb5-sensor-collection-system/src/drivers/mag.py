
import time

from common.idriver import *
from common.logging import *
from common.drv_types import *

class MagDriver(IDriver):
	"""
	Driver implementation for an Magnetometer logger
	"""
	def __init__(self, log_path, rate):
		super(MagDriver, self).__init__(log_path, DriverTypes.Mag, rate)
		self._logger = Logging(log_path, self._drv_type, rate)
		self.path = self._logger._path + "/" + str(int(time.time())) + ".csv" 

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
			print("ERROR unable to create logging directory for magnetometer")
			return False

		# prepare command for running binary
		cmd = "read_magnetometer -p " + self.path
		cmd += " -t " + str(duration)

		# if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			start_time = time.time()
			while not super(MagDriver,self).duration_ended(start_time, duration):
				# magX - floating point, millitesla, mT
				# magY - floating point, millitesla, mT
				# magZ - floating point, millitesla, mT
				self._logger.log_file('0.00, 0.01, 0.02')
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
