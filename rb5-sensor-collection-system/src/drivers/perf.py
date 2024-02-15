
import os
import time
import subprocess

from common.idriver import *
from common.logging import *
from common.drv_types import *

class PerfDriver(IDriver):
	"""
	Driver implementation for an Performance logger
	"""
	def __init__(self, log_path, rate):
		super(PerfDriver, self).__init__(log_path, DriverTypes.Perf, rate)
		self._logger = Logging(log_path, self._drv_type, rate)

	def initialize(self):
		"""
		Initialize the driver.  
		:return: True if initialization is succssful, else false
		"""
		pass

	def start(self, duration):
		"""
		Start running the driver
		:param duration: length in seconds to log, 0 to log forever
		:return: True if duration ran and exitted with success
		"""
		start_time = time.time()

		# if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			while not super(PerfDriver,self).duration_ended(start_time, duration):
				# cpu_usage - float,  percentage
				self._logger.log_file('99.99')
				time.sleep(self._sleep_sec)
				
			# flush
			self._logger.log_file("", True)
			return True
		else:
			try:
				#self._logger.log_console("starting driver for duration: " + str(duration))

				while not super(PerfDriver,self).duration_ended(start_time, duration):
					csv_data=str(round(float(os.popen('''grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage }' ''').readline()),2))
					cur_time = time.time()
					self._logger.log_file(csv_data, timestamp=cur_time)
				
				# flush
				self._logger.log_file("", True)
				return True

			except:
				print("[ERROR] in " + self.name)
				return False
