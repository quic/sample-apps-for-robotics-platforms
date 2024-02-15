import time

from common.idriver import *
from common.logging import *
from common.drv_types import *

class ChirpDriver(IDriver):
	"""
	Driver implementation for an Chirp logger
	"""
	def __init__(self, log_path, rate):
		super(ChirpDriver, self).__init__(log_path, DriverTypes.Chrip, rate)
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
			print("ERROR unable to create logging directory for chirp")
			return False

		# create command to call chirp app binary
		cmd = f"tdk-chx01-get-data-app -d {int(duration)} "
		cmd += f"-l {str(self.path)} > /dev/null 2>&1" #Can not change rate or tdk-ch101 app produces junk data
		start_time = time.time()

		# if simulation enabled in config file
		if self.simulation == 1:
			self._logger.log_console("simulation mode")
			self._logger.log_console("starting driver for duration: " + str(duration))

			while not super(ChirpDriver,self).duration_ended(start_time, duration):
				# distance - floating point, meters
				self._logger.log_file('0.0')
				time.sleep(self._sleep_sec)
				
			# flush
			self._logger.log_file("", True)
			return True
		
		# running actual logging using command
		else:
			ret = os.system(cmd)
			if (ret != 0):
				return False # unsuccessful
			return True # success 

