import os
from datetime import datetime
from pathlib import Path

from common.drv_types import *

class Logging:

	def __init__(self, path, drv_type, rate):
		self._log_file_name = "log"
		self._drv_type = drv_type

		self._buffer = []
		if rate is 0:
			self._buffer_length = 32
		else:
			self._buffer_length = rate
		
		self._drv_name = DriverTypes.get_name(drv_type)
		self._header = "rb5-scs: " + self._drv_name
		
		self._root = path + "/" + self._drv_name
		self._path = self._root + "/" + self._log_file_name

	def log_console(self, message):
		"""
		:param message: string message to log
		"""
		date_time = datetime.now().strftime("%Y-%b-%d-%H:%M:%S.%f")
		header = date_time + ' ' + self._header.ljust(30)
		print(header + message)

	def log_file(self, message, flush=False, timestamp=None):
		"""
		:param message: string message to log
		"""
		try:

			if not os.path.exists(self._root):
				Path(self._root).mkdir(parents=True, exist_ok=True)
				with open(self._path, 'w'): pass

			if timestamp:
				date_time = str(timestamp)
			else:
				date_time = datetime.now().strftime("%Y-%b-%dT%H:%M:%S.%f")
			log_message = date_time + ', ' + str(self._drv_type.value) + ', ' + message

			self._buffer.append(log_message)
			
			if flush or len(self._buffer) > self._buffer_length - 1:
				with open(self._path, 'a') as f:
					for m in self._buffer:
						f.write(m + '\n')
				self._buffer.clear()
		except Exception as e:
			print(e)
			print("[ERROR] " + self._header)
