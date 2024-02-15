# Removes debug logging information for matplotlib and gstreamer
import logging
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING)

gst_logger = logging.getLogger('pygst.GstPipeline')
gst_logger.setLevel(logging.WARNING)

import os
import json
from datetime import datetime
import time
import setproctitle

from drivers.cam_ir import CamIrDriver
from drivers.cam_thermal import CamThermalDriver
from drivers.cam_stereo import CamStereoDriver
from drivers.cam_tracking import CamTrackingDriver
from drivers.chirp  import ChirpDriver
from drivers.gps import GpsDriver
from drivers.imu_4622x import Imu4622xDriver
from drivers.imu_42688 import Imu42688Driver
from drivers.mag import MagDriver
from drivers.pressure import PressureDriver
from drivers.temp import TempDriver
from drivers.perf import PerfDriver

from common.time_series import create_plots

DEFAULT_CFG_FILE = "/etc/rb5-scs.config"
LOCAL_CFG_FILE   = "../config/rb5-scs.config"
TIMESTAMP_FORMAT = "%Y-%m-%d-%H:%M:%S"

def get_driver(name, log_path, rate):
	"""
	Factory Method for creating Drivers
	"""
	if name == "cam_ir":
		return CamIrDriver(log_path, rate)
	elif name == "cam_stereo":
		return CamStereoDriver(log_path, rate)
	elif name == "cam_thermal":
		return CamThermalDriver(log_path, rate)
	elif name == "cam_tracking":
		return CamTrackingDriver(log_path, rate)
	elif name == "chirp":
		return ChirpDriver(log_path, rate)
	elif name == "gps":
		return GpsDriver(log_path, rate)
	elif name == "imu_42688":
		return Imu42688Driver(log_path, rate)
	elif name == "imu_4622x":
		return Imu4622xDriver(log_path, rate)
	elif name == "mag":
		return MagDriver(log_path, rate)
	elif name == "pressure":
		return PressureDriver(log_path, rate)
	elif name == "temp":
		return TempDriver(log_path, rate)
	elif name == "perf":
		return PerfDriver(log_path, rate)
	else:
		return None

if __name__ == "__main__":

	# Load config from file
	config_path = ""
	if os.path.exists(DEFAULT_CFG_FILE):
		config_path = DEFAULT_CFG_FILE
	elif os.path.exists(LOCAL_CFG_FILE):
		config_path = LOCAL_CFG_FILE
	else:
		print("[ERROR] missing config file")
		exit(1)
	print("[INFO] using config file at: " + config_path)

	# Parse coonfig from JSON
	config = None
	with open(config_path) as json_file:
		config = json.load(json_file)
	
	#
	# We use the time right now as a session value, and create a new session
	# directory to hold the various logs
	#
	start_time = datetime.now()
	start_time_str = start_time.strftime(TIMESTAMP_FORMAT)
	log_path = os.path.join(config['log_path'], start_time_str)
	
	print("[INFO] starting")
	print("[INFO] log directory at: " + log_path)

	# load drivers for the given config file
	drivers = []
	for sensor in config['sensors']:
		if sensor['enabled'] == 1:
			d = get_driver(sensor['name'], log_path, sensor['rate'])
			d.simulation = config['simulation']
			d.name = sensor['name']
			drivers.append(d)


	# to test out a single driver with debugger, uncomment and use this usage:
	#print("START DEBUG!")
	#perf_driver = PerfDriver(log_path, 100)
	#perf_driver.initialize()
	#perf_driver.simulation = config['simulation']
	#perf_driver.start(10)
	#print("END DEBUG!")
	#exit(0)

	#
	# now, to provide each driver it's own process, we'll fork a new process for each 
	# driver. The parent process dies after forking, but it's child will live on to 
	# initialize and start it's driver and also fork a new process that will do the same


	for d in drivers:

		newpid = os.fork()
		if newpid == 0:
			setproctitle.setproctitle("rb5-scs:" + d.name)
			# we are in a child process now.  Let's start the driver and the start
			# function will be a blocking call so this process stays here until the 
			# driver exits if ever
			d.initialize()
			if d.start(config['log_duration']):
				print("[INFO] process ended with success: " + d.name)
			else:
				print("[ERROR] process ended: " + d.name)
			break

	# Uncomment the lines below in order to enable logging
	#time.sleep(config['log_duration'])	
	#create_plots(log_path, config['log_duration'])
