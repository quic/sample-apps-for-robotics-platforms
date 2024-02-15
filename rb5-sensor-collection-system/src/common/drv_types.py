from enum import Enum

class DriverTypes(Enum):
	"""
	Enumeration representing possible actions for bunker to handle
	"""
	CamIr       = 0
	CamThermal  = 1
	CamStereo   = 2
	CamTracking = 3
	Chrip       = 4
	Gps         = 6
	Imu42688    = 7
	Imu4622x    = 8
	Mag         = 9
	Pressure    = 10
	Temp        = 11
	Perf        = 100

	@staticmethod
	def get_name(drv_type):
		"""
		Gets friendly name
		:param drv_type: driver type
		:return string name of driver
		"""
		if drv_type is DriverTypes.CamIr:
			drv_name = "cam_ir"
		elif drv_type is DriverTypes.CamThermal:
			drv_name = "cam_therm"
		elif drv_type is DriverTypes.CamStereo:
			drv_name = "cam_stereo"
		elif drv_type is DriverTypes.CamTracking:
			drv_name = "cam_tracking"
		elif drv_type is DriverTypes.Chrip:
			drv_name = "chirp"
		elif drv_type is DriverTypes.Gps:
			drv_name = "gps"
		elif drv_type is DriverTypes.Imu4622x:
			drv_name = "imu_4622x"
		elif drv_type is DriverTypes.Imu42688:
			drv_name = "imu_42688"
		elif drv_type is DriverTypes.Mag:
			drv_name = "mag"
		elif drv_type is DriverTypes.Pressure:
			drv_name = "pressure"
		elif drv_type is DriverTypes.Temp:
			drv_name = "temp"
		elif drv_type is DriverTypes.Perf:
			drv_name = "perf"

		return drv_name