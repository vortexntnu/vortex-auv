from PID import PIDRegulator
from backstepping import BacksteppingDesign, BacksteppingControl

class AutopilotPID:

	def __init__(self):
		# PIDRegulator(p, i, d, sat)
		self.pid = PIDRegulator(25, 0.024, 3.5, 5.0)

	def updateGains(self, p, i, d, sat):

		self.pid.p = p
		self.pid.i = i
		self.pid.d = d
		self.pid.sat = sat

	def headingController(self, psi_d, psi, t):

		# error ENU
		e_rot = psi_d - psi

		# regulate(err, t)
		tau = self.pid.regulate(e_rot, t)

		return tau

	def depthController(self, z_d, z, t):

		e = z_d - z;

		tau = self.pid.regulate(e, t)

		return tau


class AutopilotBackstepping:

	def __init__(self):
												# 0.75, 30, 12, 2.5
		self.backstepping = BacksteppingControl(3.75, 45.0, 28.0, 10.5)

	def updateGains(self, c, k1, k2, k3):

		pass
