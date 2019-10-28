from PID import PIDRegulator
from backstepping import BacksteppingDesign, BacksteppingControl

class AutopilotPID:

	def __init__(self):
		# PIDRegulator(p, i, d, sat)
		self.pid_heading = PIDRegulator(5, 1.0, 0.0, 1.0)

	def updateGains(self, p, i, d, sat):

		self.pid_heading.p = p
		self.pid_heading.i = i
		self.pid_heading.d = d
		self.pid_heading.sat = sat

	def headingController(self, psi_d, psi, t):

		# error ENU
		e_rot = psi_d - psi

		# regulate(err, t)
		tau = self.pid_heading.regulate(e_rot, t)

		return tau


class AutopilotBackstepping:

	def __init__(self):
		
		self.backstepping = BacksteppingControl(0.75, 30.0, 12.0, 2.5)
		#self.backstepping = BacksteppingControl(1.25, 40, 10, 4.0)

	def updateGains(self, p, i, d, sat):

		pass
