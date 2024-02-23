# Import libraries
import Adafruit_PCA9685
import pandas
import numpy

class ThrusterInterfaceAUVDriver:
    def __init__(self,
                 PCA9685_FREQUENCY = 60,
                 PWM_MIN = 1100,
                 PWM_MAX = 1900,
                 STARTING_PLACE_FOR_PWM = 0,
                 SYSTEM_OPERATIONAL_VOLTAGE = 16.0,
        ):
        # Initialice the PCA9685 PCB
        self.PCA9685_Module = Adafruit_PCA9685.PCA9685()
        self.PCA9685_Module.set_pwm_freq(PCA9685_FREQUENCY)

        # Set variables for the object to controll PCA9685 PCB
        self.PWM_MIN = PWM_MIN
        self.PWM_MAX = PWM_MAX
        self.STARTING_PLACE_FOR_PWM = STARTING_PLACE_FOR_PWM

        # Convert SYSTEM_OPERATIONAL_VOLTAGE to a whole even number to work with
        # This is because we have multiple files for the behavious of the thrusters depending on the voltage of the drone
        if (SYSTEM_OPERATIONAL_VOLTAGE < 11.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 10
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 13.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 12
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 15.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 14
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 17.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 16
        elif (SYSTEM_OPERATIONAL_VOLTAGE < 19.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 18
        elif (SYSTEM_OPERATIONAL_VOLTAGE >= 19.0):
            self.SYSTEM_OPERATIONAL_VOLTAGE = 20
            
    def _interpolate_forces_to_pwm(self, thruster_forces_array):
        """
        Takes in Array of forces in Newtosn [N]
        takes 8 floats in form of:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Returns an Array of PWM
        Gives out 8 ints in form of:
        [0, 0, 0, 0, 0, 0, 0, 0]
        """
        # Read the important data from the .csv file
        thrusterDatasheetFileData = pandas.read_csv(
            f"motion/thruster_interface_auv/resource/T200-Thrusters-{self.SYSTEM_OPERATIONAL_VOLTAGE}V.csv",
            usecols = [' PWM (µs)',' Force (Kg f)'])
        
        # Convert Newtons to Kg as Thruster Datasheet is in Kg format
        for i, thruster_forces in enumerate(thruster_forces_array):
            thruster_forces_array[i] = thruster_forces/9.80665

        # Interpolate data
        thrusterDatasheetFileForces = thrusterDatasheetFileData[' Force (Kg f)'].values
        thrusterDatasheetFileDataPWM = thrusterDatasheetFileData[' PWM (µs)'].values
        interpolated_pwm = numpy.interp(thruster_forces_array, thrusterDatasheetFileForces, thrusterDatasheetFileDataPWM)

        # Convert PWM signal to integers as they are interpolated and can have floats
        interpolated_pwm = [int(pwm) for pwm in interpolated_pwm]

        return interpolated_pwm

    def drive_thrusters(self, thruster_forces_array):
        """
        Takes in Array of forces in Newtosn [N]
        takes 8 floats in form of:
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Converts Forces to PWM signals
        PWM signals sent to PCA9685 module through I2C
        PCA9685 Module sends electrical PWM signals to the individual thruster ESCs
        The ESCs send corecponding electrical power to the Thrustres
        Thrusters then generate thrust acordingly to the Forces sent to this driver

        Returns an Array of PWM signal for debugging purposes
        Gives out 8 ints in form of:
        [0, 0, 0, 0, 0, 0, 0, 0]
        """
        # Convert Forces to PWM
        thruster_pwm_array = self._interpolate_forces_to_pwm(thruster_forces_array)

        # Drive thrusters with the specific PWM desired with the help of PCA9685 Module that sends PWM signals to ESCS
        for ESC_channel, ESC_pwm in enumerate(thruster_pwm_array):
            # Clamping pwm signal in case it is out of range
            if (ESC_pwm < self.PWM_MIN): # To small
                ESC_pwm = self.PWM_MIN 
            elif (ESC_pwm > self.PWM_MAX): # To big
                ESC_pwm = self.PWM_MAX

            # Send PWM signal to the PCA9685 PCB through I2C that will then be sent to the individual ESCs and run the thrusters
            self.PCA9685_Module.set_pwm(ESC_channel, self.STARTING_PLACE_FOR_PWM, ESC_pwm)
        
        return thruster_pwm_array
    