# Python Libraries
import time
import csv
from datetime import datetime


class AcousticsDataRecordLib:
    def __init__(self, ROS2_PACKAGE_DIRECTORY=""):
        # Global variables for .csv file manipulation ----------
        # Get the path for the directory where we will store our data
        self.acoustics_data_directory = ROS2_PACKAGE_DIRECTORY + "acoustics_data/"

        timestamp = time.strftime("%Y-%m-%d_%H:%M:%S")
        data_file_name = "acoustics_data_" + timestamp + ".csv"
        self.data_file_location = self.acoustics_data_directory + data_file_name

        self.csv_headers = [
            "Time",
            "Hydrophone1",
            "Hydrophone2",
            "Hydrophone3",
            "Hydrophone4",
            "Hydrophone5",
            "FilterResponse",
            "FFT",
            "Peaks",
            "TDOA",
            "Position",
        ]

        # Make new .csv file for loging blackbox data ----------
        with open(self.data_file_location, mode="w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(self.csv_headers)

    # Methods for external uses ----------
    def log_data_to_csv_file(
        self,
        hydrophone1=[0],
        hydrophone2=[0],
        hydrophone3=[0],
        hydrophone4=[0],
        hydrophone5=[0],
        filter_response=[0],
        fft=[0],
        peaks=[0],
        tdoa=[0.0],
        position=[0.0],
    ):
        """
        Logs the provided data to a CSV file.

        Parameters:
        self (object): The instance of the class.
        hydrophone1 (list, optional): Data from hydrophone 1. Defaults to [0].
        hydrophone2 (list, optional): Data from hydrophone 2. Defaults to [0].
        hydrophone3 (list, optional): Data from hydrophone 3. Defaults to [0].
        hydrophone4 (list, optional): Data from hydrophone 4. Defaults to [0].
        hydrophone5 (list, optional): Data from hydrophone 5. Defaults to [0].
        filter_response (list, optional): Filter response data. Defaults to [0].
        fft (list, optional): FFT data. Defaults to [0].
        peaks (list, optional): Peaks data. Defaults to [0].
        tdoa (list, optional): Time Difference of Arrival data. Defaults to [0.0].
        position (list, optional): Position data. Defaults to [0.0].

        Writes the current time and provided data to a CSV file located at self.data_file_location.
        """
        # Get current time in hours, minutes, seconds and miliseconds
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Write to .csv file
        with open(self.data_file_location, mode="a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    current_time,
                    hydrophone1,
                    hydrophone2,
                    hydrophone3,
                    hydrophone4,
                    hydrophone5,
                    filter_response,
                    fft,
                    peaks,
                    tdoa,
                    position,
                ]
            )
