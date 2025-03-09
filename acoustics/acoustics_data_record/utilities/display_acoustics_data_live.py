#!/usr/bin/env python3

# Libraries for file manipulation
import glob
import os

# Libraries for anmation
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt

# Libraries for handling data structures
import pandas as pd

# Variables for setting upp data structures correctly
hydrophone_data_size = (
    (2**10) * 3
)  # 1 hydrophone buffer is 2^10 long, Each hydrophone data has 3 buffers full of this data
dsp_data_size = 2**10  # DSP (Digital Signal Processing) has 2^10 long data
tdoa_data_size = (
    5  # TDOA (Time Difference Of Arrival) has 5 hydrophones it has times for
)
position_data_size = 3  # position only has X, Y, Z basically 3 elements

# Important variables for later processing of data
SAMPLE_RATE = 430_000  # 430 kHz
MAX_FREQUENCY_TO_SHOW = 60_000  # 60 kHz
FPS = 1


# Make a good plot layout ==================================================
fig = plt.figure()
# Create an outer GridSpec for the two columns
outer_gs = gridspec.GridSpec(1, 2, figure=fig, width_ratios=[1, 1])
# Create an inner GridSpec for the first column
gs_hydrophone = gridspec.GridSpecFromSubplotSpec(
    5, 1, subplot_spec=outer_gs[0], hspace=0.1
)
# Create an inner GridSpec for the second column, with height ratios for the 70%/30% split
gs_dsp = gridspec.GridSpecFromSubplotSpec(
    2, 1, subplot_spec=outer_gs[1], height_ratios=[7, 3], hspace=0.3
)

hydrophone_axis = [None] * 5

# Add subplots in the first column for hydrophone data
for i in range(5):
    hydrophone_axis[i] = fig.add_subplot(
        gs_hydrophone[i, 0], sharex=hydrophone_axis[0] if i else None
    )
    hydrophone_axis[i].label_outer()
fig.text(0.25, 0.965, "Hydrophone Data", ha="center")

# Add subplots in the second column
fft_axis = fig.add_subplot(gs_dsp[0])
filter_axis = fig.add_subplot(gs_dsp[1])

# Plot type so that the size is dynamic
plt.tight_layout()

# Select nice color pallet for graphs
color_soft_purple = (168 / 255, 140 / 255, 220 / 255)
color_soft_blue = (135 / 255, 206 / 255, 250 / 255)
color_soft_green = (122 / 255, 200 / 255, 122 / 255)


# .CSV Setup ==================================================
# Get Directory of the .csv files
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACOUSTICS_CSV_FILE_DIR = PACKAGE_DIR + "/acoustics_data"

# List of all the acoustic files
acoustics_csv_files = csv_files = glob.glob(
    ACOUSTICS_CSV_FILE_DIR + "/acoustics_data_" + "*.csv"
)

# Get the latest csv file name for acoustics data
acoustics_csv_file = max(acoustics_csv_files, key=os.path.getctime)


def convert_pandas_object_to_int_array(pandas_object):
    pandas_string = pandas_object.iloc[0]
    pandas_string = pandas_string.removeprefix("array('i', ").removesuffix(")")
    pandas_int_array = [int(x.strip()) for x in pandas_string.strip('[]').split(',')]
    return pandas_int_array


def convert_pandas_object_to_float_array(pandas_object):
    pandas_string = pandas_object.iloc[0]
    pandas_string = pandas_string.removeprefix("array('f', ").removesuffix(")")
    pandas_float_array = [
        float(x.strip()) for x in pandas_string.strip('[]').split(',')
    ]
    return pandas_float_array


def get_acoustics_data():
    # Variables that will be filled with latest acoustics data ----------
    hydrophone1 = [0] * hydrophone_data_size
    hydrophone2 = [0] * hydrophone_data_size
    hydrophone3 = [0] * hydrophone_data_size
    hydrophone4 = [0] * hydrophone_data_size
    hydrophone5 = [0] * hydrophone_data_size

    unfiltered_data = [0] * dsp_data_size
    filtered_data = [0] * dsp_data_size
    fft_data = [0] * dsp_data_size
    peaks_data = [0] * dsp_data_size
    fft_amplitude_data = [0] * dsp_data_size
    fft_frequency_data = [0] * dsp_data_size
    peaks_amplitude_data = [0] * dsp_data_size
    peaks_frequency_data = [0] * dsp_data_size

    tdoa_data = [0.0] * tdoa_data_size
    position_data = [0.0] * position_data_size

    # Read latest acoustics data ----------
    acoustics_dataframe = pd.read_csv(acoustics_csv_file)
    latest_acoustics_data = acoustics_dataframe.tail(1)

    try:
        # Get latest hydrophone data
        hydrophone1 = convert_pandas_object_to_int_array(
            latest_acoustics_data["Hydrophone1"]
        )
        hydrophone2 = convert_pandas_object_to_int_array(
            latest_acoustics_data["Hydrophone2"]
        )
        hydrophone3 = convert_pandas_object_to_int_array(
            latest_acoustics_data["Hydrophone3"]
        )
        hydrophone4 = convert_pandas_object_to_int_array(
            latest_acoustics_data["Hydrophone4"]
        )
        hydrophone5 = convert_pandas_object_to_int_array(
            latest_acoustics_data["Hydrophone5"]
        )

        # Unfiltered data is special as it is the same as Hydrophone 1 first 1024 values
        # This is because Acoustics PCB uses Hydrophone 1 to perform DSP
        # Hydrophones have a ring buffer the size of 3 buffers each containing 1024 values (2^10)
        # We always use the first ring buffer of Hydrophone 1 to perform DSP
        # That is why unfiltered data is the same as Hydrophone 1 first buffer
        unfiltered_data = hydrophone1[0:1024]

        # Get DSP data
        filtered_data = convert_pandas_object_to_int_array(
            latest_acoustics_data["FilterResponse"]
        )  # Also known as Filter response to the raw unfiltered data
        fft_data = convert_pandas_object_to_int_array(latest_acoustics_data["FFT"])
        peaks_data = convert_pandas_object_to_int_array(latest_acoustics_data["Peaks"])

        # Get multilateration data
        tdoa_data = convert_pandas_object_to_float_array(latest_acoustics_data["TDOA"])
        position_data = convert_pandas_object_to_float_array(
            latest_acoustics_data["Position"]
        )
    except:
        print("ERROR: Couldn't read acoustics data")

    # Post process DSP data to desired scale and amount ----------
    # 1. Convert fft_data to its corresponding frequency amount
    # 2. Cut out big FFT frequencies out as they are not relevant
    # 3. Cut out big peak frequencies as they are not relevant
    sample_length = len(fft_data)
    max_frequency_index = int(MAX_FREQUENCY_TO_SHOW * sample_length / SAMPLE_RATE)

    fft_amplitude_data = fft_data[0:max_frequency_index]
    fft_frequency_data = [
        (i * (SAMPLE_RATE / sample_length)) for i in range(sample_length)
    ]
    fft_frequency_data = fft_frequency_data[0:max_frequency_index]

    # Peaks data is special as each peak data value is a array of [Amplitude, Frequency, Phase] of the peak
    # We want to get amplitude and frequency, dont really care about the phase
    try:
        temp_amplitude = []
        temp_frequency = []
        for i in range(1, len(peaks_data), 3):
            if peaks_data[i] < MAX_FREQUENCY_TO_SHOW:
                temp_amplitude += [peaks_data[i - 1]]
                temp_frequency += [peaks_data[i]]

        peaks_amplitude_data = temp_amplitude
        peaks_frequency_data = temp_frequency
    except:
        print("ERROR processing DSP data")

    # return processed data ----------
    return [
        hydrophone1,
        hydrophone2,
        hydrophone3,
        hydrophone4,
        hydrophone5,
        unfiltered_data,
        filtered_data,
        fft_amplitude_data,
        fft_frequency_data,
        peaks_amplitude_data,
        peaks_frequency_data,
        tdoa_data,
        position_data,
    ]


def display_live_data(frame):
    # Get latest acoustics data
    acoustics_data = get_acoustics_data()

    # Set the latest acoustics data in appropriate variables
    hydrophone_data = [
        acoustics_data[0],  # Hydrophone 1
        acoustics_data[1],  # Hydrophone 2
        acoustics_data[2],  # Hydrophone 3
        acoustics_data[3],  # Hydrophone 4
        acoustics_data[4],  # Hydrophone 5
    ]

    unfiltered_data = acoustics_data[5]

    filter_data = acoustics_data[6]
    fft_amplitude_data = acoustics_data[7]
    fft_frequency_data = acoustics_data[8]
    peaks_amplitude_data = acoustics_data[9]
    peaks_frequency_data = acoustics_data[10]

    tdoa_data = acoustics_data[11]  # Currently not in use
    position_data = acoustics_data[12]  # Currently not in use

    # Plot hydrophone data
    for i in range(5):
        x_hydrophone = list(range(len(hydrophone_data[i][::])))
        hydrophone_axis[i].clear()
        hydrophone_axis[i].plot(
            x_hydrophone,
            hydrophone_data[i],
            label=f"Hydrophone {i + 1}",
            color=color_soft_blue,
            alpha=1,
        )
        hydrophone_axis[i].legend(loc="upper right", fontsize="xx-small")

    # Plot Filter response
    x_raw = list(range(len(unfiltered_data)))
    x_filter = list(range(len(filter_data)))
    filter_axis.clear()
    filter_axis.set_title("Filter response")
    filter_axis.plot(
        x_raw, unfiltered_data, label="Raw", color=color_soft_blue, alpha=0.5
    )
    filter_axis.plot(
        x_filter, filter_data, label="Filter", color=color_soft_green, alpha=0.7
    )
    filter_axis.legend(loc="upper right", fontsize="xx-small")

    # Plot FFT data
    fft_axis.clear()
    fft_axis.set_title("FFT")
    fft_axis.set_xlabel("Frequency [Hz]")
    fft_axis.set_ylabel("Amplitude")
    fft_axis.bar(
        fft_frequency_data,
        fft_amplitude_data,
        label="FFT",
        color=color_soft_purple,
        alpha=1,
        width=500,
    )
    fft_axis.scatter(
        peaks_frequency_data,
        peaks_amplitude_data,
        label="Peaks",
        color="red",
        alpha=0.7,
        s=30,
        linewidths=1.4,
        marker="x",
    )
    fft_axis.legend(loc="upper right", fontsize="xx-small")

    # Print out the unused Multilateration data
    print(f"TDOA Data: {tdoa_data}     |     Position Data: {position_data}")


# Plotting live data
ani = animation.FuncAnimation(fig, display_live_data, interval=1000 / FPS)
plt.show()
