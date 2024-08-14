#!/usr/bin/env python3

# Libraries for file manipulation
import os
import sys
import ast
import glob

# Libraries for handling data structures
import pandas as pd
import numpy as np
import array

# Libraries for anmation
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt



# Variables for seting upp data structures correctly
hydrophoneDataSize = (2**10) * 3 # 1 hydrophone buffer is 2^10 long, Each hydrophone data has 3 buffers full of this data
DSPDataSize = 2**10 # DSP (Digital Signal Processing) has 2^10 long data
TDOADataSize = 5 # TDOA (Time Difference Of Arrival) has 5 hydrophones it has times for
positionDataSize = 3 # position only has X, Y, Z basicaly 3 elements

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

hydrophoneAxis = [None] * 5

# Add subplots in the first column for hydrophone data
for i in range(5):
    hydrophoneAxis[i] = fig.add_subplot(
        gs_hydrophone[i, 0], sharex=hydrophoneAxis[0] if i else None
    )
    hydrophoneAxis[i].label_outer()
fig.text(0.25, 0.965, "Hydrophone Data", ha="center")

# Add subplots in the second column
FFTAxis = fig.add_subplot(gs_dsp[0])
filterAxis = fig.add_subplot(gs_dsp[1])

# Plot type so that the size is dynamic
plt.tight_layout()

# Select nice color pallet for graphs
colorSoftPurple = (168 / 255, 140 / 255, 220 / 255)
colorSoftBlue = (135 / 255, 206 / 255, 250 / 255)
colorSoftGreen = (122 / 255, 200 / 255, 122 / 255)



# .CSV Setup ==================================================
# Get Directory of the .csv files
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACOUSTICS_CSV_FILE_DIR = PACKAGE_DIR + "/acoustics_data"

# List of all the acoustic files
acousticsCSVFiles = csv_files = glob.glob(ACOUSTICS_CSV_FILE_DIR + "/acoustics_data_" + "*.csv")

# Get the latest csv file name for acoustics data
acousticsCSVFile = max(acousticsCSVFiles, key=os.path.getctime)



def convertPandasObjectToIntArray(pandasObject):
    pandasString = pandasObject.iloc[0].strip("array('i', ").rstrip(')')
    pandasIntArray = [int(x.strip()) for x in pandasString.strip('[]').split(',')]

    return pandasIntArray

def convertPandasObjectToFloatArray(pandasObject):
    pandasString = pandasObject.iloc[0].strip("array('f', ").rstrip(')')
    pandasFloatArray = [float(x.strip()) for x in pandasString.strip('[]').split(',')]

    return pandasFloatArray



def getAcousticsData():
    # Variables that will be filled with latest acoustics data ----------
    hydrophone1 = [0] * hydrophoneDataSize
    hydrophone2 = [0] * hydrophoneDataSize
    hydrophone3 = [0] * hydrophoneDataSize
    hydrophone4 = [0] * hydrophoneDataSize
    hydrophone5 = [0] * hydrophoneDataSize

    unfilteredData = [0] * DSPDataSize
    filteredData = [0] * DSPDataSize
    FFTData = [0] * DSPDataSize
    peaksData = [0] * DSPDataSize
    FFTAmplitudeData = [0] * DSPDataSize
    FFTFrequencyData = [0] * DSPDataSize
    peaksAmplitudeData = [0] * DSPDataSize
    peaksFrequencyData = [0] * DSPDataSize

    tdoaData = [0.0] * TDOADataSize
    positonData = [0.0] * positionDataSize

    # Read latest acoustics data ----------
    acousticsDataFrame = pd.read_csv(acousticsCSVFile)
    latestAcousticsData = acousticsDataFrame.tail(1)

    try:
        # Get latest hydrophone data
        hydrophone1 = convertPandasObjectToIntArray(latestAcousticsData["Hydrophone1"])
        hydrophone2 = convertPandasObjectToIntArray(latestAcousticsData["Hydrophone2"])
        hydrophone3 = convertPandasObjectToIntArray(latestAcousticsData["Hydrophone3"])
        hydrophone4 = convertPandasObjectToIntArray(latestAcousticsData["Hydrophone4"])
        hydrophone5 = convertPandasObjectToIntArray(latestAcousticsData["Hydrophone5"])
        
        # Unfiltered data is special as it is the same as Hydrohone 1 first 1024 values
        # This is because Acoustics PCB uses Hydrophone 1 to perform DSP
        # Hydrohones have a ring buffer the size of 3 buffers each containing 1024 values (2^10)
        # We always use the first ring buffer of Hydrophone 1 to performe DSP
        # That is why unfiltered data is the same as Hydrphne 1 first buffer
        unfilteredData = hydrophone1[0:1024]

        # Get DSP data
        filteredData = convertPandasObjectToIntArray(latestAcousticsData["FilterResponse"]) # Also known as Filter response to the raw unfiltered data
        FFTData = convertPandasObjectToIntArray(latestAcousticsData["FFT"])
        peaksData = convertPandasObjectToIntArray(latestAcousticsData["Peaks"])

        # Get multilateration data
        tdoaData = convertPandasObjectToFloatArray(latestAcousticsData["TDOA"])
        positonData = convertPandasObjectToFloatArray(latestAcousticsData["Position"])
    except:
        print("ERROR: Coulden't read acoustics data")

    # Post process DSP data to desired scale and amount ----------
    # 1. Convert FFTData to its corresponding frequency amount
    # 2. Cut out big FFT frequencies out as they are not relevant
    # 3. Cut out big peak frequencies as they are not relevant 
    sampleLength = len(FFTData)
    maxFrequencyIndex = int(MAX_FREQUENCY_TO_SHOW * sampleLength / SAMPLE_RATE)

    FFTAmplitudeData = FFTData[0:maxFrequencyIndex]
    FFTFrequencyData = [(i * (SAMPLE_RATE / sampleLength)) for i in range(sampleLength)]
    FFTFrequencyData = FFTFrequencyData[0:maxFrequencyIndex]

    # Peaks data is special as each peak data value is a array of [Amplitude, Frequency, Phase] of the peak
    # We want to get amplitude and frequency, dont really care about the phase
    try:
        tempAmplitude = []
        tempFrequency = []
        for i in range(1, len(peaksData), 3):
            if peaksData[i] < MAX_FREQUENCY_TO_SHOW:
                tempAmplitude += [peaksData[i - 1]]
                tempFrequency += [peaksData[i]]

        peaksAmplitudeData = tempAmplitude
        peaksFrequencyData = tempFrequency
    except:
        print("ERROR processing DSP data")

    # return processed data ----------
    return [
        hydrophone1,
        hydrophone2,
        hydrophone3,
        hydrophone4,
        hydrophone5,

        unfilteredData,

        filteredData,
        FFTAmplitudeData,
        FFTFrequencyData,
        peaksAmplitudeData,
        peaksFrequencyData,

        tdoaData,
        positonData,
        ]



def display_live_data(frame):
    # Get latest acoustics data
    acousticsData = getAcousticsData()

    # Set the lates acoustics data in apropriate variables
    hydrophoneData = [
        acousticsData[0], # Hydrophone 1
        acousticsData[1], # Hydrophone 2
        acousticsData[2], # Hydrophone 3
        acousticsData[3], # Hydrophone 4
        acousticsData[4], # Hydrophone 5
        ]

    unfilteredData = acousticsData[5]

    filterData = acousticsData[6]
    FFTAmplitudeData = acousticsData[7]
    FFTFrequencyData = acousticsData[8]
    peaksAmplitudeData = acousticsData[9]
    peaksFrequencyData = acousticsData[10]

    tdoaData = acousticsData[11] # Currently not in use
    positionData = acousticsData[12] # Currently not in use

    # Plot hydrophone data
    for i in range(5):
        xHydrophone = list(range(len(hydrophoneData[i][::])))
        hydrophoneAxis[i].clear()
        hydrophoneAxis[i].plot(
            xHydrophone,
            hydrophoneData[i],
            label=f"Hydrophone {i + 1}",
            color=colorSoftBlue,
            alpha=1,
        )
        hydrophoneAxis[i].legend(loc="upper right", fontsize="xx-small")

    # Plot Filter response
    xRaw = list(range(len(unfilteredData)))
    xFilter = list(range(len(filterData)))
    filterAxis.clear()
    filterAxis.set_title("Filter response")
    filterAxis.plot(xRaw, unfilteredData, label="Raw", color=colorSoftBlue, alpha=0.5)
    filterAxis.plot(
        xFilter, filterData, label="Filter", color=colorSoftGreen, alpha=0.7
    )
    filterAxis.legend(loc="upper right", fontsize="xx-small")

    # Plot FFT data
    FFTAxis.clear()
    FFTAxis.set_title("FFT")
    FFTAxis.set_xlabel("Frequency [Hz]")
    FFTAxis.set_ylabel("Amplitude")
    FFTAxis.bar(
        FFTFrequencyData,
        FFTAmplitudeData,
        label="FFT",
        color=colorSoftPurple,
        alpha=1,
        width=500,
    )
    FFTAxis.scatter(
        peaksFrequencyData,
        peaksAmplitudeData,
        label="Peaks",
        color="red",
        alpha=0.7,
        s=30,
        linewidths=1.4,
        marker="x",
    )
    FFTAxis.legend(loc="upper right", fontsize="xx-small")

    # Print out the unused Multilateration data
    print(f"TDOA Data: {tdoaData}     |     Position Data: {positionData}")

# Plotting live data
ani = animation.FuncAnimation(fig, display_live_data, interval=1000/FPS)
plt.show()

