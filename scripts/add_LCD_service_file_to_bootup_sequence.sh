#!/bin/bash

# Define variables for future use
SERVICE_FILE_NAME="LCD.service"
SERVICE_FILE_PATH="../mission/LCD/startup_scripts/"
SYSTEMD_PATH="/etc/systemd/system/"
PYTHON_VERSION="python3.10"



# Get scripts directory and go there
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
cd $SCRIPT_DIR

# Copy the .service file to current directory
cp $SERVICE_FILE_PATH$SERVICE_FILE_NAME $SERVICE_FILE_NAME

# Replace placeholders in the .service file
# Note: This assumes <pathToThisFile> is to be replaced with the current directory
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH|g" $SERVICE_FILE_NAME
# Place in the correct python dependency for the scripts that service file executes
sed -i "s|<PythonVersion>|$PYTHON_VERSION|g" $SERVICE_FILE_NAME

# Kill the systems service file if it exists
sudo systemctl kill $SERVICE_FILE_NAME

# Copy the modified .service file to the systemd directory
# Note: You might need sudo permission to copy to /etc/systemd/system
sudo cp $SERVICE_FILE_NAME $SYSTEMD_PATH

# Delete the redundant copy
rm $SERVICE_FILE_NAME

# Change permision of the .service file
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME

# Reload systemd to recognize the new service and enable it to start on boot
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_FILE_NAME

# Start the service immediately
sudo systemctl start $SERVICE_FILE_NAME

echo "'$SERVICE_FILE_NAME' has been installed and started successfully :)"
