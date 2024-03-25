#!/usr/bin/python3
# Python Libraries
import subprocess

class IPDriver:
    def __init__(self):
        self.cmd = "hostname -I | cut -d' ' -f1"

    def get_IP(self):
        IP_bytes = subprocess.check_output(self.cmd, shell=True)
        IP_str = IP_bytes.decode("utf-8")

        return IP_str