#!/usr/bin/python3
# Python Libraries
import subprocess


class IPDriver:
    def __init__(self) -> None:
        self.cmd = "hostname -I | cut -d' ' -f1"

    def get_ip(self) -> str:
        """
        Executes a shell command to retrieve the IP address.

        Returns:
            str: The IP address as a string.
        """
        ip_bytes = subprocess.check_output(self.cmd, shell=True)
        ip_str = ip_bytes.decode("utf-8")

        return ip_str
