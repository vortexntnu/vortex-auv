# Setting up libraries
import errno
import time
from socket import AF_INET, SOCK_DGRAM, socket


class TeensyCommunicationUDP:
    """Handles the RPI side of Teensy-RPI UDP communication.

    This class is implemented with a singleton pattern for convenience.

    Note:
        Private members are denoted by `_member_name`.

    Attributes:
    -----------
        _TEENSY_IP (string): self-explanatory
        _TEENSY_PORT (int): Teensy's data port
        _my_port (int): The device's data port
        _MAX_PACKAGE_SIZE_RECEIVED (int): Max size (in bytes) of UDP receive buffer
        _timeout (int): Socket timeout when reading data
        _client_socket (socket): UDP socket for Teensy communication
        _timeout_max (int): Time to wait before retrying handshake
        _data_string (str): Buffer for received Teensy data
        _data_target (str): The field of `acoustics_data` that is written to
        acoustics_data (dict[str, list[int]]): Container for data from Teensy

    Methods:
    --------
        init_communication(frequenciesOfInterest: list[tuple[int, int]]) -> None:
            Sets up socket for communication with Teensy and waits for handshake
        fetch_data() -> None:
            Reads a full data message from Teensy and saves it
        _write_to_target() -> None:
            Writes _data_string to the correct field of acoustics_data
        _parse_data_string(is_float: bool) -> list[float] | list[int] | None:
            Converts _data_string into a list of either floats or integers
        _get_ip() -> None:
            Gets the IP address of the device
        _send_acknowledge_signal() -> None:
            Sends the _INITIALIZATION_MESSAGE to Teensy
        _check_if_available() -> None:
            Checks the UDP message buffer for a READY message
        _send_frequencies_of_interest(frequenciesOfInterest: list[tuple[float, float]]) -> None:
            Sends the list of frequencies and variances to Teensy
    """

    # Teensy networking Setup
    _TEENSY_IP = "10.0.0.111"
    _TEENSY_PORT = 8888
    _my_port = 9999
    _MAX_PACKAGE_SIZE_RECEIVED = 65536
    _timeout = 1
    _address = (_TEENSY_IP, _TEENSY_PORT)

    _INITIALIZATION_MESSAGE = "HELLO :D"  # This is a message only sent once to establish 2 way communication between Teensy and client

    _client_socket = socket(AF_INET, SOCK_DGRAM)

    _timeout_max = 10
    _data_string = ""
    _data_target = ""
    acoustics_data = {
        "HYDROPHONE_1": [0],
        "HYDROPHONE_2": [0],
        "HYDROPHONE_3": [0],
        "HYDROPHONE_4": [0],
        "HYDROPHONE_5": [0],
        "SAMPLES_FILTERED": [0],
        "FFT": [0],
        "PEAK": [0],
        "TDOA": [0],
        "LOCATION": [0],
    }

    @classmethod
    def init_communication(cls, frequencies_of_interest: list[tuple[int, int]]) -> None:
        """Sets up communication with teensy.

        Args:
            frequencies_of_interest (list[tuple[int, int]]): List of frequencies to look for
        """
        assert len(frequencies_of_interest) == 10, (
            "Frequency list has to have exactly 10 entries"
        )

        cls.my_ip = cls._get_ip()

        # Socket setup
        cls._client_socket.settimeout(cls._timeout)
        cls._client_socket.bind((cls.my_ip, cls._my_port))
        cls._client_socket.setblocking(False)

        cls._send_acknowledge_signal()
        time_start = time.time()

        # Wait for READY signal
        while not cls._check_if_available():
            print("Did not receive READY signal. Will wait.")
            time.sleep(1)

            if time.time() - time_start > cls._timeout_max:
                print("Gave up on receiving READY. Sending acknowledge signal again")
                # Start over
                time_start = time.time()
                cls._send_acknowledge_signal()

        print("READY signal received, sending frequencies...")
        cls._send_frequencies_of_interest(frequencies_of_interest)

    @classmethod
    def fetch_data(cls) -> None:
        """Gets data from teensy and stores it in `acoustics_data`."""
        i = 0

        while True:
            data = cls._get_raw_data()

            if data == None:
                return

            if data not in cls.acoustics_data.keys():
                cls._data_string += data
            else:
                cls._write_to_target()
                cls._data_target = data

            # Ah yes, my good friend code safety
            i += 1

            if i > 1000:
                i = 0
                print("Max tries exceeded")
                break

    @classmethod
    def _write_to_target(cls) -> None:
        """Writes to the current target in `acoustics_data` and clears the data string."""
        if cls._data_target == "TDOA" or cls._data_target == "LOCATION":
            data = cls._parse_data_string(is_float=True)
        else:
            data = cls._parse_data_string(is_float=False)

        if data == None:
            cls._data_string = ""
            return

        cls.acoustics_data[cls._data_target] = data

        cls._data_string = ""

    @classmethod
    def _get_raw_data(cls) -> str | None:
        """Gets a message from teensy.

        Returns:
            The message in the UDP buffer if there is one
        """
        try:
            rec_data, _ = cls._client_socket.recvfrom(cls._MAX_PACKAGE_SIZE_RECEIVED)
            message_received = rec_data.decode()
            return message_received
        except OSError as e:  # `error` is really `socket.error`
            if e.errno == errno.EWOULDBLOCK:
                pass
            else:
                print("Socket error: ", e)

    @classmethod
    def _parse_data_string(cls, is_float: bool) -> list[float] | list[int] | None:
        """Converts _data_string to a list.

        Args:
            is_float (bool): whether _data_string should be seen as a list of floats or ints

        Returns:
            The converted list
        """
        if cls._data_string == '':
            return

        try:
            # Format data from CSV string to floats, ignore last value
            if is_float:
                return list(map(float, cls._data_string.split(",")[:-1]))
            else:
                return list(map(int, cls._data_string.split(",")[:-1]))
        except Exception as e:
            print(f"The string '{cls._data_string}' caused an error when parsing")
            print(f"The exception was: {e}")

    # stackoverflow <3
    # https://stackoverflow.com/questions/166506/finding-local-ip-addresses-using-pythons-stdlib
    @classmethod
    def _get_ip(cls) -> None:
        """Gets the device's IP address."""
        s = socket(AF_INET, SOCK_DGRAM)
        s.settimeout(0)
        try:
            # doesn't even have to be reachable
            s.connect((cls._TEENSY_IP, 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()

        return ip

    @classmethod
    def _send_acknowledge_signal(cls) -> None:
        """Sends "HELLO :D to teensy."""
        try:
            cls._client_socket.sendto(
                cls._INITIALIZATION_MESSAGE.encode(), cls._address
            )
            print("DEBUGGING: Sent acknowledge package")
        except Exception as e:
            print("Error from send_acknowledge_signal")
            print(e)
            pass

    @classmethod
    def _check_if_available(cls) -> None:
        """Checks if READY has been received.

        Note: The while loop here may not be necessary, it is just there to make absolutely sure that *all*
        the data in the UDP buffer is read out when waiting for ready signal, to avoid strange bugs
        """
        try:
            i = 0
            while True:
                # Read data
                message = cls._get_raw_data()
                # Check if there is no more data left
                if message == None:
                    return False

                # Check if correct signal was sent
                if message == "READY":
                    return True

                i += 1

                if i > 200:
                    i = 0
                    print("Max tries exceeded")
                    break
        except Exception as e:
            print(f"check_if_available rased exception: {e}")
            return False

    @classmethod
    def _send_frequencies_of_interest(
        cls, frequencies_of_interest: list[tuple[float, float]]
    ) -> None:
        """Sends the list of frequencies with variance to teensy.

        Args:
            frequencies_of_interest (list[tuple[float, float]]): The list of frequencies w/ variance
        """
        try:
            # Format (CSV): xxx,x,xx,x...,x (frequency list comes first, then variances)
            assert len(frequencies_of_interest) == 10, (
                "List of frequencies has to be ten entries long!"
            )

            # ten messages in total, one message for each entry to work around the max packet size
            for frequency, variance in frequencies_of_interest:
                frequency_variance_msg = f"{str(frequency)},{str(variance)},"

                # print(self.address);
                cls._client_socket.sendto(frequency_variance_msg.encode(), cls._address)
        except:
            print("Couldn't send Frequency data")
