import struct
import traceback

import zmq


class ZMQInterface(object):
    """
        ZMQ Interface for communication between the CITRIFIED control and the Python evaluation of the GPR model.

        Available methods (for usage, see documentation at function definition):
            - send
            - receive
        """

    def __init__(self, socket_uri="0.0.0.0:7777"):
        self._socket = self._configure_pair_socket(socket_uri, False)

    @staticmethod
    def _configure_pair_socket(socket_uri, bind):
        context = zmq.Context(1)
        socket = context.socket(zmq.PAIR)

        if bind:
            socket.bind("tcp://" + socket_uri)
        else:
            socket.connect("tcp://" + socket_uri)

        return socket

    def send(self, msg, datatype='d'):
        """
        Send state to ZMQ socket.
        :param msg: GPR mean and std as list of doubles
        :param datatype: message datatype
        :type msg: list of float
        :type datatype: char
        :return: Boolean if sending was successful
        :rtype: bool
        """
        if datatype == 'd':
            encoded_state = b"".join([struct.pack('d', msg[i]) for i in range(len(msg))])
        elif datatype == '?':
            encoded_state = b"".join([struct.pack('?', msg)])
        else:
            encoded_state = []
            print("[ZMQInterface::send] This datatype could not be encoded.")
            exit(1)
        res = self._socket.send(encoded_state, flags=0)
        return res is not None

    def receive_raw(self, flags=0):
        """
        Receive message from ZMQ socket.
        :param flags: ZMQ flags
        :type flags: int
        :return: Raw message if message was received, False if there was an error
        """
        try:
            return self._socket.recv(flags=flags)
        except zmq.ZMQError as e:
            if e.errno is not zmq.EAGAIN:
                traceback.print_exc()
            return None
