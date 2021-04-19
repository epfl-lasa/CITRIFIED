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
            encoded_state = b"".join([struct.pack('?', True)])
        else:
            encoded_state = []
            print("This datatype could not be encoded.")
            exit(1)
        res = self._socket.send(encoded_state, flags=0)
        return res is not None

    def receive(self, datatype='d', flags=0):
        """
        Receive message from ZMQ socket.
        :param datatype: expected datatype
        :param flags: ZMQ flags
        :type datatype: char
        :type flags: int
        :return: Message if message was received, False if there was an error
        """
        try:
            if datatype == 'd':
                res = self._socket.recv(flags=flags)
                return [struct.unpack('d', res[i:i + 8])[0] for i in
                        range(0, len(res), 8)] if res is not None else False
            elif datatype == 's':
                res = self._socket.recv_string(flags=flags)
                return res if res is not None else False
            else:
                print("This datatype could not be decoded.")
                return False
        except zmq.ZMQError as e:
            if e.errno is not zmq.EAGAIN:
                traceback.print_exc()
            return False
