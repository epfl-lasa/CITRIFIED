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

    def __init__(self, interface_type='GPR'):
        switcher = {'GPR': {'subscriber_uri': "0.0.0.0:7771", 'publisher_uri': "0.0.0.0:7770", 'bind': False}}

        self._subscriber, self._publisher = self._configure_sockets(switcher[interface_type]['subscriber_uri'],
                                                                    switcher[interface_type]['publisher_uri'],
                                                                    switcher[interface_type]['bind'])

    @staticmethod
    def _configure_sockets(subscriber_uri, publisher_uri, bind):
        context = zmq.Context(1)

        subscriber = context.socket(zmq.SUB)
        subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        subscriber.setsockopt(zmq.CONFLATE, 1)

        publisher = context.socket(zmq.PUB)

        if bind:
            subscriber.bind("tcp://" + subscriber_uri)
            publisher.bind("tcp://" + publisher_uri)
        else:
            subscriber.connect("tcp://" + subscriber_uri)
            publisher.connect("tcp://" + publisher_uri)
        return subscriber, publisher

    def send(self, msg):
        """
        Send state to ZMQ socket.
        :param msg: Current robot state as defined in bullet_robot.py
        :type msg: list of float
        :return: Boolean if sending was successful
        :rtype: bool
        """
        encoded_state = b"".join([struct.pack('d', msg[i]) for i in range(len(msg))])
        res = self._publisher.send(encoded_state, flags=0)
        return res is not None

    def receive(self, flags=0):
        """
        Receive message from ZMQ socket.
        :param flags: ZMQ flags
        :type flags: int
        :return: Message if message was received, False if there was an error
        """
        try:
            res = self._subscriber.recv(flags=flags)
            if res is None:
                return False
            else:
                return [struct.unpack('d', res[i:i + 8])[0] for i in range(0, len(res), 8)]
        except zmq.ZMQError as e:
            if e.errno is not zmq.EAGAIN:
                traceback.print_exc()
            return False
