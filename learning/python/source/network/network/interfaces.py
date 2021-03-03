from .zmq_interface import *


class Interface(object):

    def __init__(self, type='GPR'):
        switcher = {'GPR': {'subscriber_uri': "0.0.0.0:7771", 'publisher_uri': "0.0.0.0:7770", 'bind': False}}

        self._subscriber, self._publisher = zmq_configure_sockets(switcher[type]['subscriber_uri'],
                                                                  switcher[type]['publisher_uri'],
                                                                  switcher[type]['bind'])

    def receive(self):
        return zmq_receive(self._subscriber)

    def poll(self):
        return zmq_poll(self._subscriber)

    def send(self, message):
        return zmq_send(self._publisher, message)
