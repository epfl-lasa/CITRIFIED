import struct
import traceback

import zmq


def zmq_configure_sockets(subscriber_uri, publisher_uri, bind):
    context = zmq.Context(1)

    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

    publisher = context.socket(zmq.PUB)

    if bind:
        subscriber.bind("tcp://" + subscriber_uri)
        publisher.bind("tcp://" + publisher_uri)
    else:
        subscriber.connect("tcp://" + subscriber_uri)
        publisher.connect("tcp://" + publisher_uri)
    return subscriber, publisher


def zmq_send(publisher, msg):
    encoded_state = b"".join([struct.pack('d', msg[i]) for i in range(len(msg))])
    res = publisher.send(encoded_state, flags=0)
    return res is not None


def zmq_receive(subscriber, flags=0):
    try:
        res = subscriber.recv(flags=flags)
        if res is None:
            return False
        else:
            return [struct.unpack('d', res[i:i + 8])[0] for i in range(0, len(res), 8)]
    except zmq.ZMQError as e:
        if e.errno is not zmq.EAGAIN:
            traceback.print_exc()
        return False


def zmq_poll(subscriber):
    return zmq_receive(subscriber, flags=zmq.NOBLOCK)
