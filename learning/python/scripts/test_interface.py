import os

from gpr import GPR
from interface import ZMQInterface

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = ZMQInterface('0.0.0.0:7777')
    # create gpr from pickle file
    file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gpr_rbf.pickle")
    gpr = GPR()

    while True:
        # now = time.time()
        if not gpr.initialized():
            request = interface.receive(datatype='s')
            # print(request)
            if gpr.init(file):
                interface.send(True, '?')
        else:
            request = interface.receive()
            reply = gpr.predict(request)
            # print(reply)
            interface.send(reply)
