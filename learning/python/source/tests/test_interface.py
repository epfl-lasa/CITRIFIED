import os
import time

from gpr import GPR
from interface import ZMQInterface

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = ZMQInterface('GPR')
    # create gpr from pickle file
    file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gpr_rbf.pickle")
    gpr = GPR(file)

    start = time.time()
    k = 0
    while k < 1e6:
        now = time.time()

        request = interface.receive()
        print(request)
        if request:
            interface.send(gpr.predict(request))

        elapsed = time.time() - now
        k = k + 1
        print("Average rate: ", k / (time.time() - start))
