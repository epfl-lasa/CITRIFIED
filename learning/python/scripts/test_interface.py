import os
import struct

from gpr import GPR, ZMQInterface

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = ZMQInterface('0.0.0.0:7777')
    # create gpr from pickle file
    file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rbf_orange_0503.pickle")
    gpr = GPR()
    class_switcher = {1: 'orange', 2: 'apple'}

    while True:
        # now = time.time()
        raw = interface.receive_raw()
        if not raw:
            continue

        header = struct.unpack('i', raw[0:4])[0]
        if header == 1:
            raw = raw[8:]
            request = [struct.unpack('d', raw[i:i + 8])[0] for i in range(0, len(raw), 8)]
            if gpr.initialized():
                reply = gpr.predict(request)
                interface.send(reply)
            else:
                interface.send(False, '?')
        elif header == 0:
            gpr.reset()
            try:
                class_name = class_switcher[struct.unpack('i', raw[4:8])[0]]
            except KeyError as e:
                interface.send(False, '?')
                print("This class index is not supported. Could not load GPR model.")
                continue
            if gpr.init(file):
                interface.send(True, '?')
        else:
            print("This kind of header type is not defined. Exit")
            exit(1)
