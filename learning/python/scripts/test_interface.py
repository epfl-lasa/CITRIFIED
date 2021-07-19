import struct

from gpr import GPR, ZMQInterface

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = ZMQInterface('0.0.0.0:7777')
    gpr = GPR()
    class_switcher = {0: 'apple', 2: 'orange'}

    while True:
        # now = time.time()
        raw = interface.receive_raw()
        if not raw:
            continue

        header = struct.unpack('i', raw[0:4])[0]
        if header == 1:
            interface.send([1, 0])
        elif header == 0:
            gpr.reset()
            try:
                class_name = class_switcher[struct.unpack('i', raw[4:8])[0]]
            except KeyError as e:
                interface.send(False, '?')
                print("This class index is not supported. Could not load GPR model.")
                continue
            interface.send(True, '?')
        elif header == 99:
            interface.send('True', '?')
        else:
            print("This kind of header type is not defined. Exit")
            exit(1)
