from network import Interface
import time

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = Interface('GPR')

    start = time.time()
    k = 0
    while k < 1e6:
        now = time.time()

        request = interface.receive()
        print(request)
        if request:
            interface.send([8])

        elapsed = time.time() - now
        sleep_time = (1. / 200) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1

        print("Average rate: ", k / (time.time() - start))
