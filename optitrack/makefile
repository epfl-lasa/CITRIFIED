include = -I./include
libpath = -L./lib
libs = -lNatNetLibShared -lzmq

all: SampleClient OptiTrackZMQBridge test_receive

SampleClient:
	g++ source/SampleClient.cpp $(include) $(libpath) $(libs) -o run/SampleClient

OptiTrackZMQBridge:
	g++ source/OptiTrackZMQBridge.cpp source/OptiTrackClient.cpp $(include) $(libpath) $(libs) -o run/OptiTrackZMQBridge

test_receive:
	g++ source/test_receive_zmq.cpp $(include) $(libpath) $(libs) -o run/test_receive_zmq

.PHONY: clean
clean:
	@rm -r ./run/*