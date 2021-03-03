IMAGE_NAME=citrified/learning/runtime
SCRIPT=test_interface
PORT_STATE=7771
PORT_GPR_OUTPUT=7770

#docker run -it --rm -p"$PORT_STATE":"$PORT_STATE" -p"$PORT_GPR_OUTPUT":"$PORT_GPR_OUTPUT" --name test $IMAGE_NAME $SCRIPT
docker run -it --rm --name test $IMAGE_NAME $SCRIPT
