# GPR server

Due to the lack of good GPR libraries for C++, this package implements a GPR server in python to provide a fast and
reliable way of getting GPR predictions from a model.

The `GPR` class loads a desired *.pickle* file containing a GPR model (`GaussianProcessRegressor`
from `sklearn.gaussian_process`) and the corresponding scaler (`StandardScaler` from `sklearn.preprocessing`). Then, a
client can request the GPR prediction from a state via a ZMQ interface (also implemented in python).

## Running an example

To see how the GPR server is intended to use, check
the [test_gpr_server](../../control/executables/tests/test_gpr_server.cpp) executable.

First, run the python server:

```bash
bash build.sh
bash run.sh
python test_interface.py
```

Then, execute `test_gpr_server`. The output should look like this:

```
Waiting for GPR server...
GPR server ready
Loading GPR file...
mean: 1, std: 0
mean: 1, std: 0
mean: 1, std: 0
...
```

## Running the server

Before running the server, make sure to configure the main function of the [gpr_server.py](scripts/gpr_server.py) script
properly:

```python
interface = ZMQInterface('0.0.0.0:7777')
gpr = GPR()
class_switcher = {0: 'apple', 2: 'orange'}
file_switcher = {'apple': 'rbf_apple_0503.pickle', 'orange': 'rbf_orange_0503.pickle'}
```

1. Make sure that the URI of the `ZMQInterface` is correct and corresponds with the client interface.
2. Configure your classes with corresponding class index in the `class_switcher` dict.
3. Then, make sure to provide a `.pickle` file in the `script` folder for each class such that the model can be
   correctly loaded.

Then, to run the server:

```bash
bash build.sh
bash run.sh
python gpr_server.py
```

A use case for the GPR server can be found
in [optitrack_incision_trials](../../control/executables/optitrack_incision_trials.cpp).

## Training models

To see how to generate GPR models and save them to a `.pickle` file, refer to [this notebook]().

## Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Dominic Reber ([dominic.reber@epfl.ch](mailto:dominic.reber@epfl.ch))