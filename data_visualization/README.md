# MATLAB function for plotting trial data in JSON format

This MATLAB function is used to plot the required variables from the trial data. Messages are logged in the JSON format. The MATLAB function parses the data and plots the required values.

## Input Arguments

Two inputs have to be given to the function depending on which data has to be plotted:

- `TYPE` - velocity, force, position
- `SIGNAL` - raw, filtered

## Outputs

This function plots the requried data. Function outputs - time as `t` and array of the plotted data as `data`.


### Example MATLAB Code for using this function

```
filename = 'xxx.json';

TYPE = 'force';
SIGNAL = 'filtered';

[t, data] = plotJSONTrialData(filename,TYPE,SIGNAL)
```
