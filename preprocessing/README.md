# Data preprocessing


## findForcePlateTransform

This MATLAB function will find the transformation matrix that converts
FT measurements from sensor frame into the optitrack world frame. It
resamples the optitrack and FT data from a calibration set into a common
timebase, then segmenst the force data into areas of high vertical force
(the calibration corner points). It calculates the XY center of pressure
for those points and correlates them with the XYZ tool tip at that time.

Finally SVD is used to find the transform that best fits the FT cop points
onto the optitrack tooltip points, giving the FT-to-World frame transform.
