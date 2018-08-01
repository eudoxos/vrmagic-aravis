Example programs in c++
========================

* `read-trsf-params.cpp` reads coordinate transformation parameters to check whether you Aravis installation suffers from the unsigned int bug;
* `live-plot.cpp` is Qt5-based live plot of data coming from the sensor; it shows how to use thin wrapper in `vrmagic-sensor.hpp` to access the sensor, in conjunction with Aravis, and run the acquisition in background thread not to block the GUI.
