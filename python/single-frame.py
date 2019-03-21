import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis
import matplotlib.pyplot as plt
import numpy as np
import ctypes, struct, time, sys

sys.path.append('.')
import vrmagicTransformer

cam=Aravis.Camera()
dev=cam.get_device()
print('raw temperature [uncalibrated]:',dev.get_integer_feature_value("SensorTemperatureRegisterRawValue"))
print('device temperature [°C]:',.001*dev.get_integer_feature_value("DeviceTemperature"))

if 1:
    trsf=vrmagicTransformer.VRMagicTransformer(cam)
    stream=cam.create_stream(None,None)
    stream.push_buffer(Aravis.Buffer.new_allocate(cam.get_payload()))
    cam.start_acquisition()
    buf=stream.pop_buffer()
    res=trsf.payload2dict(buf.get_data(),retRaw=True)
    for range in ('Current','Full'):
       for AC in 'AC':
            pref='ActCalib%sCoordinate%sRange'%(range,AC)
            print(pref+' min…max (scale): %d…%d (%d)'%tuple([dev.get_integer_feature_value(pref+what) for what in ('Min','Max','Scale')]))
    cam.stop_acquisition()
    for w in ['c16','a16','i16']: plt.plot(res[w],label=w)
    plt.legend(); plt.grid('True')
    plt.figure()
    plt.plot(res['A'],res['C'])
    plt.grid(True)
    plt.show()

