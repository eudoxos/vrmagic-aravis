import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis
import matplotlib.pyplot as plt
import numpy as np
import ctypes, struct, time, sys

sys.path.append('.')
import vrmagicTransformer

cam=Aravis.Camera()
trsf=vrmagicTransformer.VRMagicTransformer(cam)
stream=cam.create_stream(None,None)
stream.push_buffer(Aravis.Buffer.new_allocate(cam.get_payload()))
cam.start_acquisition()
buf=stream.pop_buffer()
res=trsf.payload2dict(buf.get_data(),retRaw=True)
cam.stop_acquisition()
for w in ['c16','a16','i16']: plt.plot(res[w],label=w)
plt.legend(); plt.grid('True')
plt.figure()
plt.plot(res['A'],res['C'])
plt.grid(True)
plt.show()

