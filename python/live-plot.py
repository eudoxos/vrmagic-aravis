import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis
import matplotlib.pyplot as plt, numpy as np

import sys
sys.path.append('.')
import vrmagicTransformer

cam=Aravis.Camera()
trsf=vrmagicTransformer.VRMagicTransformer(cam)
stream=cam.create_stream(None,None)
stream.push_buffer(Aravis.Buffer.new_allocate(cam.get_payload()))
cam.start_acquisition()
gr=plt.plot([0,1,2],[0,1,2])[0]
ax=plt.gca()
xmin=xmax=0
ymin=ymax=0
plt.grid(True)
plt.ion()
plt.show()
while True:
    buf=stream.pop_buffer()
    res=trsf.payload2dict(buf.get_data())
    stream.push_buffer(buf)
    gr.set_xdata(res['A'])
    gr.set_ydata(res['C'])
    xmin,xmax=min(xmin,np.nanmin(res['A'])),max(xmax,np.nanmax(res['A']))
    ymin,ymax=min(ymin,np.nanmin(res['C'])),max(ymax,np.nanmax(res['C']))
    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymin,ymax)
    # print(xmin,xmax,ymin,ymax,np.sum(np.isnan(res['C'])))
    plt.suptitle('t=%g s'%res['timestamp'])
    plt.draw()
    plt.pause(0.001)
cam.stop_acquisition()

