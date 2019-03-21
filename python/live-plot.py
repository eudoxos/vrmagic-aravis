import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis
import matplotlib.pyplot as plt, numpy as np

import sys
sys.path.append('.')
import vrmagicTransformer

cam=Aravis.Camera.new('VRmagicImaging-EFD0AFMH07')
trsf=vrmagicTransformer.VRMagicTransformer(cam,yAOI=(700,300),xAOI=(650,800),fps=1000)
rawCoords=True

dev=cam.get_device()

stream=cam.create_stream(None,None)
stream.push_buffer(Aravis.Buffer.new_allocate(cam.get_payload()))
cam.start_acquisition()
gr=plt.plot([0,1,2],[0,1,2])[0]
ax=plt.gca()
xmin=xmax=0
ymin=ymax=0
print('Invalid value is',trsf.invalid)
print(dev.get_integer_feature_value('WidthMax'),dev.get_integer_feature_value('HeightMax'))

#def pxMinMax(e):
#    return 2**e*2**(16-e)-2**(15)

#print('A:',1/trsf.as_,trsf.ao)
#print('C:',1/trsf.cs,trsf.co)
#print('A:',trsf.asn,trsf.asd,trsf.aon,trsf.aod)
#print('C:',trsf.csn,trsf.csd,trsf.con,trsf.cod)
#def rawMinMax(pxMax,ro,rs): return rs*(np.linspace(0,pxMax,5)*2**7)+ro
#print(rawMinMax(dev.get_integer_feature_value('WidthMax'),trsf.ao,trsf.as_))
#print(rawMinMax(dev.get_integer_feature_value('HeightMax'),trsf.co,trsf.cs))

#sys.exit(0)
#aMin,aMax=dev.get_integer_feature_value('WidthMax')
# aa,cc=np.meshgrid(32*np.arange(0,1,)dev.get_integer_feature_value('WidthMax'),200),np.arange(0,32*dev.get_integer_feature_value('HeightMax'),100*16))
# ax.pcolormesh(trsf.trsfA(aa),trsf.trsfC(cc),np.ones_like(aa),edgecolors='black',facecolor='none',alpha=.5)
plt.grid(True)
plt.ion()
plt.show()
while True:
    buf=stream.pop_buffer()
    res=trsf.payload2dict(buf.get_data(),retRaw=rawCoords)
    stream.push_buffer(buf)
    aa,cc=res['a16'] if rawCoords else res['A'],res['c16'] if rawCoords else res['C']
    if rawCoords:
        aa,cc=aa.astype('float32'),cc.astype('float32')
        aa[aa==trsf.invalid],cc[cc==trsf.invalid]=np.nan,np.nan
    gr.set_xdata(aa)
    gr.set_ydata(cc)
    xmin,xmax=min(xmin,np.nanmin(aa)),max(xmax,np.nanmax(aa))
    ymin,ymax=min(ymin,np.nanmin(cc)),max(ymax,np.nanmax(cc))
    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymax,ymin)
    # print(xmin,xmax,ymin,ymax,np.sum(np.isnan(res['C'])))
    plt.suptitle('t=%g s'%res['timestamp'])
    plt.draw()
    plt.pause(0.001)
cam.stop_acquisition()

