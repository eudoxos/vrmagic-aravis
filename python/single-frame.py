import aravis
import matplotlib.pyplot as plt
import numpy as np
plt.ion()
cam=aravis.Camera()
cam.set_feature('TransferFormat','PROFILE_COORD16')
# the format 2048x1 shows up in arv-viewer correctly now :)
cam.set_feature('IntensityDataEnable',1)
cam.set_feature('LaserMode','LaserOn')
cam.set_feature('CoordADataEnable',1)
cam.set_feature('FooterDataEnable',1)
cam.start_acquisition_continuous()
f=cam.pop_frame()[0].view(dtype='int16')
cam.stop_acquisition()
# f has 6144 items
c16,a16,i16,footer=f[:2048],f[2048:4096],f[4096:6144],f[6144:].tobytes()
sa=cam.get_feature('Scan3dCoordinateAScale_Numerator')*1./cam.get_feature('Scan3dCoordinateAScale_Denominator')
# XXX cast numerator to int16 (only here!!)
oa=np.array(cam.get_feature('Scan3dCoordinateAOffset_Numerator'),dtype='uint16').view('int16')*1./cam.get_feature('Scan3dCoordinateAOffset_Denominator')
sc=cam.get_feature('Scan3dCoordinateCScale_Numerator')*1./cam.get_feature('Scan3dCoordinateCScale_Denominator')
oc=cam.get_feature('Scan3dCoordinateCOffset_Numerator')*1./cam.get_feature('Scan3dCoordinateCOffset_Denominator')
tickHz=cam.get_feature('FooterTimestampTickFrequency')
tickOffset=cam.get_feature('IntraFooterStartOfExposureTimestampByteOffset')
import struct
tick=struct.unpack('Q',footer[tickOffset:tickOffset+8])[0]
print(tick)
print('Tick is %d, time is %g s'%(tick,tick/tickHz))
plt.plot(c16,label='c16'); plt.plot(a16,label='a16'); plt.plot(i16,label='i16')
plt.legend(); plt.grid('True')
plt.figure()
C=sc*c16+oc; A=sa*a16+oa
# set to NaN where invalid value is encoutered (unclear if it should be in A or C)
invalid=cam.get_feature('Scan3dInvalidDataValue')
C[c16==invalid]=np.nan
plt.plot(A,C)
plt.grid(True)
plt.legend()
