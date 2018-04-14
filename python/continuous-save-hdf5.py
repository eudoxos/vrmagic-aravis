import time, sys, collections, copy, ctypes, struct, os.path, datetime, threading
import numpy as np, h5py

import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis

# start talking to the camera
cam=Aravis.Camera.new()
dev=cam.get_device()
stream=cam.create_stream(None,None)
if not stream: raise RuntimeError("Failed to create stream (camera busy?).")

# set acquisition settings
dev.set_string_feature_value('TransferFormat','PROFILE_COORD16')
dev.set_integer_feature_value('IntensityDataEnable',1)
dev.set_string_feature_value('LaserMode','LaserOn')
dev.set_integer_feature_value('CoordADataEnable',1)
dev.set_integer_feature_value('FooterColumnEnable',1)
dev.set_integer_feature_value('AcquisitionFrameRate_mHz',30000)

# get parameters for post-processing raw buffers
# (+ hack around https://github.com/AravisProject/aravis/issues/147)
asn,asd,aon,aod,csn,csd,con,cod=[struct.unpack('i',dev.get_integer_feature_value(n).to_bytes(4,'little'))[0] for n in ['Scan3dCoordinateAScale_Numerator','Scan3dCoordinateAScale_Denominator','Scan3dCoordinateAOffset_Numerator','Scan3dCoordinateAOffset_Denominator','Scan3dCoordinateCScale_Numerator','Scan3dCoordinateCScale_Denominator','Scan3dCoordinateCOffset_Numerator','Scan3dCoordinateCOffset_Denominator']]
invalidValue=dev.get_integer_feature_value('Scan3dInvalidDataValue')
tickHz=dev.get_integer_feature_value('FooterTimestampTickFrequency')
tickOffset=dev.get_integer_feature_value('IntraFooterStartOfExposureTimestampByteOffset')

# set up buffers and deque for thread comm
payload=cam.get_payload()
for i in range(0,50): stream.push_buffer(Aravis.Buffer.new_allocate(payload))
bufQueue=collections.deque(maxlen=50) # discard anything unprocessed if more than 50 waiting already


def consumeBuffers():
    'Consumes buffers from bufQueue, transforms, builds larger chunks and dumps to HDF5 onces in a while'
    # save this many many consecutive frames as a single 2d array to HDF5
    h5grpLen=2048
    # name of the dump file, can be recycled
    h5name='vrmagic-dump.hdf5'
    # root group, new for every launch of this script
    h5root='dump-%s'%datetime.datetime.now().isoformat(timespec='seconds')
    # prepare chunk storage
    shape=(h5grpLen,2048)
    CC,AA,II,TT=np.empty(shape,dtype='float32'),np.empty(shape,dtype='float32'),np.empty(shape,dtype='float32'),np.empty((h5grpLen,),dtype='float32')
    # transformation params
    cs,co,as_,ao=csn/csd,con/cod,asn/asd,aon/aod
    i,grpNum=0,0 # counters
    while True:
        if not bufQueue:
            time.sleep(0.005)
            continue
        sys.stderr.write('\b'); sys.stderr.flush() # delete the > written by the producer :)
        dta=bufQueue.pop()
        f=dta[0].view(dtype='int16')
        # FIXME: should use CoordCDataByteOffset,CoordADataByteOffset,IntensityDataByteOffset,FooterDataByteOffset
        c16,a16,i16,footer=f[:2048],f[2048:4096],f[4096:6144],f[6144:].tobytes()
        C=cs*c16+co; A=as_*a16+ao; I=i16
        _inv=(c16==invalidValue)
        # print('Invalid: ',np.sum(_inv))
        C[_inv]=np.nan; A[_inv]=np.nan; I[_inv]=0
        # extract timestamp from footer; Q is uint64 (8b)
        timestamp=struct.unpack('Q',footer[tickOffset:tickOffset+8])[0]/tickHz
        # add frame to larger chunks
        CC[i,:]=C; AA[i,:]=A; II[i,:]=I; TT[i]=timestamp
        # chunks full: save to HDF5 and start new group
        if i==h5grpLen-1: 
            with h5py.File(h5name,'a') as h5:
                grp=h5root+'/%05d'%grpNum
                h5[grp+'/z'],h5[grp+'/x'],h5[grp+'/intensity'],h5[grp+'/timestamp']=CC,AA,II,TT
                i=0; grpNum+=1
            print(' {}: {}, {:_} bytes)'.format(h5name,grp,os.path.getsize(h5name)))
        else: i+=1


# inspired by https://kushalvyas.github.io/gige_ubuntu.html
def produceBuffers():
    cam.start_acquisition()
    t0=time.time()
    N,Nmod=0,100
    while True:
        buf=stream.try_pop_buffer()
        if not buf: # nothing to do, wait a little bit
            time.sleep(0.005)
            continue
        # type-casting and copy : see https://github.com/SintefRaufossManufacturing/python-aravis/blob/master/aravis.py
        arr=np.ctypeslib.as_array(ctypes.cast(buf.get_data(),ctypes.POINTER(ctypes.c_uint16)),(buf.get_image_height(),buf.get_image_width())).copy()
        # put on the consumer's table
        bufQueue.appendleft(arr)
        sys.stderr.write('>'); sys.stderr.flush()
        # free the buffer for further use
        stream.push_buffer(buf)
        # report fps once in a while
        N+=1
        if N%Nmod==0:
            t1=time.time()
            sys.stderr.write('%.3g fps                                  \r'%(Nmod*1./(t1-t0)));
            t0=t1

# start the consumer in bg thread
t=threading.Thread(target=consumeBuffers).start()
# run acquisition in fg forever; press ^C to quit
produceBuffers()
