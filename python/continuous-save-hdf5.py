import time, sys, collections, copy, ctypes, struct, os.path, datetime, threading
import numpy as np, h5py

import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis

sys.path.append('.')
import vrmagicTransformer

# start talking to the camera
cam=Aravis.Camera.new()
# already sets some parameters by itself
trsf=vrmagicTransformer.VRMagicTransformer(cam)
# increase scan rate
dev=cam.get_device()
dev.set_integer_feature_value('AcquisitionFrameRate_mHz',30000)
stream=cam.create_stream(None,None)
if not stream: raise RuntimeError("Failed to create stream (camera busy?).")

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
    # cs,co,as_,ao=csn/csd,con/cod,asn/asd,aon/aod
    i,grpNum=0,0 # counters
    while True:
        if not bufQueue:
            time.sleep(0.005)
            continue
        sys.stderr.write('\b'); sys.stderr.flush() # delete the > written by the producer :)
        d=trsf.payload2dict(bufQueue.pop())
        C,A,I,timestamp=d['C'],d['A'],d['intensity'],d['timestamp']
        # add frame to larger chunks
        CC[i,:]=C; AA[i,:]=A; II[i,:]=I; TT[i]=timestamp
        # chunks full: save to HDF5 and start new group
        if i==h5grpLen-1: 
            with h5py.File(h5name,'a') as h5:
                grp='%s/%05d'%(h5root,grpNum)
                for data,name in zip([CC,AA,II,TT],['z','x','intensity','timestamp']):
                    # compression saves about 70% of storage :))
                    h5.create_dataset(grp+'/'+name,data=data,compression='gzip',compression_opts=9)
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
        # put on the consumer's table, just raw data
        bufQueue.appendleft(buf.get_data())
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
