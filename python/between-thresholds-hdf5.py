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
dev.set_integer_feature_value('AcquisitionFrameRate_mHz',200000)
dev.set_string_feature_value('LaserMode','LaserOn')
stream=cam.create_stream(None,None)
if not stream: raise RuntimeError("Failed to create stream (camera busy?).")

# set up buffers and deque for thread comm
payload=cam.get_payload()
for i in range(0,500): stream.push_buffer(Aravis.Buffer.new_allocate(payload))
bufQueue=collections.deque(maxlen=500) # discard anything unprocessed if more than 50 waiting already


# set threshold so that mean height is above it at the very beginning and at the very end
# leaving that domain will trigger and stop acquisition respecitvely.


def consumeBuffers(thresh=-30):
    'Consumes buffers from bufQueue, transforms, builds larger chunks and dumps to HDF5 onces in a while'
    # save this many many consecutive frames as a single 2d array to HDF5
    h5chunk=2048
    # name of the dump file, can be recycled
    h5name='vrmagic-dump.hdf5'
    # root group, new for every launch of this script
    h5root='dump-%s'%datetime.datetime.now().isoformat(timespec='seconds')
    # prepare chunk storage
    shape=(h5chunk,2048)
    CC,AA,II,TT=np.empty(shape,dtype='float32'),np.empty(shape,dtype='float32'),np.empty(shape,dtype='float32'),np.empty((h5chunk,1),dtype='float32')
    chunkRow=0
    chunks=0
    # -1: inactive, 0: waiting for threshold, 1: initial threshold, 2: recording, 3: final threshold
    threshStatus=-1 if thresh is None else 0 
    lastTrigger=-1

    def writeChunk(h5chunk):
        with h5py.File(h5name,'a') as h5:
            # create datasets from scratch, empty at first
            if h5root not in h5:
                for name,c,dtype in zip(['z','x','intensity','timestamp'],[cols,cols,cols,1],['float32','float32','uint16','float32']):
                    h5.create_dataset(h5root+'/'+name,(0,c),maxshape=(None,c),dtype=dtype,compression='gzip',compression_opts=9)
            # append outstanding data
            for data,name in zip([CC,AA,II,TT],['z','x','intensity','timestamp']):
                ds=h5[h5root+'/'+name]
                #print(name,ds.shape)
                ds.resize((ds.shape[0]+h5chunk,ds.shape[1]))
                # print(name,ds.shape,data.shape,ds[-h5chunk:,:].shape)
                ds[ds.shape[0]-h5chunk:,:]=data[:h5chunk]
                hdf5rows=ds.shape[0]
        print(' {}: {} chunks, {} rows, {:_} bytes)'.format(h5name,chunks,hdf5rows,os.path.getsize(h5name)))

    while True:
        if not bufQueue:
            time.sleep(0.005)
            continue
        sys.stderr.write('\b'); sys.stderr.flush() # delete the > written by the producer :)
        d=trsf.payload2dict(bufQueue.pop())
        C,A,I,timestamp=d['C'],d['A'],d['intensity'],np.array([[d['timestamp']]])
        if threshStatus>=0:
            mean=np.nanmean(C)
            if mean<thresh:
                if threshStatus==0:
                    print('Entered initial threshold with z mean %g<%g'%(mean,thresh))
                    threshStatus=1
                elif threshStatus==2:
                    print('Entered final threshold with z mean %g<%g'%(mean,thresh))
                    threshStatus=3 # write the rest and terminate
            elif mean>=thresh:
                if threshStatus==1:
                    print('Entered recording stage with z mean %g>=%g'%(mean,thresh))
                    threshStatus=2 # starts recording
        if threshStatus in (0,1): continue
        # add frame to larger chunks
        CC[chunkRow,:]=C; AA[chunkRow,:]=A; II[chunkRow,:]=I; TT[chunkRow,:]=timestamp
        cols=CC.shape[1]
        chunkRow+=1
        # chunks full: save to HDF5 and start new group
        if chunkRow%h5chunk==0:
            writeChunk(h5chunk)
            chunks+=1
            chunkRow=0
        if threshStatus==3:
            writeChunk(chunkRow%h5chunk)
            print('Wrote final chunk to HDF5.')
            return




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
            sys.stderr.write('          %.5g fps                           \r'%(Nmod*1./(t1-t0)));
            t0=t1

# start the consumer in bg thread
t=threading.Thread(target=consumeBuffers).start()
# run acquisition in fg forever; press ^C to quit
produceBuffers()
