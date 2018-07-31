#!/usr/bin/python3
import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis
import matplotlib.pyplot as plt, numpy as np

import sys, time
sys.path.append('.')
import vrmagicTransformer

# ID strings are as reported by arvtool
cams=Aravis.Camera.new('VRmagicImaging-EFD0EEY3Y5'),Aravis.Camera.new('VRmagicImaging-EFD0AFMH07')

streams=[]
for c in cams:
    stream=c.create_stream(None,None)
    if not stream: raise RuntimeError("Error creating stream for "+str(c)+" (busy?)")
    for i in range(0,20): stream.push_buffer(Aravis.Buffer.new_allocate(c.get_payload()))
    streams.append(stream)
d0,d1=cams[0].get_device(),cams[1].get_device()
# master
d0.set_integer_feature_value('AcquisitionFrameRate_mHz',30000)
d0.set_boolean_feature_value('TrigOutMultiDeviceBusEnable',True)
# slave
d1.set_boolean_feature_value('TrigInMultiDeviceBusEnable',True)

print('TrigIn connected?:',d1.get_boolean_feature_value('TrigInMultiDeviceBusConnected'))

trsfs=[vrmagicTransformer.VRMagicTransformer(c) for c in cams]
for c in cams: c.start_acquisition()
bufs=[None for s in streams]
while True:
    bufs_=[s.try_pop_buffer() for s in streams]
    # check if we got some new buffers
    # if there is one waiting already and new one came, xrun occured: warn and discard old frame
    for i,(s,b,b_) in enumerate(zip(streams,bufs,bufs_)):
        if b_ and b!=b_:
            if b:
                print('xrun on %d'%i)
                s.push_buffer(b) # discarded
            bufs[i]=b_
    # data from all sensors gathered? if not, wait a little bit and retry
    if sum([bool(b) for b in bufs])!=len(bufs):
        time.sleep(0.01) # not too lazy
        continue
    # we have data from all sensors now, let's assemble them
    rr=[t.payload2dict(b.get_data()) for b,t in zip(bufs,trsfs)]
    # data copied, return buffers
    for s,b in zip(streams,bufs): s.push_buffer(b)
    # reset buffers with valid data
    bufs=[None for s in streams]
    # print some frame information
    for r in rr: print(r['multiDeviceBusId'],r['eventNo'],r['timestamp'])

    
