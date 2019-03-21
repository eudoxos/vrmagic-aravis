import gi
gi.require_version('Aravis','0.6')
from gi.repository import Aravis
cam=Aravis.Camera()
dev=cam.get_device()
ps=dev.get_integer_feature_value('GevSCPSPacketSize')
print('Packet size:',ps)
