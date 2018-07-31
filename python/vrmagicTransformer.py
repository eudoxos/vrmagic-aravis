import numpy as np
import ctypes, struct, warnings


class VRMagicTransformer:
    def __init__(self,cam):
        '''
        The constructor initializes the sensor (transfer coordinates rather than images, include intensity, lateral coordinate and footer data) and reads coordinate transformation constants. 
        '''
        dev=cam.get_device()
        # device setup
        dev.set_string_feature_value('TransferFormat','PROFILE_COORD16')
        dev.set_integer_feature_value('IntensityDataEnable',1)
        dev.set_integer_feature_value('CoordADataEnable',1)
        dev.set_integer_feature_value('FooterDataEnable',1)
        # get constants
        self.tickHz=dev.get_integer_feature_value('FooterTimestampTickFrequency')
        self.tickOffset=dev.get_integer_feature_value('IntraFooterStartOfExposureTimestampByteOffset')
        self.invalid=dev.get_integer_feature_value('Scan3dInvalidDataValue')
        self.triggerNoOffset=dev.get_integer_feature_value('IntraFooterTriggerPipelineNumberByteOffset')
        self.multiDeviceBusIdOffset=dev.get_integer_feature_value('IntraFooterMDBDeviceIDByteOffset')
        self.eventNoOffset=dev.get_integer_feature_value('IntraFooterEventNumberByteOffset')

        feats=[dev.get_integer_feature_value(f) for f in ['Scan3dCoordinateAScale_Numerator','Scan3dCoordinateAScale_Denominator','Scan3dCoordinateAOffset_Numerator','Scan3dCoordinateAOffset_Denominator','Scan3dCoordinateCScale_Numerator','Scan3dCoordinateCScale_Denominator','Scan3dCoordinateCOffset_Numerator','Scan3dCoordinateCOffset_Denominator']]
        # do we need the bug workaround? see https://github.com/AravisProject/aravis/issues/147
        if max(feats)>=2**31: 
            # HACK: convert back to uint64 (8 bytes), take only 4, convert to signed int32
            warnings.warn('Signed int32 bug in Aravis detected, activating workaround. See https://github.com/AravisProject/aravis/issues/147 for details and update Aravis.')
            asn,asd,aon,aod,csn,csd,con,cod=[struct.unpack('i',struct.pack('Q',f)[:4])[0] for n in feats]
        else: asn,asd,aon,aod,csn,csd,con,cod=feats
        self.cs,self.co,self.as_,self.ao=csn/csd,con/cod,asn/asd,aon/aod
    def trsfA(self,a16):
        'Transform raw uint32 a-coordinate to physical coordinates; invalid values are automatically replaced by nan. Operates on numpy.array.'
        ret=self.as_*a16+self.ao
        ret[a16==self.invalid]=np.nan
        return ret
    def trsfC(self,c16):
        'Transform raw uint32 c-coordinate to physical coordinates; invalid values are automatically replaced by nan. Operates on numpy.array.'
        ret=self.cs*c16+self.co
        ret[c16==self.invalid]=np.nan
        return ret
    def timestampFromFooter(self,footer):
        tick=struct.unpack('Q',footer[self.tickOffset:self.tickOffset+8])[0]
        return tick/self.tickHz
    def triggerIdFromFooter(self,footer):
        return struct.unpack('H',footer[self.triggerNoOffset:self.triggerNoOffset+2])[0]
    def multiDeviceBusIdFromFooter(self,footer):
        return struct.unpack('H',footer[self.multiDeviceBusIdOffset:self.multiDeviceBusIdOffset+2])[0]
    def eventNoFromFooter(self,footer):
        return struct.unpack('H',footer[self.eventNoOffset:self.eventNoOffset+2])[0]
    def payload2dict(self,payload,retRaw=False):
        '''
        Convert buffer data (as bytes object) to dictionary containing structured data.
        '''
        arr=np.ctypeslib.as_array(ctypes.cast(payload,ctypes.POINTER(ctypes.c_int16)),(len(payload)//2,)).copy()
        c16,a16,i16,footer=arr[:2048],arr[2048:4096],arr[4096:6144],arr[6144:].tobytes()
        ret=dict(A=self.trsfA(a16),C=self.trsfC(c16),intensity=i16,timestamp=self.timestampFromFooter(footer),multiDeviceBusId=self.multiDeviceBusIdFromFooter(footer),eventNo=self.eventNoFromFooter(footer))
        if retRaw: ret['c16'],ret['a16'],ret['i16']=c16,a16,i16
        return ret


