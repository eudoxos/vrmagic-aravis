import numpy as np
import ctypes, struct


class VRMagicTransformer:
    def __init__(self,cam):
        dev=cam.get_device()
        dev.set_string_feature_value('TransferFormat','PROFILE_COORD16')
        dev.set_integer_feature_value('IntensityDataEnable',1)
        dev.set_integer_feature_value('CoordADataEnable',1)
        dev.set_integer_feature_value('FooterDataEnable',1)

        self.tickHz=dev.get_integer_feature_value('FooterTimestampTickFrequency')
        self.tickOffset=dev.get_integer_feature_value('IntraFooterStartOfExposureTimestampByteOffset')

        self.trsfA,self.trsfC=self.getTrsfFuncObjs(dev)

    def getTrsfFuncObjs(self,dev):
        '''
        Given Aravis device *dev*, return tuple of 2 function objects for transforming A and C coordinates from the buffer array to physical coordinates; those function accept numpy.array as input. Invalid values are replaced by np.nan.
        '''
        # HACK: convert back to uint64 (8 bytes), take only 4, convert to signed int32
        # see https://github.com/AravisProject/aravis/issues/147
        asn,asd,aon,aod,csn,csd,con,cod=[struct.unpack('i',struct.pack('Q',dev.get_integer_feature_value(n))[:4])[0] for n in ['Scan3dCoordinateAScale_Numerator','Scan3dCoordinateAScale_Denominator','Scan3dCoordinateAOffset_Numerator','Scan3dCoordinateAOffset_Denominator','Scan3dCoordinateCScale_Numerator','Scan3dCoordinateCScale_Denominator','Scan3dCoordinateCOffset_Numerator','Scan3dCoordinateCOffset_Denominator']]
        cs,co,as_,ao=csn/csd,con/cod,asn/asd,aon/aod
        invalid=dev.get_integer_feature_value('Scan3dInvalidDataValue')
        def trsfA(a16):
            ret=as_*a16+ao
            ret[a16==invalid]=np.nan
            return ret
        def trsfC(c16):
            ret=cs*c16+co
            ret[c16==invalid]=np.nan
            return ret
        return trsfA,trsfC
    def timestampFromFooter(self,footer):
        tick=struct.unpack('Q',footer[self.tickOffset:self.tickOffset+8])[0]
        return tick/self.tickHz
    def payload2dict(self,payload,retRaw=False):
        '''
        Convert buffer data (as bytes object) to dictionary containing structured data.
        '''
        arr=np.ctypeslib.as_array(ctypes.cast(payload,ctypes.POINTER(ctypes.c_int16)),(len(payload)//2,)).copy()
        c16,a16,i16,footer=arr[:2048],arr[2048:4096],arr[4096:6144],arr[6144:].tobytes()
        ret=dict(A=self.trsfA(a16),C=self.trsfC(c16),intensity=i16,timestamp=self.timestampFromFooter(footer))
        if retRaw: ret['c16'],ret['a16'],ret['i16']=c16,a16,i16
        return ret


