#include<string>
#include<boost/core/noncopyable.hpp>
#include<Eigen/Core>
#include<iostream>
#include<limits>

extern "C" {
	#include<arv.h>
	#include<gio/gio.h>
}

typedef Eigen::Array<uint16_t,Eigen::Dynamic,1> ArrayXui16;
typedef Eigen::Array<int16_t,Eigen::Dynamic,1> ArrayXi16;

/*
Convenience wrapper for accessing VRMagic LineSensor3D via Aravis.
The wrapper is not meant to be watertight, many things need to be done with
Aravis directly -- see example on how to use.

The convenience functions cover:
* basic setup (in the ctor)
* query parameters of data layout and others (in the ctor)
* data extraction and transformation

*/
class VRMagicLineScan3D: public boost::noncopyable{
public:
	// populated automatically in the ctor
	ArvCamera* cam;
	ArvDevice* dev;
	// must be created manually, see examples
	ArvStream* stream;

	// generic parameters: constants queried from the device
	int tickHz;
	int16_t invalidVal;
	float aScale,aOff,cScale,cOff;

	// generic parameters, dependent on layout (like ROI etc), initially queried from the device
	int32_t imgWd, imgHt;
	int32_t imageSensorWidth;
	size_t payloadSize;

	// offsets inside payload, queried from the device
	struct PayloadOffsets{
		uint16_t dataAOff, dataCOff, dataIOff, footerOff;
	};
	PayloadOffsets poff;

	// offsets inside footer, queried from the device
	struct FooterOffsets{
		uint16_t encoderPositionOff;
		uint16_t eventNumberOff;
		uint16_t exposureDurationOff;
		uint16_t frameCounterOff;
		uint16_t gateNumberOff;
		uint16_t mdbDeviceIdOff;
		uint16_t startOfExposureTimestampOff;
		uint16_t triggerPipelineNumberOff;
	};
	FooterOffsets foff;

	// struct to hold extracted footer data, via getFooterData
	struct FooterData{
		uint32_t encoderPosition;
		uint32_t eventNumber;
		uint64_t exposureDuration;
		uint16_t frameCounter;
		uint16_t gateNumber;
		uint16_t mdbDeviceId;
		uint64_t startOfExposureTimestamp;
		uint16_t triggerPipelineNumber;
	};


	VRMagicLineScan3D(const char* id){
		/* ?? copied blindly from qarv */
		#if !GLIB_CHECK_VERSION(2, 35, 1)
			g_type_init();
		#endif
		cam=arv_camera_new(id);
		dev=arv_camera_get_device(cam);
		basicCameraSetup();
		// query parameters
		queryPayloadLayout(); // these can change with ROI changes
		queryFooterLayout(); // the rest never changes (presumably)
		queryScalingParams();
		queryGeneralParams();
	};

	template<typename T>
	T binExtract(const char* buf, size_t off){ return *(T*)(buf+off); }

	FooterData getFooterData(const char* data){
		FooterData ret;
		const auto& f(poff.footerOff);
		ret.encoderPosition=binExtract<uint32_t>(data,f+foff.encoderPositionOff);
		ret.eventNumber=binExtract<uint32_t>(data,f+foff.eventNumberOff);
		ret.exposureDuration=binExtract<uint64_t>(data,f+foff.exposureDurationOff);
		ret.frameCounter=binExtract<uint16_t>(data,f+foff.frameCounterOff);
		ret.gateNumber=binExtract<uint16_t>(data,f+foff.gateNumberOff);
		ret.mdbDeviceId=binExtract<uint16_t>(data,f+foff.mdbDeviceIdOff);
		ret.startOfExposureTimestamp=binExtract<uint64_t>(data,f+foff.startOfExposureTimestampOff);
		ret.triggerPipelineNumber=binExtract<uint64_t>(data,f+foff.triggerPipelineNumberOff);
		return ret;
	};
	Eigen::Map<ArrayXi16> rawA(const char* data){
		return Eigen::Map<ArrayXi16> ((int16_t*)(data+poff.dataAOff),imageSensorWidth);
	}
	Eigen::Map<ArrayXi16> rawC(const char* data){
		return Eigen::Map<ArrayXi16> ((int16_t*)(data+poff.dataCOff),imageSensorWidth);
	}
	Eigen::Map<ArrayXui16> rawI(const char* data){
		return Eigen::Map<ArrayXui16> ((uint16_t*)(data+poff.dataIOff),imageSensorWidth);
	}


	void basicCameraSetup(){
		// basic setup: these influence data format, thus must come first
		arv_device_set_string_feature_value(dev,"TransferFormat","PROFILE_COORD16");
		arv_device_set_integer_feature_value(dev,"IntensityDataEnable",1);
		arv_device_set_integer_feature_value(dev,"CoordADataEnable",1);
		arv_device_set_integer_feature_value(dev,"FooterDataEnable",1);
	}
	/* this one MUST be called whenever ROI changes -- data layout will change as well */
	void queryPayloadLayout(){
		payloadSize=arv_camera_get_payload(cam);
		// payload offsets
		poff.dataAOff=arv_device_get_integer_feature_value(dev,"CoordADataByteOffset");
		poff.dataCOff=arv_device_get_integer_feature_value(dev,"CoordCDataByteOffset");
		poff.dataIOff=arv_device_get_integer_feature_value(dev,"IntensityDataByteOffset");
		poff.footerOff=arv_device_get_integer_feature_value(dev,"FooterDataByteOffset");
		// image format
		imgWd=arv_device_get_integer_feature_value(dev,"ImageWidth"); // quite useless, actually
		imgHt=arv_device_get_integer_feature_value(dev,"ImageHeight"); // just for checking
		imageSensorWidth=arv_device_get_integer_feature_value(dev,"ImageSensorWidth");
		// coordinate transformation params
		// runtime check?
		if(imgHt!=1) throw std::runtime_error("Camera reports image with non-unit height "+std::to_string(imgHt)+"?");
	};
	/* these are constant, calling once is enough */
	void queryFooterLayout(){
		foff.eventNumberOff=arv_device_get_integer_feature_value(dev,"IntraFooterEventNumberByteOffset");
		foff.exposureDurationOff=arv_device_get_integer_feature_value(dev,"IntraFooterExposureDurationByteOffset");
		foff.frameCounterOff=arv_device_get_integer_feature_value(dev,"IntraFooterFrameCounterByteOffset");
		foff.mdbDeviceIdOff=arv_device_get_integer_feature_value(dev,"IntraFooterMDBDeviceIDByteOffset");
		foff.startOfExposureTimestampOff=arv_device_get_integer_feature_value(dev,"IntraFooterStartOfExposureTimestampByteOffset");
	}
	/* these are constant as well */
	void queryScalingParams(){
		int32_t cf[8];
		const char* nm[8]={"Scan3dCoordinateAScale_Numerator","Scan3dCoordinateAScale_Denominator","Scan3dCoordinateAOffset_Numerator","Scan3dCoordinateAOffset_Denominator","Scan3dCoordinateCScale_Numerator","Scan3dCoordinateCScale_Denominator","Scan3dCoordinateCOffset_Numerator","Scan3dCoordinateCOffset_Denominator"};
		for(int i=0;i<8;i++){
			int64_t tmp=arv_device_get_integer_feature_value(dev,nm[i]);
			if(tmp>=(1L<<31)) throw std::runtime_error("Signed int32 bug in Aravis detected (workaround not implemented). See https://github.com/AravisProject/aravis/issues/147 for details. Update Aravis and recompile. (parameter is "+std::string(nm[i])+", reported value "+std::to_string(tmp)+")");
			cf[i]=(int32_t)tmp;
		}
		aScale=cf[0]*1.f/cf[1]; aOff=cf[2]*1.f/cf[3];
		cScale=cf[4]*1.f/cf[5]; cOff=cf[6]*1.f/cf[7];
	};
	/* these are constant */
	void queryGeneralParams(){
		invalidVal=arv_device_get_integer_feature_value(dev,"Scan3dInvalidDataValue");
		tickHz=arv_device_get_integer_feature_value(dev,"FooterTimestampTickFrequency");
	}

	const float NaNf=std::numeric_limits<float>::signaling_NaN(); // convenience

	Eigen::ArrayXf trsfGeneric(const ArrayXi16& src, float scale, float offset){
		const auto valid=(src!=invalidVal);
		Eigen::ArrayXf ret=src.cast<float>()*scale+offset;
		return valid.select(ret,NaNf);
	}
	Eigen::ArrayXf trsfA(const ArrayXi16& a0){ return trsfGeneric(a0,aScale,aOff); }
	Eigen::ArrayXf trsfC(const ArrayXi16& c0){ return trsfGeneric(c0,cScale,cOff); }
};

