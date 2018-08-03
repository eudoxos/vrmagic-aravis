#include<string>
#include"vrmagic-sensor.hpp"
#include<boost/multi_array.hpp>
#include<chrono>
#include<thread>
#include<vector>
#include<mutex>

using std::string, std::to_string, std::shared_ptr, std::vector;
typedef VRMagicLineScan3D::CookedFrameData CookedFrameData;

/*
Circular buffer for storing synchronized data. Event number is modular
(evNoMod), not monotonic. So the arithmetics is double-modular, one for
buffer lines (rows) and one for event number (evNoMod), both set by
the constructor.

Based on template params, the struct can be accessed from

(a) single thread (with locking=false), where no locking is involved,

(b) multiple threads (with locking=true), where each call of the public
interface (pushBuf and popBuf) is protected with recursive mutex.

It seems multi-threading is not faster than single-thread polling all sensors
(and avoiding locking). Anyway, single-thread takes about 2% of CPU
(with optimized build) and 72% or that is spent in Eigen ops transforming data.

*/
template<bool locking=false>
class EvSyncBuf{
	#define _EV_SYNC_BUF__OPTIONAL_SCOPED_LOCK std::unique_ptr<std::scoped_lock<std::mutex>> lock; if constexpr(locking){ lock=std::make_unique<std::scoped_lock<std::mutex>>(mutex_); }
	std::mutex mutex_;
	const uint32_t rows, srcs;
	const uint32_t evNoMod;
	// storage
	boost::multi_array<shared_ptr<CookedFrameData>,2> arr;
	// lowest event number we will store; if negative, not yet initialized
	int ev0=-1;
	// row index for ev0 (will be moving)
	int ev0ix=0; 
	// count consecutive discarded packets; if too many, start using those
	int discarded=0;

	// compute row for evNo, return -1 if invalid
	int evRow(int evNo){
		if(!((evNo-ev0)%evNoMod<rows)) return -1;
		return (ev0ix+(evNo-ev0)%evNoMod)%rows;
	}
	bool evNoFilled(int evNo){
		int row=evRow(evNo);
		if(row<0) return false;
		return rowFilled(row);
	}
	bool rowFilled(int row){
		assert(row>=0 && row<rows);
		for(size_t src=0; src<srcs; src++) if(!arr[row][src]) return false;
		return true;
	}
	void moveToEvNo(int evNo){
		assert(evNo<evNoMod);
		// std::cerr<<"moveToEvNo("<<evNo<<"): old ev0="<<ev0<<", ev0ix="<<ev0ix<<std::endl;
		for(int e=ev0; e!=evNo; e=(e+1)%evNoMod){
			int r=evRow(e);
			if(r>=0){
				for(int s=0; s<srcs; s++){
					// if(arr[r][s]) std::cerr<<"-- resetting ["<<r<<","<<s<<"] (evNo "<<arr[r][s]->footerData.eventNumber<<")"<<std::endl;
					arr[r][s].reset();
				}
			}
		}
		ev0ix=(ev0ix+(evNo-ev0)%evNoMod)%rows;
		ev0=evNo;
		///std::cerr<<"moteToEvNo("<<evNo<<"): new ev0="<<ev0<<", ev0ix="<<ev0ix<<std::endl;
	};
	bool pushBuf_impl(const shared_ptr<CookedFrameData>& d, int evNo, int src){
		assert(evNo>=0);
		if(ev0<0) ev0=evNo; // first call ever
		int row=evRow(evNo);
		if(row>=0){
			discarded=0;
			// okay, write data
			// std::cerr<<"++ writing to ["<<row<<","<<src<<"]"<<std::endl;
			if(arr[row][src]){
				std::cerr<<"ERROR: #"<<src<<", evNo="<<evNo<<": data slot not empty, discarding old data."<<std::endl;
				arr[row][src].reset();
			}
			assert(!arr[row][src]); // should be empty
			arr[row][src]=d;
			return rowFilled(row);
		}
		uint32_t fwdDist=(evNo-ev0)%evNoMod;
		if(fwdDist>evNoMod/2 && discarded<20){
			// recent past or very distant future, discard
			std::cerr<<"WARN: discarding recent past (or distant future) #"<<src<<", evNo="<<evNo<<", ev0="<<ev0<<"."<<std::endl;
			discarded++;
			return false;
		} else {
			// distant past or very near future, move ahead
			std::cerr<<"WARN: moving to near future (or distant past) event #"<<src<<", evNo="<<evNo<<", ev0="<<ev0<<", fwdDist="<<fwdDist<<"."<<std::endl;
			moveToEvNo(evNo);
			return pushBuf_impl(d,evNo,src);
		}
	}
	public:
		EvSyncBuf(int rows_, int srcs_, int evNoMod_=(1L<<16)): rows(rows_), srcs(srcs_), evNoMod(evNoMod_), arr(boost::extents[rows_][srcs_]){}
		/*
		Push buffer with evNo event number;
		Returns true if the event is gathered already (data from all sensors present), false otherwise
		*/
		bool pushBuf(const shared_ptr<CookedFrameData>& d, int evNo, int src){
			_EV_SYNC_BUF__OPTIONAL_SCOPED_LOCK
			return pushBuf_impl(d,evNo,src);
		};
		/*
		Pop buffer for evNo, returned as vector.
		*/
		vector<shared_ptr<CookedFrameData>> popBuf(int evNo){
			_EV_SYNC_BUF__OPTIONAL_SCOPED_LOCK
			vector<shared_ptr<CookedFrameData>> ret;
			int row=evRow(evNo);
			ret.reserve(srcs);
			if(row<0){
				std::cerr<<"WARN: attempt to pop invalid buffer, evNo="<<evNo<<", ev0="<<ev0<<"."<<std::endl;
				return ret;
			}
			for(size_t src=0; src<srcs; src++){
				if(!arr[row][src]) std::cerr<<"ERROR: arrmpting to pop buffer which is not gathered yet (evNo="<<evNo<<", data missing from source #"<<src<<")"<<std::endl;
				ret.push_back(arr[row][src]);
			}
			moveToEvNo((evNo+1)%evNoMod);
			return ret;
		};
};

// define to change modulo of the event value (normally is 1<<16=65536)
#define FAKE_EV_NO_MOD (1<<7)

// define to process each sensor in separate thread (via OpenMP)
#define ACQUIRE_PARALLEL


/*
This funcs tries to get data from sensor and acts accordingly.
If no valid buffer is present, returns false (and the caller should sleep a bit).
If valid buffer is present, it is pushed into the gatherer. If all buffers
from the event are gathered, they are popped from the gatherer (and currently
discarded; later, they would be handed to the controlling thread as new data).
*/
template<bool locking>
bool tryGetBuffer(VRMagicLineScan3D& sensor, int i, EvSyncBuf<locking>& gatherer){
	ArvBuffer *buf=arv_stream_try_pop_buffer(sensor.stream);
	if(!buf) return false;
	ArvBufferStatus status=arv_buffer_get_status(buf);
	if(status!=ARV_BUFFER_STATUS_SUCCESS) return false;
	// copy data
	size_t dataLen;
	const char* data=(const char*)arv_buffer_get_data(buf,&dataLen);
	auto cookedData=sensor.cookData(data);
	// release buffer
	arv_stream_push_buffer(sensor.stream,buf);
	#ifdef FAKE_EV_NO_MOD
		auto& evNo=cookedData->footerData.eventNumber;
		evNo+=2*i;
		evNo%=FAKE_EV_NO_MOD;
	#else
		const auto evNo=cookedData->footerData.eventNumber;
	#endif
	//std::cerr<<"Pushing #"<<i<<", evNo="<<evNo<<std::endl;
	bool gathered=gatherer.pushBuf(cookedData,evNo,i);
	if(gathered){
		auto g=gatherer.popBuf(evNo);
		if(evNo%10==0){
			std::cerr<<"INFO: Frame gathered, event numbers:";
			for(int i=0; i<g.size(); i++) std::cerr<<" "<<g[i]->footerData.eventNumber;
			std::cerr<<std::endl;
		}
	}
	return true;
}


int main(int argc, char** argv){
	// setup sensors
	VRMagicLineScan3D s0("VRmagicImaging-EFD0EEY3Y5");
	VRMagicLineScan3D s1("VRmagicImaging-EFD0AFMH07");
	const int nSensors=2;
	VRMagicLineScan3D* sensors[nSensors]={&s0,&s1};
	for(int i=0; i<nSensors; i++){
		auto& s(*sensors[i]);
		// stream & buffers
		s.stream=arv_camera_create_stream(s.cam,NULL,NULL);
		if(!s.stream) throw std::runtime_error("Error creating stream for sensor #"+to_string(i)+" (busy?).");
		for(int i=0; i<10; i++){
			arv_stream_push_buffer(s.stream,arv_buffer_new(s.payloadSize,NULL));
		}
		// i==0 is master
		arv_device_set_boolean_feature_value(s.dev,"TrigOutMultiDeviceBusEnable",i==0?1:0);
		// i>0 are slaves
		arv_device_set_boolean_feature_value(s.dev,"TrigInMultiDeviceBusEnable",i==0?0:1);
	}
	// set master's frame rate
	arv_device_set_integer_feature_value(sensors[0]->dev,"AOIHeight",200);
	arv_device_set_integer_feature_value(sensors[1]->dev,"AOIHeight",1000);
	sensors[0]->queryPayloadLayout();
	sensors[1]->queryPayloadLayout();
	arv_device_set_integer_feature_value(sensors[0]->dev,"AcquisitionFrameRate_mHz",200000);
	// start all cameras
	for(int i=0;i<nSensors;i++) arv_camera_start_acquisition(sensors[i]->cam);

	#ifdef ACQUIRE_PARALLEL
		const bool EvSyncBuf_locking=true;
	#else
		const bool EvSyncBuf_locking=false;
	#endif

	#ifdef FAKE_EV_NO_MOD
		EvSyncBuf<EvSyncBuf_locking> gatherer(10,nSensors,FAKE_EV_NO_MOD);
	#else
		EvSyncBuf<EvSyncBuf_locking> gatherer(10,nSensors,/*evNoMod*/(1<<16));
	#endif

	// single-threaded
	#ifndef ACQUIRE_PARELLEL
		while(true){
			int n=0;
			for(int i=0;i<nSensors;i++){
				if(tryGetBuffer(*sensors[i],i,gatherer)) n++;
			}
			if(n==0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	#else
		#pragma omp parallel for num_threads(nSensors)
		for(int i=0; i<nSensors; i++){
			while(true){
				if(tryGetBuffer(*sensors[i],i,gatherer)) continue;
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
	#endif
};
