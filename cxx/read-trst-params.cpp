/*
	show values and ranges of coordinate transformation constants;
	Aravis probably ignores (see https://github.com/AravisProject/aravis/issues/147)
	the field width, leading to reporting bad values in some cases
*/


#include<arv.h>
#include<iostream>
int main(int argc, char** argv){
	ArvCamera *camera;
	ArvDevice *device;
	camera=arv_camera_new(argc>1?argv[1]:NULL);
	device=arv_camera_get_device(camera);
	for(const auto& f:{"Scan3dCoordinateAScale_Numerator","Scan3dCoordinateAScale_Denominator","Scan3dCoordinateAOffset_Numerator","Scan3dCoordinateAOffset_Denominator","Scan3dCoordinateCScale_Numerator","Scan3dCoordinateCScale_Denominator","Scan3dCoordinateCOffset_Numerator","Scan3dCoordinateCOffset_Denominator"}){
		gint64 v=arv_device_get_integer_feature_value(device,f);
		gint64 lo,hi;
		arv_device_get_integer_feature_bounds(device,f,&lo,&hi);
		std::cout<<f<<": "<<v<<" ["<<*(gint32*)(&v)<<"] ("<<lo<<"..."<<hi<<")"<<std::endl;
	}
	g_clear_object (&camera);
	return 0;
}
