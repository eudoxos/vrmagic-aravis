#include"vrmagic-sensor.hpp"
#include<memory>
#include<boost/core/noncopyable.hpp>
#include<QApplication>
#include<QThread>
#include<QChart>
#include<QScatterSeries>
#include<QValueAxis>
#include<QMainWindow>
#include<QChartView>

QT_CHARTS_USE_NAMESPACE

const float NaNf=std::numeric_limits<float>::signaling_NaN(); // convenience
using std::isnan;

struct CookedFrameData: public boost::noncopyable{
	Eigen::ArrayXf xx,zz;
	ArrayXui16 ii;
	VRMagicLineScan3D::FooterData footerData;
};
// enable smuggling this type through signal
Q_DECLARE_METATYPE(std::shared_ptr<CookedFrameData>);

/*
Two good resources for threading in Qt:
https://www.vikingsoftware.com/how-to-use-qthread-properly/
https://nachtimwald.com/2015/05/02/effective-threading-using-qt/
*/

/*
This is the acquisition loop which will be emitting signals (lineAcquired)
whenever new frame arrives from the sensor. This signal will be received
by the Chart class (below) which will update plot data.
*/
class SensorWorker: public QObject{	
	Q_OBJECT
	public:
		/* ctor of VRMagicLineScan3D does most of the hard work for us */
		SensorWorker(const char* id): sensor(VRMagicLineScan3D(id)){};
	public slots:
		void stop(){ running=false; }
		void loop(); // define below
	signals:
		// this implementation is generated by MOC automatically, that's why we need to #include"live-plot.moc" below
		void lineAcquired(const std::shared_ptr<CookedFrameData>& data);
		void finished();
	private:
		VRMagicLineScan3D sensor;
		std::atomic<bool> running=true;
};

void SensorWorker::loop(){
	qInfo("Entering sensor loop.");
	// setup camera stream and buffers
	sensor.stream=arv_camera_create_stream(sensor.cam,NULL,NULL);
	arv_device_set_boolean_feature_value(sensor.dev,"TrigInMultiDeviceBusEnable",0);
	for(int i=0; i<10; i++){
		arv_stream_push_buffer(sensor.stream,arv_buffer_new(sensor.payloadSize,NULL));
	}
	arv_camera_start_acquisition(sensor.cam);
	while(running){
		// keep everything responsive
		QCoreApplication::instance()->processEvents();
		// try to get buffer, which will be non-NULL if there is data
		ArvBuffer *buf=arv_stream_try_pop_buffer(sensor.stream);
		if(buf==NULL){
			// no data, sleep for a little and try again
			QThread::currentThread()->msleep(20);
			continue;
		}
		// got buffer, check the status; discard bad ones
		ArvBufferStatus status=arv_buffer_get_status(buf);
		if(status!=ARV_BUFFER_STATUS_SUCCESS) continue;
		// decode buffer, copy data away
		size_t dataLen;
		const char* data=(const char*)arv_buffer_get_data(buf,&dataLen);
		auto fr=std::make_shared<CookedFrameData>();
		// save transformed data so that we can release the buffer
		fr->xx=sensor.trsfA(sensor.rawA(data));
		fr->zz=sensor.trsfC(sensor.rawC(data));
		fr->ii=sensor.rawI(data);
		fr->footerData=sensor.getFooterData(data);
		// original data not needed anymore, return the buffer to aravis
		arv_stream_push_buffer(sensor.stream,buf);
		// use signal to send data (in CookedFrameData) away
		emit lineAcquired(fr);
	}
	arv_camera_stop_acquisition(sensor.cam);
	/* TODO: delete buffers etc */
	emit finished();
}

/*
Displays the profile data. Inspiration from this example:
https://doc.qt.io/qt-5/qtcharts-dynamicspline-example.html
*/
class Chart: public QChart{
	Q_OBJECT
	QScatterSeries* series;
	QValueAxis* axis_x;
	// use persistent minima and maxima
	float xMin=NaNf, xMax=NaNf, yMin=NaNf, yMax=NaNf;
	public:
		Chart(QGraphicsItem *parent=0, Qt::WindowFlags wFlags=0): QChart(QChart::ChartTypeCartesian,parent,wFlags), axis_x(new QValueAxis) {
			series=new QScatterSeries(this);
			addSeries(series);
			createDefaultAxes();
			setAxisX(axis_x,series);
			series->setUseOpenGL(true);
			series->setMarkerSize(5);
		}
		virtual ~Chart(){}
	public slots:
		// this slot receives the SensorWorker::lineAcquired signal
		void displayNewLine(const std::shared_ptr<CookedFrameData>& d){	
			QList<QPointF> pts; pts.reserve(d->xx.size());
			for(size_t i=0; i<d->xx.size(); i++){
				const auto& x(d->xx[i]); const auto& z(d->zz[i]);
				if(isnan(x)||isnan(z)) continue;
				// adjust min/max as needed
				if(isnan(xMin) || x<xMin) xMin=x;
				if(isnan(xMax) || x>xMax) xMax=x;
				if(isnan(yMin) || z<yMin) yMin=z;
				if(isnan(yMax) || z>yMax) yMax=z;
				pts.append(QPointF(x,z));
			}
			series->replace(pts);
			axisX()->setRange(xMin,xMax);
			axisY()->setRange(yMin,yMax);
		}
};

// MOC-generated stuff
#include "live-plot.moc"

int main(int argc, char* argv[]){
	QApplication app(argc, argv);
	// enable sending through signal (complements Q_DECLARE_METATYPE above)
	qRegisterMetaType<std::shared_ptr<CookedFrameData>>();

	// create the GUI
	QMainWindow window;
	Chart *chart=new Chart;
	chart->setTitle("LineSensor3D live data");
	chart->legend()->hide();
	chart->setAnimationOptions(QChart::NoAnimation);
	QChartView chartView(chart);
	chartView.setRenderHint(QPainter::Antialiasing);
	window.setCentralWidget(&chartView);
	window.resize(800,600);

	// now the real stuff
	// acquisition will run in this thread (not the same one as the GUI)
	auto sensorThread=new QThread;
	// construct the sensor worker, which contains the sensor object as well
	auto sensorWorker=new SensorWorker("VRmagicImaging-EFD0AFMH07");
	// the worker is moved to that separate thread
	sensorWorker->moveToThread(sensorThread);

	// connect signals to slots
	// 1. worker should start with the thread
	QObject::connect(sensorThread,&QThread::started,sensorWorker,&SensorWorker::loop);
	// 2. after line is acquired, trigger plot update with new data
	QObject::connect(sensorWorker,&SensorWorker::lineAcquired,chart,&Chart::displayNewLine);
	// 3. stop worker when closing the window
	// FIXME: get the syntax right??
	// QObject::connect(window,&QMainWindow::closeEvent,[&sensorWorker](QCloseEvent* ev){ sensorWorker->stop(); });
	// 4. stop the thread if the worker finishes
	QObject::connect(sensorWorker,&SensorWorker::finished,sensorThread,&QThread::quit);
	#if 0
		// 5. debugging: show dot for each frame received
	   QObject::connect(sensorWorker,&SensorWorker::lineAcquired,
        [](const std::shared_ptr<CookedFrameData>& d){ std::cerr<<"."; /* qInfo("Frame %d, event %d",d->footerData.frameCounter,d->footerData.eventNumber); */ }
		);
	#endif

	// start data acquisition
	sensorThread->start();
	// show the window
	window.show();
	// start Qt event loop (returns when app closes)
	return app.exec();
}
