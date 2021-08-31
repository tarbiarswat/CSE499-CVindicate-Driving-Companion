#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

//Image Processing variables

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate;
Mat ROILane;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result;

RaspiCam_Cv Camera;

stringstream ss;

vector<int> histrogramLane;

Point2f Source[] = {Point2f(100,135),Point2f(355,135),Point2f(15,185), Point2f(355,185)};
Point2f Destination[] = {Point2f(60,0),Point2f(300,0),Point2f(60,240), Point2f(300,240)};

//Machine Learning variables
CascadeClassifier Stop_Cascade, Object_Cascade; 
Mat frame_Stop, RoI_Stop, gray_Stop, frame_Object, RoI_Object, gray_Object;
vector<Rect> Stop, Object;
int dist_Stop, dist_Object;

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
{
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,70 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,150 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}


void Capture()
{
    Camera.grab();
    Camera.retrieve(frame);
    cvtColor(frame, frame, COLOR_BGR2RGB);
    cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
    cvtColor(frame, frame_Object, COLOR_BGR2RGB);
}


void Perspective()
{
	line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
	line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
	line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
	
       
	
	Matrix = getPerspectiveTransform(Source, Destination);
	warpPerspective(frame, framePers, Matrix, Size(400,240));
}

void Threshold()
{
    cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
    inRange(frameGray, 190, 255, frameThresh);
    Canny(frameGray, frameEdge, 600, 700, 3, false);
    add(frameThresh, frameEdge, frameFinal);
    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
    cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);
}

void Histrogram()
{
    histrogramLane.resize(400);
    histrogramLane.clear();
    
    for(int i=0; i<400; i++)      //frame.size().width = 400
    {
	ROILane = frameFinalDuplicate(Rect(i,140,1,100));
	divide(255, ROILane, ROILane);
	histrogramLane.push_back((int)(sum(ROILane) [0]));
    }
}

void LaneFinder()
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr);
    
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin() + 250, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
    
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0,255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);
}

void LaneCenter()
{
    laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    frameCenter = 179;
    
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);

    Result = laneCenter-frameCenter;
}


void stop_detection()
{
   Stop_Cascade.load("//home//pi//CSE499-CVindicate-Driving-Companion//DatasetsV2//classifier//Stop_cascade.xml");
  
    
    RoI_Stop = frame_Stop(Rect(200,0,200,140));
    cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(gray_Stop, gray_Stop);
    Stop_Cascade.detectMultiScale(gray_Stop, Stop);
    
    for(int i=0; i<Stop.size(); i++)
    {
	Point P1(Stop[i].x, Stop[i].y);
	Point P2(Stop[i].x + Stop[i].width, Stop[i].x + Stop[i].height);
	
	rectangle(RoI_Stop, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 1.5);
	dist_Stop = (-1.0)*(P2.x-P1.x)+ 105;
	ss.str("");
	ss.clear();
	ss<<"Distance= "<<dist_Stop<<"(cm)";
	putText(RoI_Stop, ss.str(), Point2f(1,130), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1.5);
    }
}


void Object_detection()
{
   Object_Cascade.load("//home/pi//CSE499-CVindicate-Driving-Companion//ObjectDatasetsV3//classifier//Object_cascade.xml");
  
    
    RoI_Object = frame_Object(Rect(50,50,200,140));
    cvtColor(RoI_Object, gray_Object, COLOR_RGB2GRAY);
    equalizeHist(gray_Object, gray_Object);
    Object_Cascade.detectMultiScale(gray_Object, Object);
    
    for(int i=0; i<Object.size(); i++)
    {
	Point P1(Object[i].x, Object[i].y);
	Point P2(Object[i].x + Object[i].width, Object[i].x + Object[i].height);
	
	rectangle(RoI_Object, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Object, "Object", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 1.5);
	dist_Object = (-1.4545)*(P2.x-P1.x)+ 177.8;
	ss.str("");
	ss.clear();
	ss<<"Distance= "<<dist_Object<<"cm";
	putText(RoI_Object, ss.str(), Point2f(1,130), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1.5);
    }
}



int main(int argc, char **argv)
{
	
    wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
    Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	if (!Camera.open())	
	{
		cout<<"Failed to Connect"<<endl;
		return -1;
    }
    cout<<"Camera Id = "<<Camera.getId()<<endl;
    
    while(1)
    {
	auto start = std::chrono::system_clock::now();
	Capture();
	Perspective();
	Threshold();
	Histrogram();
	LaneFinder();
	LaneCenter();
	stop_detection();
	Object_detection();
	
	if (dist_Stop > 5 && dist_Stop < 20)
	{
	    digitalWrite(21, 0);
	    digitalWrite(22, 0);
	    digitalWrite(23, 0);	//decimal=8
	    digitalWrite(24, 1);
	    cout<<"Left3"<<endl;
	    
	    goto Stop_sign;
	    }
	
	if (Result ==0)
	{
	    digitalWrite(21, 0);
	    digitalWrite(22, 0);
	    digitalWrite(23, 0);	//decimal=0
	    digitalWrite(24, 0);
	    cout<<"Forward"<<endl;
	}
	
	else if (Result >=0 && Result <10)
	{
	    digitalWrite(21, 1);
	    digitalWrite(22, 0);
	    digitalWrite(23, 0);	//decimal=1
	    digitalWrite(24, 0);
	    cout<<"Right1"<<endl;
	}
	
	else if (Result >=10 && Result <20)
	{
	    digitalWrite(21, 0);
	    digitalWrite(22, 1);
	    digitalWrite(23, 0);	//decimal=2
	    digitalWrite(24, 0);
	    cout<<"Right2"<<endl;
	}
	
	else if (Result > 20)
	{
	    digitalWrite(21, 1);
	    digitalWrite(22, 1);
	    digitalWrite(23, 0);	//decimal=3
	    digitalWrite(24, 0);
	    cout<<"Right3"<<endl;
	}
	
	else if (Result <0 && Result >-10)
	{
	    digitalWrite(21, 0);
	    digitalWrite(22, 0);
	    digitalWrite(23, 1);	//decimal=4
	    digitalWrite(24, 0);
	    cout<<"Left1"<<endl;
	}
	
	else if (Result <=-10 && Result >-20)
	{
	    digitalWrite(21, 1);
	    digitalWrite(22, 0);
	    digitalWrite(23, 1);	//decimal=5
	    digitalWrite(24, 0);
	    cout<<"Left2"<<endl;
	}
	
	else if (Result <-20)
	{
	    digitalWrite(21, 0);
	    digitalWrite(22, 1);
	    digitalWrite(23, 1);	//decimal=6
	    digitalWrite(24, 0);
	    cout<<"Left3"<<endl;
	}
	
	Stop_sign:
	
	ss.str("");
	ss.clear();
	ss<<"Result = "<<Result;
	putText(frame, ss.str(), Point2f(1,50), 0, 1, Scalar(0,0,255), 2);
	
	namedWindow("FrontView", WINDOW_KEEPRATIO);
	moveWindow("FrontView", 0, 100);
	resizeWindow("FrontView", 640, 480);
	imshow("FrontView", frame);
	
	namedWindow("Birds Eye View", WINDOW_KEEPRATIO);
	moveWindow("Birds Eye View", 640, 100);
	resizeWindow("Birds Eye View", 640, 480);
	imshow("Birds Eye View" ,framePers);
	
	namedWindow("Final", WINDOW_KEEPRATIO);
	moveWindow("Final", 1280, 100);
	resizeWindow("Final", 640, 480);
	imshow("Final" ,frameFinal);
	
	namedWindow("StopSign", WINDOW_KEEPRATIO);
	moveWindow("StopSign", 1280, 580);
	resizeWindow("StopSign", 640, 480);
	imshow("StopSign" ,RoI_Stop);
	
	namedWindow("ObjectDetection", WINDOW_KEEPRATIO);
	moveWindow("ObjectDetection", 640, 580);
	resizeWindow("ObjectDetection", 640, 480);
	imshow("ObjectDetection" ,RoI_Object);
	
	waitKey(1);
	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	float t = elapsed_seconds.count();
	int FPS = 1/t;
	cout<<"FPS = "<<FPS<<endl;
    
	
	}
	return 0;
     
    }
