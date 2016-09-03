//============================================================================
// Name        : HB_Vision.cpp
// Author      : Ivan Ovinnikov
// Version     :
// Description : Vision ROS package for the autonomous landing system
//============================================================================

#define VISION_DEBUG 0

/* ==== Project includes ==== */

#include "Blob.h"
#include "PoseEstimation.h"
#include "OpticalFlow.h"
#include "hb_vision.h"

using namespace std;
using namespace cv;
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

/* ==== Frame to frame stuff ==== */

/* Callback counter */
int loopcounter = 0;
/* Blob and respected paths containers */
vector<Blob> ids, idsLast;
Mat last_frame;
vector<vector<Point> > paths;

/* =========================== */

/* ==== Global ROS stuff ==== */

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
// Pose publisher
ros::Publisher pose;

/* calibration file location */
const char* filename = "/home/linaro/rosws/sandbox/HB_Vision/out_camera_data.yml";

/*
 * Print published/subscribed topics
 */
void printTopicInfo()
{
    std::string nodeName = ros::this_node::getName();
    ros::V_string topics;
    ros::this_node::getSubscribedTopics(topics);
    std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
    for (unsigned int i = 0; i < topics.size(); i++)
        topicsStr += ("\t\t" + topics.at(i) + "\n");

    topicsStr += "\tadvertised topics:\n";
    ros::this_node::getAdvertisedTopics(topics);
    for (unsigned int i = 0; i < topics.size(); i++)
        topicsStr += ("\t\t" + topics.at(i) + "\n");

    ROS_INFO_STREAM(""<< topicsStr);
}

/* ====================== */

/*
 *	getCalibrationData
 *	
 *	Fetches the calibration data from the calibration file
 *	and writes it to the camera matrix and the distortion coefficients
 */
void getCalibrationData(const char * filename, Mat& camMatrix, Mat& distCoeffs)
{
	FileStorage fs(filename, FileStorage::READ);
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
}

/*
 * Filter the contours and stamp ids on them
 */
vector<Blob> BlobIdentifier(vector<vector<Point> >& contours, Mat& img)
{
	int id = 0;
	vector<Blob> out;
	for(unsigned i = 0;i<contours.size();i++)
	{
		Rect bRect = boundingRect( Mat(contours[i]) );
		Point2f center = Point(bRect.tl().x + (bRect.width / 2), bRect.tl().y + (bRect.height/2));
		vector<float> dists;
		for (size_t pointIdx = 0; pointIdx < contours[i].size(); pointIdx++)
		{
			Point2f pt = contours[i][pointIdx];
			dists.push_back(norm(center - pt));
		}
		sort(dists.begin(), dists.end());
		int radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;

		float aspectRatio = (float)bRect.height/bRect.width;

		// filter contours by color
		if(img.at<Vec3b>(center)[0] > hmin &&
				img.at<Vec3b>(center)[1] > smin &&
				img.at<Vec3b>(center)[2] > vmin)
		{
			// scan for rectangles which are big enough and have acceptable aspect ratios
			if(bRect.area() > minarea && (aspectRatio < bhigh && aspectRatio > blow))
			{
				Blob ident = Blob(contours.at(i),id,center,radius,bRect);
				out.push_back(ident);
				id++;
			}
		}
	}
	return out;
}

/*
 * bestMatch
 *
 * This function performs a matching of blobs between two frames
 * on the distance criteria. If the blobs in idFrame1 are not too
 * far apart from the blobs in idFrame2, then the ids match.
 * If the sizes of the vectors mismatch, new ids are assigned or
 * old ones are dropped.
 *
 *
 * Inputs: blobs from the first and second identified frame, distance, image
 * Output: newly identified blobs
 *
 */
vector<Blob> bestMatch(vector<Blob>& idframe1, vector<Blob> & idframe2, float dist, Mat& img)
{
	vector<Blob> out;
	int newid;
	int size1 = idframe1.size();
	int size2 = idframe2.size();

	unsigned size = min(idframe1.size(), idframe2.size());
	// blobs with IDs from last frame
	for(unsigned i = 0; i< min(idframe1.size(), idframe2.size()); i++)
	//for(vector<Blob>::iterator it = idframe1.begin();it != idframe1.end();++it)
	{
		float aspectRatio = (float)idframe2[i].bRect.height/idframe2[i].bRect.width;

		// filter contours by color
		if(img.at<Vec3b>(idframe2.at(i).center)[0] > hmin &&
				img.at<Vec3b>(idframe2.at(i).center)[1] > smin &&
				img.at<Vec3b>(idframe2.at(i).center)[2] > vmin)
		{
			// scan for rectangles which are big enough and have acceptable aspect ratios
			if(idframe2.at(i).bRect.area() > minarea && (aspectRatio < bhigh && aspectRatio > blow))
			{
				float min = 10000;
				int mindistind;

				for(unsigned j = 0; j<size; j++)
				{
					// checks the distances and returns the minimal blob index for matching
					float temp = sqrt(sqdistance(idframe1.at(i).center, idframe2.at(j).center));
					if(min>temp)
					{
						min = temp;
						mindistind = j;
					}
				}
				//newid = idframe1.at(mindistind).id; // get the id of the minimal distance
				// move the mindistind to the end of the vectors
//				Blob temp = idframe2.at(idframe2.size()-1);
//				idframe2.at(idframe2.size()-1) = idframe2.at(mindistind);
//				idframe2.at(mindistind) = temp;
//				temp = idframe1.at(idframe1.size()-1);
//				idframe1.at(idframe1.size()-1) = idframe1.at(i);
//				idframe2.at(i) = temp;
//				// remove the blob from the end of the vector
//				idframe1.pop_back();
//				idframe2.pop_back();
				//idframe2.erase (idframe2.begin()+minind);	// remove the newly id'd blob from the vector
				//idframe1.erase (idframe1.begin()+minind);
				if(sqrt(sqdistance(idframe1.at(i).center, idframe2.at(i).center)) < dist)
				{
					// keep id if the blobs aren't too far apart
					newid = idframe1[i].id;
					//cout << "Keeping ID " << newid << endl;
				}
				else
				{
					// random id again if distance too big (should not happen)
					newid = i;
					//cout << "New ID " << newid << endl;
				}

				Blob ident = Blob(idframe2[i].contour,newid,
						idframe2[i].center,idframe2[i].radius,idframe2[i].bRect);
				out.push_back(ident);
			}
		}
	}
	// new blobs without IDs yet
	if(size2>size1)
	{
		for(int i = size1; i<size2; i++)
		{
			float aspectRatio = (float)idframe2[i].bRect.height/idframe2[i].bRect.width;

			if(img.at<Vec3b>(idframe2[i].center)[0] > hmin &&
					img.at<Vec3b>(idframe2[i].center)[1] > smin &&
					img.at<Vec3b>(idframe2[i].center)[2] > vmin)
			{
				// scan for rectangles which are big enough and have acceptable aspect ratios
				if(idframe2[i].bRect.area() > minarea && (aspectRatio < bhigh && aspectRatio > blow))
				{
					Blob ident = Blob(idframe2[i].contour,i /*index of the iteration*/,
							idframe2[i].center,idframe2[i].radius,idframe2[i].bRect);
					out.push_back(ident);
				}
			}
		}
	}
	return out;
}

/*
 * Track the blob using mean shift
 */
RotatedRect msTrack(Mat &hsv, Mat &bw, Mat &hist, Blob idsLast, int &counter)
{
	Mat hue,backproj;
	RotatedRect tb;
	float hranges[] = {0,180};
	const float * phranges = hranges;
	int hsize = 16;
	int ch[] = {0, 0};
	// create a mixed channel image of hue and hsv
	hue.create(hsv.size(), hsv.depth());
	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	// sanity check if bRect is out of bounds
	if(idsLast.bRect.tl().x < hsv.cols && idsLast.bRect.tl().x > 0 &&
		idsLast.bRect.tl().y < hsv.rows && idsLast.bRect.tl().y > 0
			&& idsLast.bRect.br().x < hsv.cols && idsLast.bRect.br().x >0
			&& idsLast.bRect.br().y < hsv.rows && idsLast.bRect.br().y > 0)
	{
		// build a ROI using the bounding rectangle of the blob from last frame
		if(counter < 3)
		{
			Mat roi(hue, idsLast.bRect), maskroi(bw, idsLast.bRect);
			calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
			normalize(hist, hist, 0, 255, CV_MINMAX);
		}
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= bw;
		tb = CamShift(backproj, idsLast.bRect,
				TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
	}
	return tb;
}

/*
 * The image callback function performs all the image processing of the frames when called by ros:spin
 */

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	timeval frame_start, frame_end;
	gettimeofday(&frame_start,NULL);
    /* Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing */

	//timeval cvb_s, cvb_e;

	//gettimeofday(&cvb_s,NULL);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // Always copy, returning a mutable CvImage
        // OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        // if there is an error during conversion, display it
        ROS_ERROR("cv_proc.cpp::cv_bridge exception: %s", e.what());
        return;
    }
	//gettimeofday(&cvb_e,NULL);

	//cout << "CV bridge conversion time " << cvb_e.tv_usec-cvb_s.tv_usec << endl;

	/* for diverse timing measurements */
	timeval start, end;

	//gettimeofday(&start,NULL);
	/* Image containers (actual image + histogram)*/
	Mat src, hist, dst;
	/* Camera matrix, Distortion coefficients -> no need to read every time 
		-> defined statically in header file */
	// Mat camMatrix, distCoeffs; 
	/* Output vectors for the solvePNP pose estimation */
	Mat tvec(3,1, DataType<double>::type);
	Mat rvec(3,1, DataType<double>::type);
	/* Image container for the undistorted image */
	Mat undistorted;
	/* Position estimate */
	Mat P(3,1, DataType<double>::type);

	src = cv_ptr->image;

	/* 2D image and 3D object points for the PNP algorithm */
	vector<Point2f> imagePoints;
	vector<Point3f> objectPoints;
	objectPoints.push_back(Point3f(1,0,0));
	objectPoints.push_back(Point3f(0,1,0));
	objectPoints.push_back(Point3f(-1,0,0));
	objectPoints.push_back(Point3f(0,-1,0));

	

	if(VISION_DEBUG){
	/* To adjust HSV values -> to set the color filter */
		createTrackbar( "Hmin", "Processed Image", &hmin, 256, 0 );
		createTrackbar( "Smin", "Processed Image", &smin, 256, 0 );
		createTrackbar( "Vmin", "Processed Image", &vmin, 256, 0 );
		createTrackbar( "Hmax", "Processed Image", &hmax, 256, 0 );
		createTrackbar( "Smax", "Processed Image", &smax, 256, 0 );
		createTrackbar( "Vmax", "Processed Image", &vmax, 256, 0 );
	//	createTrackbar( "Threshold", "Blob Detection", &dist, 1000, 0 );
	}
	// get the camera matrix and the distortion coefficients from the calibration file

	

	// getCalibrationData(filename, camMatrix, distCoeffs);
	// we create an instance of the pose estimator class with the camera intrinsics found in the calibration step
	PoseEstimation pose_est(objectPoints, distCoeffs, camMatrix);
	// we create an instance of the LK optical flow estimator class with generic parameters
	OpticalFlow flow_est;

	//gettimeofday(&end,NULL); 

	//cout << "Initialisation etc. time: " << end.tv_usec-start.tv_usec << endl;	

	/* Undistort image using the distortion coefficients
	 * from the calibration file
	 * TODO: adaptive undistortion depending on blob position in the image
	 */
	//undistort(src,undistorted,camMatrix,distCoeffs);
	//src = undistorted;

	/* Segmentation using erode function */
	//Mat mod;
	//Mat element(5,5,CV_8U,cv::Scalar(1));
	//erode(src,mod,element);

	/* convert to HSV color space */
	//timeval hsv_s, hsv_e;

	//gettimeofday(&hsv_s, NULL);	

	Mat hsv;
	cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);

	//gettimeofday(&hsv_e, NULL);

	//cout << "HSV conversion time " << hsv_e.tv_usec-hsv_s.tv_usec << endl;

	/* only display the image parts which are in the color range
	 * -> binarization step
	 */

	//timeval bin_s, bin_e;

	//gettimeofday(&bin_s, NULL);	
	Mat bw;
	inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), bw);
	//gettimeofday(&bin_e, NULL);	

	//cout << "Binarization time " << bin_e.tv_usec-bin_s.tv_usec << endl;

	/* Two blur approaches -> too CPU-heavy for onboard use -> try others/separate filters */
	//Mat filtered; // to avoid in-place filtering
	//gettimeofday(&start, NULL);
	//GaussianBlur(bw, filtered, Size(3,3), 1.5, 1.5);
	//bw = filtered;
	//gettimeofday(&end, NULL);

	//cout << "Gaussian blur filtering time " << end.tv_usec-start.tv_usec << endl;
	// NOTE: roughly 300-450 us on core i7, 2500-14000us on Gumstix

	/* find the contours */

	//timeval contours_s, contours_e;

	//gettimeofday(&contours_s,NULL);

	vector<vector<Point> > contours;
	findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//gettimeofday(&contours_e,NULL);

	//cout << "Contours time " << contours_e.tv_usec-contours_s.tv_usec << endl;

	if(VISION_DEBUG)
	{
		/* drawContours to fill the blanks */
		dst = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
		drawContours(dst, contours, -1, Scalar::all(255), CV_FILLED);
	}

	/*
	 * Get the initial IDs for the blobs, thus actually creating
	 * a Blob vector with detected blobs in every frame
	 */
	
	//timeval blob_s,blob_e;

	//gettimeofday(&blob_s,NULL);
	ids = BlobIdentifier(contours,cv_ptr->image);
	//gettimeofday(&blob_e,NULL);

	//cout << "Blob id time " << blob_e.tv_usec-blob_s.tv_usec << endl;

	int size1 = ids.size();
	int size2 = idsLast.size();

	int size_min = min(size1,size2);
	int size_max = max(size1,size2);

	/* After initial identification we use
	 * the best match with Kalman/Particle filter for
	 * estimation or the mean shift approach to
	 * track the identified blobs
	 */
	if(loopcounter > 1)
	{
		/* Update the blobs according to the best match principle */
		ids = bestMatch(idsLast,ids,dist,cv_ptr->image);
		/* The following commented out section includes the mean shift tracking code
		 * which is not used for our purposes, but served as testing ground
		 */
		//Mat hist,hue,backproj;
		//			for(int i = 0; i < size_min; i++)
		//			{
		//				RotatedRect tb = msTrack(hsv,bw,hist,idsLast[i],loopcounter);
		//				if(tb.boundingRect().contains(idsLast[i].center))
		//						ids[i].id = idsLast[i].id;
		//				// update the bounding rectangles
		//				ids[i].bRect = tb.boundingRect();
		//				rectangle(src,tb.boundingRect(),Scalar(255,0,255), 1, CV_AA);
		//				ellipse(src, tb, Scalar(255,0,255), 1, CV_AA );
		//			}
		//			for(int j = size_min; j<size_max; j++)
		//			{
		//				// TODO: handle the new blobs here
		//				//cout << "test" << endl;
		//			}
	}
	// print blob count every 30th iteration
	if((loopcounter % 30) == 0) 
	    cout << "Blob count: " << size_min << endl;
	// Display the blob rectangles and their IDs
	imagePoints.clear(); //clear old imagePoints
	for(int i = 0; i < size_min; i++ )
	{
		imagePoints.push_back(ids.at(i).center); //set imagePoints
		// paths.push_back(getPath(ids.at(i))); //XXX: opt
		// draw the bounding rectangles
		//if(VISION_DEBUG)
		//{	
			rectangle(cv_ptr->image, ids.at(i).bRect.tl(), ids.at(i).bRect.br(), Scalar(255,0,255), 2, 8, 0 );
			ostringstream str;
			str << ids.at(i).id;
			putText(cv_ptr->image, str.str(), ids.at(i).center,
				FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,0,0), 1, CV_AA);
		//}
	}

	// vector<Point2f> flow_features = getPointVector(ids); //XXX: opt

	/*only estimate position if the blob count is 4 <-> only if all the markers have been detected*/
	if(size_min == 4)
	{
		//gettimeofday(&start, NULL);

		//Point2f flow = flow_est.calcFlow(dst,last_frame,flow_features);
		// iterative method takes roughly 50ms, P3P ~10ms, EPNP ~15ms
		P = pose_est.estimate(imagePoints,rvec,tvec,CV_P3P,0);

		//cout << "Position estimate " << P << endl;
		//cout << "Translation vector " << tvec << endl;		
		//gettimeofday(&end,NULL);

		//cout << "Time to solve PNP: " << end.tv_usec - start.tv_usec << " microseconds" << endl;
	}

	if(VISION_DEBUG)
	{
		/* draw blob paths */
		drawContours(src, paths, -1, Scalar(200,0,200), 3);

		/* overlay of the segmented parts of the image ->
		 * only displays the "color bits"
		 */
		dst &= cv_ptr->image; //XXX: opt

		//cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
		//cv::imshow("test", dst);
		/* Display the image using OpenCV -> this is only needed to debug the vision part*/
		cv::imshow("Processed Image", cv_ptr->image);
		/* Add some delay in miliseconds.
	 	* The function only works if there is at least one
	 	* HighGUI window created and the window is active.
	 	* If there are several HighGUI windows, any of them can be active.
		*/
		cv::waitKey(3);
	}

	/* update the ids and the loopcounter */
	idsLast = ids;
	//last_frame = dst; // for flow estimation
	loopcounter++;

	
	/* generate PoseWithCovarianceStamped message here and fill it with our pose estimate */
	geometry_msgs::PoseWithCovarianceStamped p_msg;
	p_msg.header.stamp = ros::Time::now(); // the timestamp header
	//p_msg.pose.covariance = {1,0,0,0,0,0,  // the pose covariance -> identity matrix?
	//						 0,1,0,0,0,0,
	//						 0,0,1,0,0,0,
	//						 0,0,0,1,0,0,
	//						 0,0,0,0,1,0,
	//						 0,0,0,0,0,1};
	/* Position of the world origin (our landing pad), in camera coordinates
	 * XXX: review the correctness of this approach (some transformation is probably required
	 * to do continuous computations in the world frame -> use P instead maybe
	 */
	p_msg.pose.pose.position.x = tvec.at<double>(0);
	p_msg.pose.pose.position.y = tvec.at<double>(1);
	p_msg.pose.pose.position.z = tvec.at<double>(2);
	geometry_msgs::Quaternion quat =
			tf::createQuaternionMsgFromRollPitchYaw(rvec.at<double>(0),rvec.at<double>(1),rvec.at<double>(2));
	p_msg.pose.pose.orientation = quat;

	pose.publish(p_msg);
	pub.publish(cv_ptr->toImageMsg());
	//gettimeofday(&frame_end,NULL);

}

int main(int argc, char **argv)
{

    ROS_INFO("Initialising HB Vision Node");
    ros::init(argc, argv, "hb_vision");

    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

	if(VISION_DEBUG)
	{
		ROS_INFO("Vision debug mode on");
    	cv::namedWindow("Processed Image", CV_WINDOW_AUTOSIZE);
	}
    // measure processing time
    
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    
    cv::destroyWindow("Processed Image");
    
    /*
     * We advertise two topics: the processed image and the pose estimation done above
     * The pose is published to the fcu/pose topic, so the SSDK (and later the DEKF)
     * observers can fuse the pose estimates with the IMU and provide a better
     * interpolated position estimate
     */
    pub = it.advertise("hb_vision/image_processed", 1);
    pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("fcu/pose", 1);

	printTopicInfo();
    /**
    * In this application all user callbacks will be called from within the ros::spin() call. 
    * ros::spin() will not return until the node has been shutdown, either through a call 
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    ROS_INFO("HB_Vision::No error.");
}
