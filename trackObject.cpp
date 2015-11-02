/*
 * trackObject.cpp
 *
 * Stephen Markham
 * University of Otago 2015
 *
 */

#include <pthread.h>
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include <chrono>

// #include "gimbal_control.h"
using namespace std;
using namespace cv;

#ifdef __APPLE__
	VideoCapture capture;
#elif __linux
	#include <raspicam/raspicam_cv.h> // For Pi Camera
	VideoCapture capture;
	raspicam::RaspiCam_Cv Camera;
#endif

using namespace std::chrono;

//---------------------------------
// Various Options For Testing
//----------------------------------

// Use orb rather than Good Features to Track
//#define usingOrb

//Test using video files
#define TEST

//Slow the frame rate (testing)
#define SLOW

//Verbose Output
#define VERBOSE

//Split GFFT into 4 corners
#define CORNERS

//Use Kalman Filter
//#define USING_KALMAN

//#define SHOWOBJECT

//------------------------------
//         CONSTANTS
//------------------------------

//  number of seconds between frames whilst going slow
double SECONDS = 1;

//  SVD Ratio Allowed for Good Homography
int HOMOGRAPHY_RATIO = 10000; //  1000

//  Size of the Output Frame
Size VIEWSIZE = Size(1000, 500);

#ifndef usingOrb
	// Feature Tracking Constants
	double MIN_DISTANCE = 25;  //Bigger value forces more distance between points
	int EIG_BLOCK_SIZE = 3;  // 3 // 100
	int USE_HARRIS = false;	  // Use Harris Corners

	//This is the starting value as it will float to get the best quality points
	double QUALITY_LEVEL = 0.1;

	//This number is for whole image and will be split over 4 corners of image
	int MAX_FEATURES = 40; // Should be divisible by 4
#endif

//-------------------------------
//     GLOBAL VARIABLES
//-------------------------------
// Overall Homography to Axis Matrix
Mat overallHomography = Mat::eye(Size(3,3), CV_32F);

// Frame Count
int frameCount = 0;

// Rotation Matrix
Mat rotationMatrix (Size(3,3), CV_32F);

// Camera Feed
Mat cameraFeed;

// Used when checking
bool garbage = false;

// Output image
Mat outputImage;

// Kalman Filter Variables
int allowDiff = 50;
KalmanFilter KF(4, 2, 0);
Mat_<float> measurement(2,1);


#ifdef TEST
//-----------------------------------
//   TEST FILES
//------------------------------------
	//String TESTFILE = "Test Files/15x0_1.mp4"; // Always Sees Axis
	//String TESTFILE = "Test Files/30x30_1.mp4";
	//String TESTFILE = "Test Files/60x30_1.mp4"; // Always Sees Axis
	//String TESTFILE = "Test Files/60x30_2.mp4"; // Issues with brightness
	//String TESTFILE = "Test Files/100x100_1.mp4";
	//String TESTFILE = "Test Files/outside.mp4";
	//String TESTFILE = "Test Files/object1.mp4";
	//String TESTFILE = "Test Files/object2.mp4";
 	//String TESTFILE = "Test Files/car.mp4";
 	//String TESTFILE = "Test Files/car2.mp4";
	String TESTFILE = "Test Files/inside1.mp4";
 	//String TESTFILE = "Test Files/inside1at320.mp4";
	//String TESTFILE = "Test Files/rotate1.mp4";
 	//String TESTFILE = "Test Files/rotate1at320.mp4";
	//String TESTFILE = "Test Files/test_4.mp4";
	//String TESTFILE = "Test Files/test_5.mp4";
#else
 	//  Video Frame Size
	int FRAMESIZEX = 640;
	int FRAMESIZEY = 480;
#endif


/*
 * bool getImage()
 *
 * Gets image from camera or video
 * Returns true if successful
 */
bool getImage()
{
	Mat imageFrame;
	#ifdef TEST // get image from video file
		if(capture.read(imageFrame) ==false){
			if (frameCount >= 2) cout << "Reached end of Video" << endl;
			else cout << "Error Finding Video Frame" << endl;
			return false;
		}
	#else
		#ifdef __linux // get image from pi camera
			Camera.grab();
			Camera.retrieve (imageFrame);
		#else // using standard web camera
			if (capture.read(imageFrame) == false){
				if (frameCount >= 2) cout << "Reached end of Video" << endl;
				else cout << "Error Finding Video Frame" << endl;
				return false;
			}
		#endif
	#endif

	// convert to grey scale
	cv::cvtColor(imageFrame, imageFrame, CV_BGR2GRAY);

	cameraFeed = imageFrame.clone();
	return true;
}


#ifndef usingOrb

/*
 * std::vector<cv::Point2f> getFeatures
 *
 * Gets features from 4 corners of video
 * returns std::vector<cv::Point2f> corners
 */
 std::vector<cv::Point2f> getFeatures(Mat im){

 	// Vectors to hold the various sets of points
 	std::vector<cv::Point2f> feats, topLeft, topRight, bottomLeft, bottomRight;

	//Top Right
	Mat mask = Mat::zeros(im.size(), CV_8U);  //  type of mask is CV_8U
	for (int i = 0; i < mask.rows/2; i++){
		for (int j = mask.cols/2; j < mask.cols; j++){
			mask.at<unsigned char>(i,j) = 1;
		}
	}
	cv::goodFeaturesToTrack(im,topRight,(MAX_FEATURES/4),QUALITY_LEVEL,MIN_DISTANCE,mask,EIG_BLOCK_SIZE,USE_HARRIS);


	//Top Left
	mask = Mat::zeros(im.size(), CV_8U);  //  type of mask is CV_8U
	for (int i = 0; i < mask.rows/2; i++){
		for (int j = 0; j < mask.cols/2; j++){
			mask.at<unsigned char>(i,j) = 1;
		}
	}
	cv::goodFeaturesToTrack(im,topLeft,(MAX_FEATURES/4),QUALITY_LEVEL,MIN_DISTANCE,mask,EIG_BLOCK_SIZE,USE_HARRIS);


	//Bottom Left
	mask = Mat::zeros(im.size(), CV_8U);  //  type of mask is CV_8U
	for (int i = mask.rows/2; i < mask.rows; i++){
		for (int j = 0; j < mask.cols/2; j++){
			mask.at<unsigned char>(i,j) = 1;
		}
	}
	cv::goodFeaturesToTrack(im,bottomLeft,(MAX_FEATURES/4),QUALITY_LEVEL,MIN_DISTANCE,mask,EIG_BLOCK_SIZE,USE_HARRIS);


	//Bottom Right
	mask = Mat::zeros(im.size(), CV_8U);  //  type of mask is CV_8U
	for (int i = mask.rows/2; i < mask.rows; i++){
		for (int j = mask.cols/2; j < mask.cols; j++){
			mask.at<unsigned char>(i,j) = 1;
		}
	}
	cv::goodFeaturesToTrack(im,bottomRight,(MAX_FEATURES/4),QUALITY_LEVEL,MIN_DISTANCE,mask,EIG_BLOCK_SIZE,USE_HARRIS);


	//Merge all of the points into a single Vector
	feats.insert(feats.end(), topLeft.begin(), topLeft.end());
	feats.insert(feats.end(), topRight.begin(), topRight.end());
	feats.insert(feats.end(), bottomLeft.begin(), bottomLeft.end());
	feats.insert(feats.end(), bottomRight.begin(), bottomRight.end());

	//Uncomment bellow to see the points displayed on an image
/*
	for (int index = 0; index < feats.size(); index++){
			circle(im,feats[index],2,Scalar(0,255,0),2);
	}
	imshow("Image", im);
*/

	// "Floating" quality level to ensure best quality points, yet enough of them
	if ( 	(topLeft.size() < MAX_FEATURES/8 		||
			topRight.size() < MAX_FEATURES/8 			||
			bottomLeft.size() < MAX_FEATURES/8 		||
			bottomRight.size() < MAX_FEATURES/8) 	&&
			QUALITY_LEVEL > 0.04) QUALITY_LEVEL -= 0.02;

	else QUALITY_LEVEL += 0.02;

	return feats;
 }

#endif

/**
 * findObject()
 * Finds object in an image
 *
 * @param Mat image (overall scene)
 * @param Mat mask(mask of the image)
 * @param Mat kvMask (mask using Kalman Filter to narrow search area)
 * @return Point location of the object
 */
Point findObject(Mat image, Mat mask, Mat kvMask)
{
	Mat thresh;

	Mat m2 = mask;

	bitwise_not(m2, m2);

	// Binary Threshold to find 'bright' spots (Axis)
	threshold(image, thresh,50,255,THRESH_BINARY);

	// Erode and dilate elements.
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(70,70));

	// dialate image
	dilate(m2,m2,dilateElement);

	//get rid of parts that don't belong
	bitwise_or(m2, thresh, thresh);
	bitwise_xor(m2, thresh,thresh);

	if (!kvMask.empty()) bitwise_and(thresh, kvMask, thresh);

	// Erode to remove noise/smaller bright objects
	erode(thresh,thresh,erodeElement);

	// Dilate to get back the axis markers to a normal size
	dilate(thresh,thresh,dilateElement);

	// Vectors to store the contours and hierarchies for marker detection
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Finding the markers
	findContours(thresh,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

	// areas for the markers
	double largestArea = 0.0;
	int xLoc = 0;
	int yLoc = 0;

	// Finding Objects Based Upon Brightness Makers
	if (hierarchy.size() > 0){
		for (int i = 0; i >= 0; i = hierarchy[i][0]){
			Moments moment = moments((cv::Mat)contours[i]);
			double area = moment.m00;
			if (area > largestArea){
				largestArea = area;

				xLoc = moment.m10/area;
				yLoc = moment.m01/area;

			}
		}
	// If 4 objects found, 3x Axis markers + 1x Object
	// Else axis not found
	}else{
		#ifdef VERBOSE
			cout << "Object NOT Found \t| ";
		#endif
		//return false;
	}

	Point p;

	p.x = xLoc;
	p.y = yLoc;
	// Returns true to inform axis found
	return p;
}

#ifdef USING_KALMAN

	/*
	 * kalmanInitilise()
 	 * Initialises the Kalman Filter values
	 *
 	 * @param Point p (Starting Point at which the object is located)
 	 */
	void kalmanInitlise(Point p)
	{
		measurement.setTo(Scalar(0));
		KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
		KF.statePre.at<float>(0) = p.x;
		KF.statePre.at<float>(1) = p.y;
		KF.statePre.at<float>(2) = 0;
		KF.statePre.at<float>(3) = 0;
		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(100));
		setIdentity(KF.measurementNoiseCov, Scalar::all(10));
		setIdentity(KF.errorCovPost, Scalar::all(1));

	}

	/*
	 * kalmanUpdate()
 	 * Updates the Kalman Filter values
	 *
 	 * @param Point p object location
	 * @return state Point
 	 */
	Point kalmanUpdate(Point p)
	{
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

		measurement(0) = p.x;
		measurement(1) = p.y;

		Mat estimated = KF.correct(measurement);

		Point statePt(estimated.at<float>(0),estimated.at<float>(1));

		return statePt;
	}

	/*
	 * kalmanPredict()
 	 * Predicted Kalman Values
	 *
 	 * @param Point p (Starting Point at which the object is located)
 	 */
	Point kalmanPredict()
	{
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

		return predictPt;
	}
#endif

/*
 * int ImageTracking
 *
 * Uses cameras to locate objects
 */
int ImageTracking()
{
	#ifdef usingOrb
		cout << "Using ORB" << endl;
	#else
		cout << "Using goodFeaturesToTrack" << endl;
	#endif

	#ifdef USING_KALMAN
		cout << "Using Kalman Filter" << endl;
	#endif

	// Testing Stuff

	#ifdef TEST
		#ifdef VERBOSE
			cout << "Running test on " << TESTFILE << endl;
		#endif
	#endif

	// Values for testing purposes
	int c = 0, TotalHomographiesCalculated = 0, NumberBadHomographies = 0;
	bool match = false;

	// Various Matrices for stuff below
	Mat prevCameraFeed, H2, temp, descriptors, prevDescriptors, prevTrans;
	// Storage of Points
	std::vector<cv::Point2f> corners, prevCorners;
	std::vector<Point> locations, kalmanv;
	std::vector<cv::KeyPoint> keypoints, prevkeypoints;
	Point p, statePt;
	// Errors and Status for below
	std::vector<uchar> status;
	std::vector<float> err;

	// Initializes Values
	frameCount = 1;

	//get image
	if (!getImage()) return -1;

	#ifdef usingOrb // use orb to get points
		ORB o = ORB(1000,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31);
		o.operator()(cameraFeed,cv::noArray(),keypoints, descriptors, false);
	#else //use GFFT to get points
		#ifdef CORNERS //using four individual corners
			corners = getFeatures(cameraFeed);
		#else	//Not using four corners (not recomeded)
			cv::Mat mask;
			cv::goodFeaturesToTrack(cameraFeed,corners,(MAX_FEATURES),QUALITY_LEVEL,MIN_DISTANCE,mask,EIG_BLOCK_SIZE,USE_HARRIS);
			/* DEBUG SHOW POINTS
			for (int index = 0; index < corners.size(); index++){
				circle(cameraFeed,corners[index],2,Scalar(0,255,0),2);
			}
			imshow("Points", cameraFeed);
			*/
		#endif
	#endif

	//Repeat forever
	while(1){
		prevDescriptors = descriptors;
		prevCorners = corners;
		prevCameraFeed = cameraFeed.clone();
		prevkeypoints = keypoints;
		Mat trans;

		// Prints frame number for debugging
		#ifdef VERBOSE
			cout << "Frame Number: " << frameCount << " \t| ";
		#endif

		// If on laptop then start clock for timing/debugging
		#ifdef __APPLE__
			high_resolution_clock::time_point t1 = high_resolution_clock::now();
		#endif

		// Increment Frame count (Mainly for debugging)
		frameCount ++;

		#ifdef usingOrb
			//Nothing Needs to go in here
		#else
			// if the results of the last homography were garbage
			// c > 5 is a cut your losses approach (POTENTIALLY DANGEROUS)
			if (!garbage || c > 5){
				// Store previous image
				prevCameraFeed = cameraFeed.clone();
				// Store previous corners so can get new ones to compare to
				prevCorners = corners;
			}

			//Ensure all points are still here
			std::vector<cv::Point2f> temp;
			for (int index = 0; index < prevCorners.size(); index++){
				if (prevCorners[index].x > 0 && prevCorners[index].x < 640 &&
					prevCorners[index].y > 0 && prevCorners[index].y < 480)
					temp.insert(temp.end(),prevCorners[index]);
			}
			prevCorners = temp;

			// Check if we need to find more points
			if (prevCorners.size() < MAX_FEATURES/5 || prevCorners.size() < 10) {
				#ifdef CORNERS //if using four corners
					prevCorners = getFeatures(cameraFeed);
				#else // else
				cv::Mat mask2;
					cv::goodFeaturesToTrack(cameraFeed,prevCorners,(MAX_FEATURES),QUALITY_LEVEL,MIN_DISTANCE,mask2,EIG_BLOCK_SIZE,USE_HARRIS);
					/* For Testing
					for (int index = 0; index < prevCorners.size(); index++){
						circle(cameraFeed,prevCorners[index],2,Scalar(0,255,0),2);
					}
					imshow("Points", cameraFeed);*/
				#endif
			}
		#endif

		// Get Image, if fails then assume video has ended
		if (!getImage()){
			#ifdef VERBOSE //Print debug info
				cout << "Bad Homographies " << ((float)NumberBadHomographies/(float)TotalHomographiesCalculated) *100 << "\% of the time" << endl;
			#endif
			return -1;
		}

		// If Using orb for point location
		#ifdef usingOrb
			o.operator()(cameraFeed,cv::noArray(), keypoints, descriptors, false);

			std::vector<Point2f> prevCorners;
	  		std::vector<Point2f> corners;

			// is the descriptors arn't empty
			if (!descriptors.empty() & !prevDescriptors.empty()){

				//Matcher
				BFMatcher matcher(NORM_L2);

				// Match (knn)
				std::vector <std::vector <cv::DMatch > > matches;
			  	matcher.knnMatch( descriptors, prevDescriptors, matches, 2);

					//Find the good matches
				std::vector< DMatch > good_matches;
				for( int i = 0; i < descriptors.rows; i++ ){
					if( matches[i][0].distance < 0.7 * matches[i][1].distance){
						good_matches.push_back( matches[i][0]);
					}
				}

				//Draw good matches
				Mat img_matches;
		  		cv::drawMatches( cameraFeed, keypoints, prevCameraFeed, prevkeypoints,
		        				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		        				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		  		imshow("Matches", img_matches);

		  		for( int i = 0; i < good_matches.size(); i++ ){
		    		prevCorners.push_back( prevkeypoints[ good_matches[i].trainIdx ].pt );
		    		corners.push_back( keypoints[ good_matches[i].queryIdx ].pt );
		  		}
	  		}
	  	#else

  			cv::calcOpticalFlowPyrLK(	prevCameraFeed,
										cameraFeed,
										prevCorners,
										corners,
										status,
										err);
  			//#ifndef CORNERS
				for (int index = 0; index < prevCorners.size(); index++){
					circle(cameraFeed,prevCorners[index],2,Scalar(0,255,0),2);
				}

				imshow("Points", cameraFeed);
			//#endif

	  	#endif

  		// Checks enough points to calculate Homography
		if (corners.size() < 4 || prevCorners.size() < 4){
			#ifdef VERBOSE
				// Prints debug data
				cout << "Not enough points to track \t| ";
			#endif

			// If enough data to calculate homography
		}else{

			// Perform Homography Calculation
			Mat H = findHomography(corners, prevCorners, CV_RANSAC);
			// Convert the homography matrix so it can be used
			H.convertTo(H2, CV_32FC2);

			TotalHomographiesCalculated++;
			// Matrix to hold Single Value Decomposition Results
			Mat w, u, vt;
			// Computing the SVD
			SVD::compute(H2, w, u, vt);
			// Ratio between First and Last Singular Values (http:// stackoverflow.com/questions/10972438/detecting-garbage-homographies-from-findhomography-in-opencv/10981249#10981249)
			float ratio = w.at<float>(0) / w.at<float>(2);
			// if ratio is less than pre-defined threshold
			if (ratio < HOMOGRAPHY_RATIO){
				// if there is no existing change then the transformation is this matrix
				if (overallHomography.empty()) overallHomography = H2;
				// else we need to compute the change back to the axis
				else overallHomography = H2 * overallHomography;

				c = 0;// For debug output
				garbage = false;

			// If ratio is too high, then homography is garbage
			}else{
				// All of below is just for debugging
				c++;
				NumberBadHomographies++;
				garbage = true;
			}
		}

		Mat P = (Mat_<double>(3,4) << 0, 640, 640, 0, 0, 0, 480, 480, 1,1,1,1);

		Mat P2;

		P.convertTo(P2, CV_32FC2);

		Mat Q = overallHomography * P2;
		//cout << "\n" << Q << endl;

		//cout << (Q.at<float>(1,2)) << endl;
		float x1 = (Q.at<float>(0,0)) / (Q.at<float>(2,0));
		float x2 = (Q.at<float>(0,1)) / (Q.at<float>(2,1));
		float x3 = (Q.at<float>(0,2)) / (Q.at<float>(2,2));
		float x4 = (Q.at<float>(0,3)) / (Q.at<float>(2,3));

		float y1 = (Q.at<float>(1,0)) / (Q.at<float>(2,0));
		float y2 = (Q.at<float>(1,1)) / (Q.at<float>(2,1));
		float y3 = (Q.at<float>(1,2)) / (Q.at<float>(2,2));
		float y4 = (Q.at<float>(1,3)) / (Q.at<float>(2,3));

		float xmin;

		if (x1 < x2 && x1 < x3 && x1 < x4) xmin = x1;
		else if (x2 < x1 && x2 < x3 && x2 < x4) xmin = x2;
		else if (x3 < x1 && x3 < x2 && x3 < x4) xmin = x3;
		else xmin = x4;

		float ymin;

		if (y1 < y2 && y1 < y3 && y1 < y4) ymin = y1;
		else if (y2 < y1 && y2 < y3 && y2 < y4) ymin = y2;
		else if (y3 < y1 && y3 < y2 && y3 < y4) ymin = y3;
		else ymin = y4;

		Mat tempHomog = overallHomography.clone();

		Mat overallMove = Mat::eye(Size(3,3), CV_32F);

		tempHomog.at<float>(0,2) = (overallHomography.at<float>(0,2) - xmin);
		tempHomog.at<float>(1,2) = (overallHomography.at<float>(1,2) - ymin);

		overallMove.at<float>(0,2) = (overallMove.at<float>(0,2) - xmin);
		overallMove.at<float>(1,2) = (overallMove.at<float>(1,2) - ymin);

		//cout << "\nxmin: "<< xmin << endl;
		//cout << "ymin: "<< ymin << endl;

		//cout << "\n" << overallHomography << endl;

		//cout << "\n" << tempHomog << endl;

		// Apply the transformation
		warpPerspective(cameraFeed, trans, overallHomography, VIEWSIZE);

		//imshow("Transformed", trans);

		Mat mask, maskKv;
		cv::threshold(trans, mask, 0, 255, cv::THRESH_BINARY);

		//if (!outputImage.empty()) warpPerspective(outputImage, outputImage, overallMove, VIEWSIZE);

		trans.copyTo(outputImage, mask);

		// if we have had more than 2 'good' frames
		if (frameCount > 2 && !garbage){
			Mat j;

			// if using kalman filetr
			#ifdef USING_KALMAN
				if (match){
					Point w = kalmanv[(kalmanv.size())-1];
					if (w.x - allowDiff > 0 && w.y - allowDiff > 0){
						if (w.x + allowDiff < mask.size().width && w.y + allowDiff < mask.size().height){
							maskKv = Mat::zeros(mask.size(), CV_8U);
							Mat roi(maskKv, cv::Rect(w.x - allowDiff,w.y - allowDiff,(allowDiff*2),(allowDiff*2)));
							roi = Scalar(255, 255, 255);
						}
					}
				}
			#endif

			absdiff(trans, prevTrans, j);

			// find the object
			p = findObject(j, mask, maskKv);

				if(p.x != 0 && p.y != 0){
					if(!match){
						#ifdef USING_KALMAN
							kalmanInitlise(p);
						#endif
						match = true;
					}

					//update kalman values
					locations.push_back(p);
					#ifdef USING_KALMAN
						kalmanv.push_back(kalmanUpdate(p));
					#endif
				}else{
					if(match){
						#ifdef USING_KALMAN
							Point pre = kalmanPredict();
							//locations.push_back(pre);
							kalmanv.push_back(pre);
						#endif
						locations.push_back(locations[locations.size()-1]);
						//allowDiff+=25;
					}
				}
		}

		prevTrans = trans;
		// Show Optical Flow Points
		/*
		for (int index = 0; index < corners.size(); index++){
			circle(cameraFeed,corners[index],2,Scalar(0,255,0),2);
			line(cameraFeed,prevCorners[index],corners[index],Scalar(0,255,0), 2, CV_AA, 0);
		}
		*/

		//if showing the object
		#ifdef SHOWOBJECT

			#ifdef USING_KALMAN //kalman points
				for(int i = 0; i < (kalmanv.size()); i++){
					circle(outputImage,kalmanv[i],2,Scalar(0,255,0),15);
					//line(outputImage,kalmanv[i],kalmanv[i+1],Scalar(0,255,0), 2, CV_AA, 0);
				}
			#else
				if (locations.size() > 1){ // non kalman points
					for(int i = 0; i < (locations.size()); i++){
						//line(outputImage,locations[i],locations[i+1],Scalar(0,255,0), 2, CV_AA, 0);
						circle(outputImage,locations[i],2,Scalar(0,255,0),15);
					}
				}
			#endif

		#endif


		//warpPerspective(outputImage, outputImage, overallMove, VIEWSIZE);

		//imshow("Feed",cameraFeed);
		//imshow("Out",outputImage);


		// If slowing down for testing, allow for greater time between frames
		#ifdef SLOW
			waitKey(SECONDS * 1000);
		#else
			waitKey(30);
		#endif

		// If on laptop can print calculation times
		#ifdef __APPLE__
			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			long duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
			float d = (float)duration/1000000;
			std::cout << std::setprecision(2);
			#ifdef VERBOSE
				cout << d << " seconds \t| ";
			#endif
		#endif

		#ifndef usingOrb
			if (((float)NumberBadHomographies/(float)TotalHomographiesCalculated) < 0.05) MAX_FEATURES += 4;
			else if ( ( ( ( (float)NumberBadHomographies/(float)TotalHomographiesCalculated ) > 0.3 ) || c > 4 ) & (MAX_FEATURES > 20)) MAX_FEATURES -= 4;
		#endif

		#ifdef VERBOSE
			// Prints out debug data is requested
			if (garbage) cout << "Bad Homography #" << c <<  " \t| ";
			cout << "" << endl;
		#endif
	}
}

/*
void *GimbalController(void *threadarg)
{
	gimbal.control();
	pthread_exit(NULL);
}
*/

/*
 * Main Method
 *
 */
int main ()
{
	#ifdef TEST
		capture = VideoCapture(TESTFILE);
	#else
		#ifdef __linux
	    	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
			Camera.set(CV_CAP_PROP_FRAME_WIDTH,FRAMESIZEX);
			Camera.set(CV_CAP_PROP_FRAME_HEIGHT,FRAMESIZEY);
			if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;}
		#else
			capture.open(0);
 			capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAMESIZEX);
 			capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAMESIZEY);
 		#endif
 	#endif

 	// pthread_t t;
	// pthread_create(&t, NULL, GimbalController, NULL);

	high_resolution_clock::time_point t3 = high_resolution_clock::now();

	ImageTracking();

	high_resolution_clock::time_point t4 = high_resolution_clock::now();
	long duration = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
	float d = (float)duration/1000000;
	std::cout << std::setprecision(2);
	cout << d << " seconds \n" << endl;

	cout << frameCount/d << " frames per second" << endl;

	return 0;
}
