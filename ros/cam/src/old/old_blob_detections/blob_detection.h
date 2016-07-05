#ifndef _COMMON_DETECTION_
#define _COMMON_DETECTION_

#include <ros/ros.h>
#include <cam/blob.h>
#include <cam/usb_cam.h> //This has the usb_cam_camera_image_t
#include <cam/detections.h> //This defines the detections message (3 arrays, for x, y and size)

typedef struct BlobParams{
	int min_hue;
	int max_hue;
	int min_sat;
	int max_sat;
	int min_val;
	int max_val;
	int min_size;
} BlobParams;

void detect_blobs(sensor_msgs::Image img, cam::detections &blobs, BlobParams &blob_params, ros::Publisher debug_pub, bool publish_debug);

void initialize_img_vector(sensor_msgs::Image img, int STEP);

void rgb_hsv(uint8_t r, uint8_t g, uint8_t b, int &h, int &s, int &M, int &c, BlobParams &blob_params, uint8_t _img[], int j, int i);

void initialize_blob_vector();

void initialize_blob_params(BlobParams &blob_params, int min_hue, int max_hue,
	int min_sat, int max_sat, int min_val, int max_val, int min_size
	);
#endif