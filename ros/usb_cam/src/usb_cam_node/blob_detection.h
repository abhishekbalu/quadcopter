#ifndef BLOB_DETECTION_H
#define BLOB_DETECTION_H

#include <ros/ros.h>

#include "blob.h"
#include <usb_cam/usb_cam.h>
#include <usb_cam/detections.h>

typedef struct BlobParams{
	int min_hue, max_hue, min_sat, max_sat, min_val, max_val;
	int min_size;

} BlobParams;

void detect_blobs(usb_cam_camera_image_t* img, usb_cam::detections &blobs, 
	BlobParams &blob_params, ros::Publisher det_pub, bool publish_debug);


#endif




