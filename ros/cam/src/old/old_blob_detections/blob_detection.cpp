//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <list>
//ROS Libraries
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

//Personal libraries
#include "cam/blob_detection.h"
#include "cam/usb_cam.h" //This has the usb_cam_camera_image_t
#include "cam/detections.h"
#include "cam/debug.h"

uint8_t* _img = NULL;
int blob_count = 0;
std::vector<Blob*> blob_vector;

void initialize_img_vector(sensor_msgs::Image img, uint8_t _img[], int STEP){
	for(int i=0; i < img.width; i++){ // 2,147,483,647 is max int
		for(int j = 0; i < img.height; j++){
			_img[3 * i + STEP * j + 0] = 0; //r
			_img[3 * i + STEP * j + 1] = 0; //g
			_img[3 * i + STEP * j + 2] = 0; //b
		}
	}
}

void rgb_hsv(uint8_t r, uint8_t g, uint8_t b, int &h, int &s, int &M, int &c, 
	BlobParams &blob_params, int j, int i, int STEP){
	//M is the maximum value in colour in the image
	//c is the colour range
	if(r > b && r > g){
		M = r; //set M to red
		if(g > b)
			c = M - b;
		else
			c = M - g;
		if(c != 0){
			h = (43 * (g - b)) / c;
			if(h<0)
				h += 256;
		}else{ //There is no hue, all colours are the same
			h = 0;
		}
	}else if(g > b){//r<b || r<g AND g>b
		M = g; //set M to green
		if(r > b)
			c = M - b;
		else
			c = M - r;
		if(c != 0)
			h = (43 * (g - b)) / c + (256/3);
		else //There is no hue, all colours are the same
			h = 0;
	}else{//r<b || r<g AND g<b
		M = b; //set M to blue
		if(r > g)
			c = M - g;
		else
			c = M - r;
		if(c != 0)
			h = (43 * (r - g)) / c + (512)/3; //2*256=512
		else
			h = 0;
	}
	//printf("%d \n", h);
	//Saturation is (max - min) / max, which is c/M
	if(M!=0)
		s = (256*c) / M;
	else
		s = 0;
	//Apply thresholds
	if((h<blob_params.max_hue && h>blob_params.min_hue) && 
		(s<blob_params.max_sat && s>blob_params.min_sat) &&
		(M<blob_params.max_val && M>blob_params.min_val)){
		_img[3 * i + STEP * j] = 255;
	}else{
		_img[3 * i + STEP * j] = 0;
	}
	//printf("HSV %d %d %d \n", h, s, M);
	_img[3 * i + STEP * j + 1] = -1;
	//_img[3 * i + 2 + 3 * img->width* j] = 0;

}
void initialize_blob_params(BlobParams &blob_params, int min_hue, int max_hue,
	int min_sat, int max_sat, int min_val, int max_val, int min_size
	){
	blob_params.min_hue = min_hue;
	blob_params.max_hue = max_hue;

	blob_params.min_sat = min_sat;
	blob_params.max_sat = max_sat;
	
	blob_params.min_val = min_val;
	blob_params.max_val = max_val;

	blob_params.min_size = min_size;


}
void initialize_blob_vector(){
	for(std::vector<Blob*>::iterator it=blob_vector.begin(); it != blob_vector.end(); ++it){
		delete (*it); // avoid leaks
	}
	blob_vector.clear();
	blob_count = 0;
}

void detect_blobs(sensor_msgs::Image img, cam::detections &blobs, 
	BlobParams &blob_params, ros::Publisher debug_pub, bool publish_debug){
	
	int WIDTH = img.width;
	int HEIGHT = img.height;
	int STEP = img.step;
	uint8_t r, g, b;
	int h = 0, s;
	int M, c; 


	int bottom_boundary = HEIGHT/4;
	int upper_boundary = 3 * HEIGHT/4;
	#ifdef VERBOSE
		ROS_INFO("Entered detect blobs...\n");
	#endif
	//initialize img vector
	if(_img == NULL){ 
		_img = new uint8_t[STEP * HEIGHT];
		//initialize_img_vector(img, _img, STEP);
	}
	#ifdef VERBOSE
		ROS_INFO("Starting RGB TO HSV...\n");
	#endif
	//RGB to HSV
	#ifdef VERBOSE
		ROS_INFO("WIDTH: %d, HEIGHT: %d\n", WIDTH, HEIGHT);
	#endif
	for(int i=0; i < WIDTH; i++){
		for(int j=0; j<HEIGHT; j++){
			//https://pt.wikipedia.org/wiki/HSV -- RED, GREEN, BLUE to HUE, SATURATION, VALUE
			
			r = img.data[3 * i + STEP*j + 0]; //get the r from the image
			g = img.data[3 * i + STEP*j + 1]; //get the g from the image
			b = img.data[3 * i + STEP*j + 2]; //get the b from the image
			rgb_hsv(r, g, b, h, s, M, c, blob_params, j, i, STEP);
		}
	}
	#ifdef VERBOSE
		ROS_INFO("Initializing blob vector...\n");
	#endif
	//Initialize blob vector
	initialize_blob_vector();
	//Analyze for blobs
	for(int j = 1; j < HEIGHT-1; j++){
		for(int i = 1; i < WIDTH-1; i++){
			if(_img[3*i + STEP*j] != 0){ //This pixel belongs to a blob

				if(_img[3*(i-1) + STEP * j] !=0 && i!=1 && j!=1){
				//---PIXEL BELONGS TO BLOB LEFT
					_img[3*i + STEP*j +1] = _img[3*(i-1) + STEP*j +1]; //Make them have same blob index
					
					blob_vector[_img[3*i + STEP*j + 1]]->add_pixel(i, j); //x, y
					
					if((_img[3*i + STEP*(j-1)] != 0) && 
						_img[3*(i-1) + STEP*j + 1] != _img[3*i+STEP*(j-1) + 1]){
						int id_blob_1 = _img[3*(i-1) + STEP + 1];
						int id_blob_2 = _img[3*i + STEP*(j-1) + 1];
						//Connect the blobs
						while(blob_vector[id_blob_1]->is_connected_to() != -1){
							id_blob_1 = blob_vector[id_blob_1]->is_connected_to();
						}
						while(blob_vector[id_blob_2]->is_connected_to() != -1){
							id_blob_2 = blob_vector[id_blob_2]->is_connected_to();
						}
						//If two blobs are not linked (maybe they are linked with two other blobs)
						//Connect the one with the largest id to the other one
						if(id_blob_1 != id_blob_2){
							if(id_blob_1 > id_blob_2){
								blob_vector[id_blob_2]->connect_to(id_blob_1);
							}else{
								blob_vector[id_blob_1]->connect_to(id_blob_2);
							}
						}
					}
/*used i!=1*/	}else if(_img[3*i + STEP *(j-1)] !=0  && j!=1){
				//---PIXEL BELONGS TO BLOB ABOVE
					_img[3*i + STEP*j] = _img[3*i + STEP*(j-1) + 1]; //make them have the same blob index
					blob_vector[_img[3*i + STEP*j + 1]]->add_pixel(i, j); //x, y

				}else{
				//---PIXEL HAS NO BLOBS CLOSE TO LEFT OR ABOVE. CREATE NEW BLOB 
					blob_vector.push_back(new Blob(i, j, blob_count));
					_img[3*i + STEP*j + 1] = blob_count;
					blob_count++;
				}
			}
		}
	}
	//Detections header is the same as the received image
	blobs.header = img.header;
	//Assemble blobs
	for(int i=blob_vector.size() - 1; i>-1; i--){
		if(blob_vector[i]->is_connected_to() == -1){
			#ifdef VERBOSE
				ROS_INFO("blob %d of size %d at %f, %f \n", i, blob_vector[i]->n_pix(), blob_vector[i]->x(), blob_vector[i]->y());
			#endif
			if(blob_vector[i]->n_pix() > blob_params.min_size){

				geometry_msgs::Point position;
				position.x = blob_vector[i]->x();
				position.y = blob_vector[i]->y();
				position.z = blob_vector[i]->n_pix();
				blobs.blob.push_back(position);
				blobs.blob_count = blob_count;
			}
		}else{
			blob_vector[blob_vector[i]->is_connected_to()]->assemble_Blob(blob_vector[i]);
		}
	}


	
	if(publish_debug){

		ROS_INFO("Publishing debug image\n");
		sensor_msgs::Image debug_img = img;
		//image, type, height, width, stepSize, data
		fillImage(debug_img, "rgb8", img.height, img.width, img.step, _img); 
		debug_pub.publish(debug_img);
	}
}

