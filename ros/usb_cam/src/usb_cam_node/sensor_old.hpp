#ifndef __SENSOR__
#define __SENSOR__

#include <Eigen/Dense>

typedef struct t_blob
{
    double x;
    double y;
    double sz;
    int valid;
} blob;

//functions to be called do operate the sensor

int detect_blobs(unsigned char* buf,unsigned int step);

int detect_markers(); //returns number of markers observed

unsigned char* get_binary_image();

blob* get_blobs();

//returns vector to be used to fetch the markers
Eigen::MatrixXd* sensor_init(int width,int height,
                             double fx_l,double fy_l,double cx_l,double cy_l,double kx1_l,double kx2_l,double ky1_l,double ky2_l);

#endif