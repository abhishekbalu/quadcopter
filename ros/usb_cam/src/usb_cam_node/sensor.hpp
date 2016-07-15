#ifndef __SENSOR__
#define __SENSOR__

#include <Eigen/Dense>
#include <vector>

//THIS ALGORITHM IS READY TO OPERATE ON WIDTH LINES, FOR ANY RGB, RGBA, etc. FOR BGR YOU NEED YOURSELF TO INVERT THE ORDER

//blob structure
typedef struct t_blob
{
    double x;
    double y;
    double sz;
    int valid;
} blob;

//tracked marker structure
typedef struct t_marker
{
    const char* name;
    Eigen::MatrixXd T;
    int imx0,imy0,imxf,imyf;
    int imx0_LED,imy0_LED,imxf_LED,imyf_LED;
    int state;
    int frequencies[10];
    int count;
    double filter_count,max;
    int current_freq;
    int failures;
    double t_old;
} marker;

//functions to be called do operate the sensor

int detect_blobs(unsigned char* buf,unsigned int step,int vl,int vh,int hl,int hh,int sl,int sh,int xi,int xf,int yi,int yf);

int detect_markers(); //returns number of markers observed

int track_markers(unsigned char* buf,unsigned int step); //return the number of tracked objects

unsigned char* get_binary_image(); //return the blob detection binary image for debug

blob* get_blobs(); //return the blob list (mostly for debug)

std::vector<marker>* get_markers();

//returns vector to be used to fetch the markers
Eigen::MatrixXd* sensor_init(int width,int height,
                             double fx_l,double fy_l,double cx_l,double cy_l,double kx1_l,double kx2_l,double ky1_l,double ky2_l,double theta_l,
			     double dt_min_l,
			     int lochm_l,int lochM_l,int locsm_l,int locsM_l,int locvm_l,int locvM_l,
			     int save_blobs_l,int noise_active_l);

void close_blob_files();			     

#endif
