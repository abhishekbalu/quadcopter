//C++ Libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <algorithm>    // std::max
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include "yaml-cpp/yaml.h"

//ROS Libraries
#include <ros/ros.h>
//User Libraries
#include "cam/blob_detection.h"
#include "cam/marker_detection.h"
#include "cam/p3p.hpp"
//Verbose
//#include "cam/debug.h" //Comment or uncomment this for verbose
//#define SHOW_MATRICES 1 //Comment or uncomment this for verbose matrices

using namespace Eigen;
using namespace std;
using namespace P3P_test;
using namespace YAML;

#define MAX_FEATURES 500
#define MAX_MARKERS 50
const int BLUE = 1;
const int RED = 0;
const int MIN_NUM_FAILURES = 5; //minimal number of failures allowed for a tracked frame
const double RED_BLOB_THRESHOLD_SCALE_FACTOR = 0.4;
const int MARKER_THRESHOLD = 4;

//................................TRACK MARKER VARIABLES.......................
int localization;
int nquad_freq=4;
double quad_freq[]={0,1,2,3};
string quad_freq_name[]={"frame0","frame1","frame2","frame3"};
const char* unknown = "unknown";

//................................DETECT MARKER VARIABLES.......................
//Really should read this crap from a file

int red_blob_sl = 60;
int red_blob_sh = 110;
int red_blob_hl = 350;
int red_blob_hh = 390;
int red_blob_vl = 50;
int red_blob_vh = 101;

int blue_blob_sl = 50;
int blue_blob_sh = 110;
int blue_blob_hl = 180;
int blue_blob_hh = 250;
int blue_blob_vl = 80;
int blue_blob_vh = 110;

std::string blue_blob_settings = "~/ros/cam/params/blue_blob_settings.yaml";
std::string red_blob_settings = "~/ros/cam/params/red_blob_settings.yaml";
std::string tracking_params = "~/ros/cam/params/tracking_params.yaml";

int objcnt=0;
int nmarkers=0;

double fx=470,fy=490; //focal distance guess?
double cx=320,cy=240; //center x, center y should be 320 and 240
double kx1=0.0000055,kx2=2e-10,ky1=0.0000020,ky2=0.0;
double theta_calib = 0.00;
double szx=0.4,szy=0.4, szLED=0.08;
double thsz=1;
double thd=400; //20*20
int nx = 640;
int ny = 480;
double objQuad[]={0.22,0.0,0.0, 0.0,0.22,0.0, 0.04,0.08,0.08, -0.0141,-0.007,0.25, 0.08,0.08,0.0};
double dt_min = 0.1;

double avg;
double radius,dx1,dx2,dx3,dx4,dy1,dy2,dy3,dy4;
double sz1, sz2, sz3, sz4;
double x1, _y1, x2, y2, x3, y3, x4, y4;
double a1, a2, a3, b1, b2, b3;
double d1, d2, d3, d4;
double vx[4], vy[4];

double freq = 0, total = 0;

blob blobs[MAX_FEATURES];
int status;
MatrixXd Tl(3,4);
MatrixXd Tsol(3,4);
MatrixXd* Tsols; //Matrix array, eigen doesn't allow 3d matrixes
double cost;
double lcost;
Matrix3d world_points;
Matrix3d Rl;
Matrix3d feature_vectors;
Vector4d markerh3D;
Vector3d trans;
Vector3d zcam;
Matrix3d camR;
Matrix3d new_frame;
Vector3d camt;

vector<marker> marker_list;

Vector3d markerh3DProjected, markerh2D, markerh2DPredicted, trans1;
int imx0[MAX_MARKERS],imy0[MAX_MARKERS],imxf[MAX_MARKERS],imyf[MAX_MARKERS];
int imx0_LED[MAX_MARKERS],imy0_LED[MAX_MARKERS],imxf_LED[MAX_MARKERS],imyf_LED[MAX_MARKERS];

//debug 
double aux1,aux2,aux3,aux4,aux1y,aux2y,aux3y,aux4y;
int iter_count=0;
void readTrackingParams(std::string file){
    YAML::Node root = YAML::LoadFile(file);
    try{
        fx = root["fx"].as<int>();
        fy = root["fy"].as<int>();
        cx = root["cx"].as<int>();
        cy = root["cy"].as<int>();
        kx1 = root["kx1"].as<double>();
        kx2 = root["kx2"].as<double>();
        ky1 = root["ky1"].as<double>();
        ky2 = root["ky2"].as<double>();
        theta_calib = root["theta_calib"].as<double>();
        szx = root["szx"].as<double>();
        szy = root["szy"].as<double>();
        szLED = root["szLED"].as<double>();
        thsz = root["thsz"].as<double>();
        thd = root["thd"].as<double>();
        nx = root["nx"].as<int>();
        ny = root["ny"].as<int>();
        dt_min = root["dt_min"].as<double>();
        for(unsigned short i=0; i<15; ++i){
            objQuad[i] = root["objQuad"][i].as<double>();
        }
    }catch(const BadConversion& e){}
}
void readBlueBlobParams(std::string file){
    YAML::Node root = YAML::LoadFile(file);
    try{
        blue_blob_vl = root["vl"].as<int>();
        blue_blob_vh = root["vh"].as<int>();
        blue_blob_hl = root["hl"].as<int>();
        blue_blob_hh = root["hh"].as<int>();
        blue_blob_sl = root["sl"].as<int>();
        blue_blob_sh = root["sh"].as<int>();
    }catch(const BadConversion& e){}
}
void readRedBlobParams(std::string file){
    YAML::Node root = YAML::LoadFile(file);
    try{
        red_blob_vl = root["vl"].as<int>();
        red_blob_vh = root["vh"].as<int>();
        red_blob_hl = root["hl"].as<int>();
        red_blob_hh = root["hh"].as<int>();
        red_blob_sl = root["sl"].as<int>();
        red_blob_sh = root["sh"].as<int>();
    }catch(const BadConversion& e){}
}
void initialize_markers(){
    //read this crap from a yaml file
    readBlueBlobParams(blue_blob_settings);
    readRedBlobParams(red_blob_settings);
    readTrackingParams(tracking_params);
    world_points.col(0)<<objQuad[0],objQuad[1],objQuad[2],
    world_points.col(1)<<objQuad[3],objQuad[4],objQuad[5],
    world_points.col(2)<<objQuad[6],objQuad[7],objQuad[8];
    zcam<<0,1,0;
    camR<<1,0,0,0,1,0,0,0,1;
    new_frame<<0,0,1,-1,0,0,0,-1,0;
    camR=new_frame*camR;
    camt<<0,0,0;
    Tsols=new Eigen::MatrixXd[MAX_MARKERS];
}

std::vector<marker>* get_markers(){
    return &marker_list;
}

int detect_markers(int no){ //This function extracts markers from blobs
	//GET THE BLOBS
	//blobs = get_blobs(); //from blob_detection
    std::copy(get_blobs(), get_blobs() + no, std::begin(blobs));
    #ifdef VERBOSE
        printf("Total number of blobs - valid or invalid %i\n", no);
    #endif
	objcnt=0;
    int cnting=0;
	Matrix<Matrix<double, 3, 4>, 4, 1> solutions;

	//undistort blobs
    for(int k=0;k<no;k++){
        if(blobs[k].valid == 2){
            #ifdef VERBOSE
                printf("A valid blob!\n");
            #endif
            dx1=(blobs[k].x-cx);
            dy1=(blobs[k].y-cy); 
            radius=dx1*dx1+dy1*dy1;
            dx1=dx1*(1+1*kx1*radius+1*kx2*radius*radius)+cx;
            dy1=dy1*(1+1*ky1*radius+ky2*radius*radius)+cy;
            blobs[k].x = dx1;
            blobs[k].y = dy1;
            //------------------maybe I need to increase also the ball size
		    //inclination of the camera plane also
		    dx1=(blobs[k].x-cx);
		    dy1=(blobs[k].y-cy); 
		    dx1=dx1*cos(theta_calib)/(1-1*dx1*sin(theta_calib)/fx);
		    blobs[k].x=dx1+cx;
		    //change sign maybe
		}
	}
	//algorithm
    for(int i1=0;i1<no;i1++){
        if(blobs[i1].valid!=2) //Get first blob
        	continue; //skip i1
        /*#ifdef VERBOSE
            printf("Passed first blob!\n");
        #endif*/
        x1=blobs[i1].x;
        _y1=blobs[i1].y;
        sz1=blobs[i1].size;
        for(int i2=i1+1;i2<no;i2++){
            if(blobs[i2].valid!=2) //Get second blob
            	continue; //skip i2
            /*#ifdef VERBOSE
                printf("Passed second blob!\n");
            #endif*/
            x2=blobs[i2].x;
            y2=blobs[i2].y;
            sz2=blobs[i2].size;
            for(int i3=i2+1;i3<no;i3++){
                if(blobs[i3].valid!=2) //Get third blob
                	continue; //skip i3
                /*#ifdef VERBOSE
                    printf("Passed third blob!\n");
                #endif*/
                x3=blobs[i3].x;
                y3=blobs[i3].y;
                sz3=blobs[i3].size;
                for(int i4=i3+1;i4<no;i4++){
                    if(blobs[i4].valid!=2) //Get forth blob
                    	continue; //skip i4
                    /*#ifdef VERBOSE
                        printf("Passed fourth blob!\n");
                    #endif*/
                    x4=blobs[i4].x;
                    y4=blobs[i4].y;
                    sz4=blobs[i4].size;
                    cnting++;

                    avg = (sz1 + sz2 + sz3 + sz4)/4;
                    if((4*avg-sz1-sz2-sz3-sz4)/avg<thsz){
                    
                    a1=x1-x2; b1=_y1-y2;
                    a2=x1-x3; b2=_y1-y3;
                    a3=x1-x4; b3=_y1-y4;
                    d1=a1*a1+b1*b1;
                    d2=a2*a2+b2*b2;
                    d3=a3*a3+b3*b3;
                    avg=(2*2)*avg/(M_PI*M_PI);
                    if((d1<avg*thd) &&(d2<avg*thd) &&(d3<avg*thd)){
                    	//evaluate hypothesis
                            cost=100000;
                            vx[0]=x1; vx[1]=x2; vx[2]=x3; vx[3]=x4;
                            vy[0]=_y1; vy[1]=y2; vy[2]=y3; vy[3]=y4;
                         	for(int j1=0;j1<4;j1++){
                                for(int j2=0;j2<4;j2++){
                                    if(j2==j1) 
                                    	continue;
                                    for(int j3=0;j3<4;j3++){
                                    	if(j3==j1) 
                                       		continue;
                                    	if(j3==j2) 
                                       		continue;
                                    	for(int j4=0;j4<4;j4++){
                                        	if(j4==j1) 
                                        		continue;
                                        	if(j4==j2) 
                                        		continue;
                                        	if(j4==j3) 
                                        		continue;
											//get the undistorted points
											dx1=vx[j1]; dy1=vy[j1];
											dx2=vx[j2]; dy2=vy[j2];
											dx3=vx[j3]; dy3=vy[j3];
											dx4=vx[j4]; dy4=vy[j4];
											//points seen by the camera
											feature_vectors << (dx1-cx)/fx, (dx2-cx)/fx, (dx3-cx)/fx, 
																  (dy1-cy)/fy, (dy2-cy)/fy, (dy3-cy)/fy,
																  1			 , 1		  , 1;
											feature_vectors.col(0) = feature_vectors.col(0)/feature_vectors.col(0).norm();
											feature_vectors.col(1) = feature_vectors.col(1)/feature_vectors.col(1).norm();
											feature_vectors.col(2) = feature_vectors.col(2)/feature_vectors.col(2).norm();

											//perform algorithm
											status = P3P::computePoses(feature_vectors, world_points, solutions);
                                            #ifdef SHOW_MATRICES
                                                printf("---------------FEATURE VECTORS-------------------\n");
                                                cout << feature_vectors << endl;
                                                printf("---------------WORLD POINTS----------------------\n");
                                                cout << world_points << endl;
                                                printf("---------------SOLUTIONS FROM P3P-------------------------\n");
                                                printf("----\n");
                                                cout << solutions(0) << endl;
                                                printf("----\n");
                                                cout << solutions(1) << endl;
                                                printf("----\n");
                                                cout << solutions(2) << endl;
                                                printf("----\n");
                                                cout << solutions(3) << endl;
                                                printf("----\n");
                                            #endif 
											//compute cost
											markerh3D << objQuad[9], objQuad[10], objQuad[11], 1;
											markerh2D << dx4, dy4, 1;
                                            #ifdef VERBOSE
                                                printf("STARTED COST EVALUATION!\n");
                                            #endif
											for(unsigned int l=0;l<4;l++){
												Rl = solutions(l).inverse();
												//trans=-Rl*solutions(l).col(3);
												Tl.col(0)=Rl.col(0); Tl.col(1)=Rl.col(1); Tl.col(2)=Rl.col(2); Tl.col(3)=-Rl * solutions(l).col(3);;
												markerh3DProjected = Tl * markerh3D;
                                              	markerh2DPredicted<<fx*(markerh3DProjected(0)/markerh3DProjected(2))+cx,fy*(markerh3DProjected(1)/markerh3DProjected(2))+cy,1;
                                                #ifdef SHOW_MATRICES
                                                    printf("MARKERS 2D ORIGINAL AND PREDICTED\n");
                                                    cout << markerh2D << endl;
                                                    cout << markerh2DPredicted << endl;
                                                    printf("-------------");
                                                #endif
                                                lcost=(markerh2DPredicted-markerh2D).norm();
                                                if(lcost < cost){ //is this the smallest cost so far?
                                                	 //if it is, does it satisfy the conditions of a quadrotor flying? (looking at us, and not turned upside down)
                                                    //conditions set for a standard camera reference frame (x - right, y - down, z - front)
                                                	if(((Rl.col(0) + Rl.col(1)).dot(-Rl*solutions(l).col(3))<0) && (zcam.dot(Rl.col(2))<-0.5)){
                                                		cost=lcost;
														Tsol=Tl;
														aux1=vx[j1]; aux2=vx[j2]; aux3=vx[j3]; aux4=vx[j4];
														aux1y=vy[j1]; aux2y=vy[j2]; aux3y=vy[j3]; aux4y=vy[j4];

                                                	}
                                                }
											}
                    					}
                    				}
                    			}
                    		}
                    		if((cost <= (0.5*(fx+fy)/Tsol(2,3))*0.10) && (objcnt<MAX_MARKERS)){ //validating cost (the error is a function of (f/<distance to object>) ) check if we have already too many markers, discard this marker
                                #ifdef VERBOSE
                                    printf("The cost is lower than required\n");
                                #endif
                                double px,py,r,prx,pry;
     	               			//Get the solution
     	               			//Update the object count that will be on the list
     	               			objcnt++;
     	               			//obtain the center of the object in the image, according to the distortion parameters
			                    trans1 = Tsol.col(3);
			    				prx = trans1(0)*(fx/trans1(2));
			    				pry = trans1(1)*(fy/trans1(2));
			    				//inclination of the camera plane
			    				prx = prx/(cos(theta_calib)+1*sin(theta_calib)*prx/fx);
			    				prx = cx+prx; px=prx-cx;
			    				pry = cy+pry; py=pry-cy;
			    				for(int k=0;k<5;k++){
			    				    r=px*px + py*py;
			    				    px=(prx-cx)/(1+r*kx1+r*r*kx2);
			    				    py=(pry-cy)/(1+r*ky1+r*r*ky2);
			    				}
			    				int i = objcnt-1; //aux variable
			    				//create the square around the detected object, with a size scaled by (f/<distance to object>) and centered on the detected object center	
                                #ifdef VERBOSE
                                    printf("Creating a square around the object...\n");
                                #endif
                                imx0[i] = cx+px-szx*(fx/trans1(2));
								imxf[i] = cx+px+szx*(fx/trans1(2));
								imy0[i] = cy+py-szy*(fy/trans1(2));
								imyf[i] = cy+py+szy*(fy/trans1(2));
								imx0[i] = std::max(imx0[i],0);
								imxf[i] = std::min(imxf[i],nx);
								imy0[i] = std::max(imy0[i],0);
								imyf[i] = std::min(imyf[i],ny);

								//obtain the center of the ID LED in the image, according to the distortion parameters
								trans1=Tsol.col(0)*objQuad[12]+Tsol.col(1)*objQuad[13]+Tsol.col(2)*objQuad[14]+Tsol.col(3);
								prx=trans1(0)*(fx/trans1(2));
								pry=trans1(1)*(fy/trans1(2));
                                #ifdef VERBOSE
                                    printf("Computing the inclination of the camera plane (theta_calib)...\n");
								#endif
                                //inclination of the camera plane
								prx=prx/(cos(theta_calib)+1*sin(theta_calib)*prx/fx);
								prx=cx+prx; px=prx-cx;
								pry=cy+pry; py=pry-cy;
								for(int k=0;k<5;k++){
								    r=px*px + py*py;
								    px=(prx-cx)/(1+r*kx1+r*r*kx2);
								    py=(pry-cy)/(1+r*ky1+r*r*ky2);
								}
                                #ifdef VERBOSE
                                    printf("Creating a square around the ID LED...\n");
								#endif
                                //create the square around the ID LED, with a size scaled by (f/<distance to object>) and centered on the ID LED center
								imx0_LED[i]=cx+px-szLED*(fx/trans1(2));
								imxf_LED[i]=cx+px+szLED*(fx/trans1(2));
								imy0_LED[i]=cy+py-szLED*(fy/trans1(2));
								imyf_LED[i]=cy+py+szLED*(fy/trans1(2));
								imx0_LED[i]=std::max(imx0_LED[i],0);
								imxf_LED[i]=std::min(imxf_LED[i],nx);
								imy0_LED[i]=std::max(imy0_LED[i],0);
								imyf_LED[i]=std::min(imyf_LED[i],ny);
                                #ifdef VERBOSE
                                    printf("Convert from the camera frame to the used camera frame...\n");
                                #endif
								//convert from the standard camera frame (x - right, y - down, z - front) to the used camera frame (x - front, y - left, z - front)
								Tsol = camR * Tsol;
								Tsol.col(3) = Tsol.col(3) + camt;
                                printf("%i\n", objcnt-1);
								Tsols[objcnt-1] = Tsol;
                                
                                #ifdef VERBOSE
                                    printf("DEBUGGING\n");
                                #endif
								//debug
								if(objcnt<=51){
								    for(int k=0;k<no;k++)
								    if(blobs[k].valid==2){
								    	#ifdef VERBOSE
                                            printf("Blob, X: %f Y: %f Size: %f", blobs[k].x, blobs[k].y, blobs[k].size);
								        #endif
                                    }
								}
                    		}
                    	}
                    }
                }
            }
        }
    }
    return objcnt;
}

int track_markers(unsigned char* buf,unsigned int step, int width, int height){

	nx = width;
	ny = height;
    int nlobjects;
	int nlblobs=0, nerase=0;
	int local_blobs[MAX_FEATURES];

	int erase[MAX_MARKERS];
	//int lochm=170,lochM=230,locsm=40,locsM=101,locvm=40,locvM=101;
	int no = detect_blobs(buf, step, blue_blob_vl, blue_blob_vh, blue_blob_hl, blue_blob_hh, blue_blob_sl, blue_blob_sh, 0,nx,0,ny, width, height, BLUE);
    #ifdef VERBOSE
        printf("Ran detec_blobs inside track_markers: %d\n", no);
	#endif
    for(vector<marker>::iterator it = marker_list.begin() ; it != marker_list.end(); ++it){
		
		int max_sz = 0, max_ind = -1;//activate the blobs seen in its proximity
		for(int i = 0; i < 4; i++){ // looking for the size of the fourth smallest blob
			for(int l = 0; l < no; l++){ // loop through all the blobs
		        if((blobs[l].x>it->imx0) && (blobs[l].y>it->imy0) && (blobs[l].x<it->imxf) && (blobs[l].y<it->imyf)){
					if((blobs[l].size > max_sz) && (blobs[l].valid==1)){ // find the biggest blob that is smaller than previous max
						max_sz = blobs[l].size;
						max_ind = l;
					}
				}
			}
		    if(max_ind>=0){ // to account for the case where the marker lags behind the blobs, and the blobs are no longer in the rectangle defined by the marker
				blobs[max_ind].valid = 2;
				local_blobs[nlblobs++] = max_ind;
		    }
		    max_sz = 0;
		}
		//detect possible markers
        int nlobjects = detect_markers(no);
        //case successful select the detected marker that actually is closer to the tracked object
        if(nlobjects>0){
            double min_norm=(Tsols[0].col(3)-it->T.col(3)).norm();
            int min_ind=0;
            for(int l=1;l<nlobjects;l++)
                if((Tsols[l].col(3)-it->T.col(3)).norm()<min_norm){
                    min_norm=(Tsols[l].col(3)-it->T.col(3)).norm();
                    min_ind=l;
                }
            it->imx0=imx0[min_ind]; 
            it->imx0_LED=imx0_LED[min_ind];
            it->imxf=imxf[min_ind]; 
            it->imxf_LED=imxf_LED[min_ind];
            it->imy0=imy0[min_ind]; 
            it->imy0_LED=imy0_LED[min_ind];
            it->imyf=imyf[min_ind]; 
            it->imyf_LED=imyf_LED[min_ind];

            it->T=Tsols[min_ind];
            it->failures=0;
        //case unsuccessful update failure variable on the structure
        }else if(it->failures++>MIN_NUM_FAILURES){ //delete target if not seen for a while
        	erase[nerase++]=it-marker_list.begin(); //increment nerase and store it in erase vector
        } 
        
        for(int l=0;l<nlblobs;l++) //reset the blobs that were used for this object
            blobs[local_blobs[l]].valid=0;
    }
    //delete objects that were not tracked
    for(int k=nerase; k>0; k--){
        marker_list.erase(marker_list.begin()+erase[k-1]);
        nmarkers--;
    }
    //advance the remaining valid to 2
    for(int k=0;k<no;k++){
        if(blobs[k].valid==1){
            blobs[k].valid=2;
        }
    }
    //get potential new markers with the blobs that remain
    nlobjects=detect_markers(no);
    if(nlobjects>0){ //Create a temporary potential new marker
        marker m;
        m.name="unknown";
        memset(m.frequencies,0,10*sizeof(int));
        m.failures=0;
        m.count=0;
        m.filter_count=m.max=0.0;
        m.state=0;
        m.current_freq=-1;
        m.t_old = ros::Time::now().toSec();
        for(int k=0;k<nlobjects;k++){
            m.T=Tsols[k];
            m.imx0=imx0[k]; m.imx0_LED=imx0_LED[k];
            m.imxf=imxf[k]; m.imxf_LED=imxf_LED[k];
            m.imy0=imy0[k]; m.imy0_LED=imy0_LED[k];
            m.imyf=imyf[k]; m.imyf_LED=imyf_LED[k];
            marker_list.push_back(m);
            nmarkers++;
        }
    }

    //Search for red blobs
    for(vector<marker>::iterator it = marker_list.begin() ; it != marker_list.end(); ++it){
        localization=0;
        detect_blobs(buf,step,red_blob_sl,red_blob_sh,red_blob_hl,red_blob_hh,red_blob_vl,red_blob_vh,it->imx0_LED,it->imxf_LED,it->imy0_LED,it->imyf_LED, width, height, RED);
        int old_state=it->state;
        it->state=0;
        for(int l=0;l<no;l++)
            if(blobs[l].valid==1){
                //expected radius of observed red marker
                double pmin=((0.5*(fx+fy)/it->T(0,3))*0.02) * ((0.5*(fx+fy)/it->T(0,3))*0.02)*M_PI; //expected pixel count (we want a percentage of this ammount)
                if((double)blobs[l].size>pmin*RED_BLOB_THRESHOLD_SCALE_FACTOR){ //previous threshold was 15 - it was working fine for frame0
                    it->state=1;
                    break;
                }
            }
        if(old_state!=it->state){
            double t_now = ros::Time::now().toSec();
            double dt = t_now - it->t_old; // time since last transition
            iter_count++;
            it->t_old = t_now; // update start time of new transition
            it->count = round(dt/dt_min-1); // find index that corresponds to detected frequency
            if((it->count < 10) && (it->count >= 0)){ // ensure that frequency is valid
                it->frequencies[it->count]++; // increment element corresponding to detected frequency
                if(it->frequencies[it->count]>it->max)
                    it->max=it->frequencies[it->count];
                it->filter_count++;
                if(it->filter_count>4){ //filter frequency histogram, by decreasing the number of events counted previously, within periods
                    for(int l=0;l<10;l++)
                        if(it->frequencies[l]>0)
                            it->frequencies[l]--; //decrease amount of events of each frequency by one
                    it->filter_count=0; //reset filter counter
                }
            }else{
                ROS_INFO("Detected ID freq. is out of bounds");
            }    
        }

        if(it->max>MARKER_THRESHOLD){ //THRESHOLD           
            //get frequency 
            freq=0,total=0;
            for(int l=0;l<10;l++)
                if(it->frequencies[l]>4){
                    freq+=(double)it->frequencies[l]*l;
                    total+=(double)it->frequencies[l];
                }
            freq/=(double)total;
        
        //get the quadrotor name from the computed frequency
        if(round(freq)>3 || round(freq)<0){
            ROS_INFO("WARNING: selected frequency index out of bounds");
        }
        else{
            it->name = quad_freq_name[(int)round(freq)].c_str(); //this was not allocated!
            //cout << "quad id = " << it->name << endl;
        }
    }
    }
    return nmarkers;
}