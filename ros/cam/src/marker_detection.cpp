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
//ROS Libraries
#include <ros/ros.h>
//User Libraries
#include "cam/blob_detection.h"
#include "cam/debug.h" //Comment or uncomment this for verbose
#include "p3p.hpp"

using namespace Eigen;
using namespace std;
using namespace P3P_test;

#define MAX_FEATURES 500
#define MAX_MARKERS 50

typedef struct t_marker{ //tracked marker structure
    const char* name;
    MatrixXd T;
    int imx0,imy0,imxf,imyf;
    int imx0_LED,imy0_LED,imxf_LED,imyf_LED;
    int state;
    int frequencies[10];
    int count;
    double filter_count,max;
    int current_freq;
    int failures;
    double t_old;
}marker;

//................................DETECT MARKER VARIABLES.......................
int objcnt=0;
double fx=230,fy=250;
double cx=320,cy=240; //center x, center y should be 320 and 240
double kx1=0.0000055,kx2=2e-10,ky1=0.0000020,ky2=0.0;
double theta_calib = 0.00;
double szx=0.4,szy=0.4, szLED=0.08;
double thsz=1;
double thd=20*20;
int nx = 640;
int ny = 320;

double objQuad[]={0.24,0.0,0.0,
				  0.0,0.24,0.0,
				  0.04,0.055,0.07,
				  -0.0141,-0.007,0.22,
				  0.08,0.08,0.0};

double avg;
double radius,dx1,dx2,dx3,dx4,dy1,dy2,dy3,dy4;
double sz1, sz2, sz3, sz4;
double x1, y1, x2, y2, x3, y3, x4, y4;
double d1, d2, d3, d4;

blob blobs[MAX_FEATURES];

MatrixXd Tl(3,4),
MatrixXd Tsol(3,4);
MatrixXd* Tsols;
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
/*
    world_points.col(0)<<objQuad[0],objQuad[1],objQuad[2];
    world_points.col(1)<<objQuad[3],objQuad[4],objQuad[5];
    world_points.col(2)<<objQuad[6],objQuad[7],objQuad[8];
    zcam<<0,1,0;
    camR<<1,0,0,0,1,0,0,0,1;
    new_frame<<0,0,1,-1,0,0,0,-1,0;
    camR=new_frame*camR;
    camt<<0,0,0;
    Tsols=new Eigen::MatrixXd[MAX_MARKERS];
*/

//................................TRACK MARKER VARIABLES.......................
int localization;

std::vector<marker>* get_markers(){
    return &marker_list;
}

int detect_markers(int no){ //This function extracts markers from blobs
	//GET THE BLOBS
	blobs = get_blobs(); //from blob_detection
	objcnt=0;
    int cnting=0;
	Matrix<Matrix<double, 3, 4>, 4, 1> solutions;

	//undistort blobs
    for(int k=0;k<no;k++){
        if(blobs[k].valid == 2){
            dx1=(blobs[k].x-cx);
            dy1=(blobs[k].y-cy); 
            radius=dx1*dx1+dy1*dy1;
            dx1=dx1*(1+1*kx1*radius+1*kx2*radius*radius)+cx;
            dy1=dy1*(1+1*ky1*radius+ky2*radius*radius)+cy;
            blobs[k].x = dx1;
            blobs[k].y = dy1;
            //cout<<"blob "<<k<<": ("<<dx1<<","<<dy1<<")"<<endl;
            //maybe I need to increase also the ball size

		    //inclination of the camera plane also
		    dx1=(blobs[k].x-cx);
		    dy1=(blobs[k].y-cy); 
		    dx1=dx1*cos(theta_calib)/(1-1*dx1*sin(theta_calib)/fx);
		    blobs[k].x=dx1+cx;
		    //blobs[k].y=dy1*fx/(fx+1*dx1*sin(theta_calib))+cy;//change sign maybe
		}
	}
	//algorithm
    for(int i1=0;i1<no;i1++){
        if(blobs[i1].valid!=2) //Get first blob
        	continue; //skip i1
        x1=blobs[i1].x;
        y1=blobs[i1].y;
        sz1=blobs[i1].size;
        for(int i2=i1+1;i2<no;i2++){
            if(blobs[i2].valid!=2) //Get second blob
            	continue; //skip i2
            x2=blobs[i2].x;
            y2=blobs[i2].y;
            sz2=blobs[i2].sz;
            for(int i3=i2+1;i3<no;i3++){
                if(blobs[i3].valid!=2) //Get third blob
                	continue; //skip i3
                x3=blobs[i3].x;
                y3=blobs[i3].y;
                sz3=blobs[i3].sz;
                for(int i4=i3+1;i4<no;i4++){
                    if(blobs[i4].valid!=2) //Get forth blob
                    	continue; //skip i4
                    x4=blobs[i4].x;
                    y4=blobs[i4].y;
                    sz4=blobs[i4].sz;
                    cnting++;

                    avg = (sz1 + sz2 + sz3 + sz4)/4;
                    //if((4*avg-sz1-sz2-sz3-sz4)/avg<thsz){
                    avg=(2*2)*avg/(PI*PI);
                    d1=(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
                    d2=(x1-x3)*(x1-x3)+(y1-y3)*(y1-y3);
                    d3=(x1-x4)*(x1-x4)+(y1-y4)*(y1-y4);
                    if((d1<avg*thd)
                    	&&(d2<avg*thd)
                    	&&(d3<avg*thd)){
                    	//evaluate hypothesis
                            cost=100000;
                            vx[0]=x1; vx[1]=x2; vx[2]=x3; vx[3]=x4;
                            vy[0]=y1; vy[1]=y2; vy[2]=y3; vy[3]=y4;
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
											status = P3P::computePoses(feature_vectors,world_points,solutions);
											//compute cost
											markerh3D << objQuad[9], objQuad[10], objQuad[11], 1;
											markerh2D << dx4, dy4, 1;
											for(unsigned int l=0;l<4;l++){
												R1 = solutions(l).inverse();
												//trans=-Rl*solutions(l).col(3);
												Tl.col(0)=Rl.col(0); Tl.col(1)=Rl.col(1); Tl.col(2)=Rl.col(2); Tl.col(3)=-Rl * solutions(l).col(3);;
												markerh3DProjected = Tl * markerh3D;
                                              	markerh2DPredicted<<fx*(markerh3DProjected(0)/markerh3DProjected(2))+cx,fy*(markerh3DProjected(1)/markerh3DProjected(2))+cy,1;
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
                    		if((cost <= (0.5*(fx+fy)/Tsol(2,3))*0.10)&&(objcnt<MAX_MARKERS)){ //validating cost (the error is a function of (f/<distance to object>) ) check if we have already too many markers, discard this marker
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
								//inclination of the camera plane
								prx=prx/(cos(theta_calib)+1*sin(theta_calib)*prx/fx);
								prx=cx+prx; px=prx-cx;
								pry=cy+pry; py=pry-cy;
								for(int k=0;k<5;k++){
								    r=px*px + py*py;
								    px=(prx-cx)/(1+r*kx1+r*r*kx2);
								    py=(pry-cy)/(1+r*ky1+r*r*ky2);
								}

								//create the square around the ID LED, with a size scaled by (f/<distance to object>) and centered on the ID LED center
								imx0_LED[i]=cx+px-szLED*(fx/trans1(2));
								imxf_LED[i]=cx+px+szLED*(fx/trans1(2));
								imy0_LED[i]=cy+py-szLED*(fy/trans1(2));
								imyf_LED[i]=cy+py+szLED*(fy/trans1(2));
								imx0_LED[i]=std::max(imx0_LED[i],0);
								imxf_LED[i]=std::min(imxf_LED[i],nx);
								imy0_LED[i]=std::max(imy0_LED[i],0);
								imyf_LED[i]=std::min(imyf_LED[i],ny);
        
								//convert from the standard camera frame (x - right, y - down, z - front) to the used camera frame (x - front, y - left, z - front)
								Tsol = camR * Tsol;
								Tsol.col(3) = Tsol.col(3) + camt;
								Tsols[objcnt-1] = Tsol;

								//debug
								if(objcnt<=51){
								    for(int k=0;k<no;k++)
								    if(blobs[k].valid==2){
								    	printf("Blob: %d, X: %d Y: %d Size: %d", blobs[k].x, blobs[k].y, blobs[k].size);
								    }
								}
                    		}
                    	}
                    //}
                }
            }
        }
    }
    return objcnt;
}

int track_markers(unsigned char* buf,unsigned int step, int vl, int vh, int hl, int hh,
	int sl, int sh, int xi, int xf, int yi, int yf, int width, int height){

	nx = width;
	ny = height;

	int nlblobs=0, nerase=0;
	int local_blobs[MAX_FEATURES];

	int erase[MAX_MARKERS];
	//int lochm=170,lochM=230,locsm=40,locsM=101,locvm=40,locvM=101;
	int no = detect_blobs(buf,step, vl,vh, hl,hh, sl,sh, 0,nx,0,ny);
	for(vector<marker>::iterator it = marker_list.begin() ; it != marker_list.end(); ++it){
		
		int max_sz = 0, max_ind = -1;//activate the blobs seen in its proximity
		for(int i = 0; i < 4; i++){ // looking for the size of the fourth smallest blob
			for(int l = 0; l < no; l++){ // loop through all the blobs
		        if((blobs[l].x>it->imx0) && (blobs[l].y>it->imy0) && (blobs[l].x<it->imxf) && (blobs[l].y<it->imyf)){
					if((blobs[l].sz > max_sz) && (blobs[l].valid==1)){ // find the biggest blob that is smaller than previous max
						max_sz = blobs[l].sz;
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
        }else if(it->failures++>5){ //delete target if not seen for a while
        	erase[nerase++]=it-marker_list.begin();
        } 
        //reset the blobs that were used for this object
        for(int l=0;l<nlblobs;l++)
            blobs[local_blobs[l]].valid=0;

	}



}