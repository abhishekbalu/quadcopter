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
//ROS Libraries
#include <ros/ros.h>
//User Libraries
#include "cam/blob_detection.h"
//#include "cam/debug.h" //Comment or uncomment this for verbose

#define PI 3.1415926535897
#define MAX_FEATURES 500
#define PIXEL_TH 10
#define BLOB_CONSTANT 2.25
#define NOISE_ACTIVE 0

int no=1;  //Blob number - assume there's an initial blob to start
int skip = 0;
int nvalid = 0;
int rnd;
unsigned char col[3]; //Array to hold RGB data 
unsigned char* p; 
int m,M,delta; //m is minimum RGB value, M is maximum RGB value, 
int hue,sat,val; //HSV computed values
unsigned char* simage; //binary image
int nx,ny; //image width and image height
unsigned char state; //Variable to check if a blob is a new blob or is connected
//Arrays for interation
int objnumb[MAX_FEATURES];
int objlink[MAX_FEATURES];
unsigned long objx[MAX_FEATURES];
unsigned long objy[MAX_FEATURES];
int objmark[MAX_FEATURES];
blob blobs[MAX_FEATURES];

/*------------------Auxiliary Functions-----------------*/
void RGB_HSV(int l, int k){
    #ifdef VERBOSE
        printf("Processing RGB to HSV... Hue overbound: %i\n",rnd);
    #endif
    skip=0;
    *(col)=*(p+0);
    m=0;
    M=0;
    *(col+1)=*(p+1); 
    if((*(col+1)>*(col+M)))
        M=1;
    if((*(col+1)<*(col+m)))
        m=1;
    
    *(col+2)=*(p+2);
    if((*(col+2)>*(col+M)))
        M=2;
    if((*(col+2)<*(col+m)))
        m=2;
    delta=col[M]-col[m];
    
    if((((int)(*(col+M)))!=0)&&(delta>0)){
        hue=120*M+(60*((int)(col[(M+1)%3])-(int)(col[(M+2)%3])))/delta;
        if(hue<0)
            hue+=360;
        sat=(100*(col[M]-col[m]))/col[M];
        val=(100*col[M])/255;
    }
    else{
        simage[l*nx+k]=0; 
        skip=1;
    }
    #ifdef VERBOSE
        printf("Hue: %d, Sat: %d, Val: %d\n", hue, sat, val);
    #endif
}
int threshold(int vl, int vh, int hl, int hh,
    int sl, int sh){
    int is_blob;
    if(rnd)
        is_blob=((hue>hl)||(hue<hh))&&(sat>sl)&&(sat<sh)&&(val>vl)&&(val<vh);
    else
        is_blob=(hue>hl)&&(hue<hh)&&(sat>sl)&&(sat<sh)&&(val>vl)&&(val<vh);
    #ifdef VERBOSE
        printf("Is blob?: %d\n", is_blob);
    #endif
    return is_blob;
}
void computeState(int l, int k){
    state=0;
    if((l>0)&&(k>0))
        state|=(simage[(l-1)*nx + (k-1)]>0);
    if(l>0)
        state|=((simage[(l-1)*nx + k]>0)<<1);
    if(k>0)
        state|=((simage[l*nx+(k-1)]>0)<<2);
}
void processState(int l, int k){
    if(state==0){//if new blob
        #ifdef VERBOSE
           printf("New blob\n");
        #endif
        simage[l*nx+k]=no;
        objnumb[no]=1;
        objx[no]+=k;
        objy[no]+=l;
        no++;
    }else{
        #ifdef VERBOSE
           printf("Not new blob\n");
        #endif
        //if connected to the pixel behind
        if((state&4)!=0){ //Mask state with 4 (100)
            simage[l*nx+k]=simage[l*nx+(k-1)];
            objnumb[simage[l*nx+(k-1)]]++;
            objx[simage[l*nx+(k-1)]]+=k;
            objy[simage[l*nx+(k-1)]]+=l;
            if((state&2)!=0){ //if also connected to the pixel above (merge objects)
                if(simage[l*nx+(k-1)] != simage[(l-1)*nx+k])
                    objlink[simage[l*nx+(k-1)]]=simage[(l-1)*nx+k];
            }
        }else if((state&2)!=0){//if connected to the upper pixel 
            simage[l*nx+k]=simage[(l-1)*nx+k];
            objnumb[simage[(l-1)*nx+k]]++;
            objx[simage[(l-1)*nx+k]]+=k;
            objy[simage[(l-1)*nx+k]]+=l;
        }else if((state&1)!=0){ //if connected to pixel in diagonal (this should not happen)
            simage[l*nx+k]=simage[(l-1)*nx+(k-1)];
            objnumb[simage[(l-1)*nx+(k-1)]]++;
            objx[simage[(l-1)*nx+(k-1)]]+=k;
            objy[simage[(l-1)*nx+(k-1)]]+=l;
        }else //This means that there is no blob there and the pixel isn't considered 
            simage[l*nx+k]=0;
    }
}

int init(int width, int height){
    nx = width; //Set nx and ny
    ny = height;
    rnd = 0; //Set rnd to be 0 by default, assuming hh < 360
    no=1; //this will define the number of the object c++ vec start at 0
    memset(objx,0,MAX_FEATURES*sizeof(int));
    memset(objy,0,MAX_FEATURES*sizeof(int));
    memset(objnumb,0,MAX_FEATURES*sizeof(int));
    memset(objlink,0,MAX_FEATURES*sizeof(int));
    memset(objmark,0,MAX_FEATURES*sizeof(int));
    memset(blobs,0,MAX_FEATURES*sizeof(blob));
    memset(simage,0,width*height*sizeof(unsigned char));
    return 0;
}
/*-------------------------------Main code---------------------*/
int detect_blobs(unsigned char* buf, unsigned int step, int vl, int vh, int hl, int hh,
	int sl, int sh, int xi, int xf, int yi, int yf, int width, int height){

	int is_blob = init(width, height);

    if(hh > 360){ //Verify if hue is over 360, since hue goes from 0 to 360, use rnd parameter
        rnd=1;
        hh=hh-360;
    }else
        rnd=0;
    //Extract blobs
    p=buf+(xi*step)+(yi*nx*step);
    for(int l=yi;l<yf;l++,p+=(nx-(xf-xi))*step){
        for(int k=xi;k<xf;k++,p+=step){
            RGB_HSV(l, k);//rgb->hsv
            if(skip)
                continue;

            is_blob = threshold(vl, vh, hl, hh, sl, sh); //check color (need to check the threshold)
            if(is_blob){ 
                #ifdef VERBOSE
            	   printf("This pixel belongs to a blob\n");
                #endif

                computeState(l, k);
                processState(l, k);
            }else{
                #ifdef VERBOSE
                    printf("This pixel does not belong to a blob\n");
                #endif
                simage[l*nx+k]=0; 
                continue;
            }
        }
    }
    //deliver blobs and solve links
    no=0; 
    for(int k=MAX_FEATURES-1;k>0;k--){
        //if there is an object and it was not treated yet
        if((objnumb[k]!=0)&&(objmark[k]==0)){
            blobs[no].size+=objnumb[k];
            blobs[no].x+=objx[k];
            blobs[no].y+=objy[k];
            //possible connections that this object contains
            int l=objlink[k];
            while(l!=0){
                blobs[no].size+=objnumb[l];
                blobs[no].x+=objx[l];
                blobs[no].y+=objy[l];
                objmark[l]=1; //exhaust it after extracting information
                l=objlink[l];
                if(objmark[l]==1)
                    break;
            }
            //Average, count blob
            blobs[no].x/=blobs[no].size; 
            blobs[no].y/=blobs[no].size; 
            objmark[k]=1; //exhaust it after extracting information
            no++;
        }
    }
    //aggregate blobs - (TODO use concept of covariance for thdp)
    for(int k=0;k<no;k++)
      blobs[k].valid=1;
    for(int k=0;k<no;k++)
      if(blobs[k].valid)
        for(int l=k+1;l<no;l++)
	  if(blobs[l].valid){
	    double rl=sqrt(std::max(blobs[k].size,blobs[l].size)/PI); //Compute blob radius
	    //Compute pixels to check if they are to be aggregated
	    double a1=blobs[k].x;
        double a2=blobs[k].y;
	    double b1=blobs[l].x;
        double b2=blobs[l].y;
	    if((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2)<(BLOB_CONSTANT*rl*rl)){
	    	double n=blobs[k].size+blobs[l].size; 
            double n1=blobs[k].size/n; 
            double n2=blobs[l].size/n;
        	blobs[k].x=blobs[k].x*n1+blobs[l].x*n2;
        	blobs[k].y=blobs[k].y*n1+blobs[l].y*n2;
        	blobs[k].size=n;
        	blobs[l].valid=0;
            }
          }  

    for(int k=0;k<no;k++){ //threshold on the blob sizes
		blobs[k].valid=(blobs[k].size<PIXEL_TH?0:blobs[k].valid);
		if((blobs[k].valid)&&(NOISE_ACTIVE)){
			blobs[k].x+=(-12+rand()%25)/100.0;
			blobs[k].y+=(-12+rand()%25)/100.0;
		}
    }
    nvalid = 0;
    for(int k = 0; k<no; k++){
    	if(blobs[k].valid)
    	    nvalid++;
    }
    #ifdef VERBOSE
        printf("Number of valid blobs: %d\n", nvalid);
    #endif
        printf("Number of valid blobs: %d\n", nvalid);
    return no;
}

unsigned char* get_binary_image(){
    return simage;
}

blob* get_blobs(){
    return blobs;
}
void init_binary_img(int width, int height){
	simage=(unsigned char*)malloc(3*width*height*sizeof(unsigned char));
}
