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

#define MAX_FEATURES 500

const int SIZE_THRESHOLD = 100; //Value for 3m which is the max range
const double BLOB_CONSTANT = 2.25;
const int NOISE_ACTIVE = 0;
const double WINDOW_SCALING = 1.5;
const int MIN_NUMBER_BLOBS = 4;
#define BLOB_INFO 1 //Comment or uncomment this to see blob info
#define LIGHT_CALIBRATION 1 //Comment or uncomment this to use the light calibration

int no = 1;  //Blob number - assume there's an initial blob to start
int skip = 0;
int nvalid = 0;
int rnd;
unsigned char col[3]; //Array to hold RGB data 
unsigned char* p; 
int m,M,delta; //m is minimum RGB value, M is maximum RGB value, 
int hue,sat,val; //HSV computed values

int sat_sum = 0;
int val_sum = 0;
int sat_sum_window = 0;
int val_sum_window = 0;

int last_sat_window_avg = 0;
int last_val_window_avg = 0;
int last_sat_total_avg = 0;
int last_val_total_avg = 0;

int window_pixel_number = 0;
int pixel_number = 0;
int max_hue = 0;
int min_hue = 1000;
int center_pixel_max_hue = 0;
int center_pixel_min_hue = 1000;


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
blob last_valid_blobs[MAX_FEATURES];

float size_sum = 0;
float size_avg;
float prev_size_avg;
int first_time = 0;


int sl_adjust = 0;
int sh_adjust = 0;
int hl_adjust = 0;
int hh_adjust = 0;
int vl_adjust = 0;
int vh_adjust = 0;
int _sl = 0;
int _sh = 0;
int _hl = 0;
int _hh = 0;
int _vl = 0;
int _vh = 0;
bool initialization = true;
/*------------------Auxiliary Functions-----------------*/
unsigned char* get_binary_image(){
    return simage;
}
blob* get_blobs(){
    return blobs;
}
void init_binary_img(int width, int height){
    simage=(unsigned char*)malloc(3*width*height*sizeof(unsigned char));
}
int get_valid(){
    return nvalid;
}
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
int threshold(int vl, int vh, int hl, int hh, int sl, int sh){
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

    //memset(last_valid_blobs, 0, MAX_FEATURES*sizeof(blob));
    return 0;
}
void solve_links(){
    no=0; 
    for(int k=MAX_FEATURES-1;k>0;k--){
        //if there is an object and it was not treated yet
        if((objnumb[k]!=0) && (objmark[k]==0)){
            blobs[no].size+=objnumb[k];
            blobs[no].x+=objx[k];
            blobs[no].y+=objy[k];
            int link = objlink[k]; //possible connections that this object contains
            while(link!=0){
                blobs[no].size+=objnumb[link];
                blobs[no].x+=objx[link];
                blobs[no].y+=objy[link];
                objmark[link]=1; //exhaust it after extracting information
                link = objlink[link];
                if(objmark[link]==1)
                    break;
            }
            blobs[no].x/=blobs[no].size;  //Average, count blob
            blobs[no].y/=blobs[no].size; 
            objmark[k]=1; //exhaust it after extracting information
            no++;
        }
    }
}

void apply_noise(int k){
    if((blobs[k].valid)&&(NOISE_ACTIVE)){
        blobs[k].x+=(-12+rand()%25)/100.0;
        blobs[k].y+=(-12+rand()%25)/100.0;
    }
}

void aggregate_blobs(){
    for(int k=0;k<no;k++)
      blobs[k].valid=1;
    for(int k=0;k<no;k++)
        if(blobs[k].valid)
            for(int l=k+1;l<no;l++)
                if(blobs[l].valid){
                    double rl=sqrt(std::max(blobs[k].size,blobs[l].size)/M_PI); //Compute blob radius
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
}
void blob_threshold(){
    for(int k=0;k<no;k++){ //threshold on the blob sizes
        blobs[k].valid=(blobs[k].size<SIZE_THRESHOLD?0:blobs[k].valid);
        apply_noise(k);
    }
}
/*-------------------------------Detect Blobs Function--------------------------------------------*/
int detect_blobs(unsigned char* buf, unsigned int step, int vl, int vh, int hl, int hh,
	int sl, int sh, int xi, int xf, int yi, int yf, int width, int height){

	int is_blob = init(width, height);
    if(initialization){
        _sl = sl;
        _sh = sh;
        _hl = hl;
        _hh = hh;
        _vl = vl;
        _vh = vh;
        
    }else{
        _sl=sl_adjust + _sl;
        _sh=sh_adjust + _sh;
        _hl=hl_adjust + _hl;
        _hh=hh_adjust + _hh;
        _vl=vl_adjust + _vl;
        _vh=vh_adjust + _vh;
    }

    if(hh > 360){ //Verify if hue is over 360, since hue goes from 0 to 360, use rnd parameter
        rnd=1;
        hh=hh-360;
    }else
        rnd=0;

    p=buf+(xi*step)+(yi*nx*step);
    sat_sum = 0; val_sum=0; pixel_number = 0; sat_sum_window=0; val_sum_window=0; window_pixel_number = 0;
    max_hue = 0; min_hue = 1000;
    
    for(int l=yi;l<yf;l++,p+=(nx-(xf-xi))*step){
        for(int k=xi;k<xf;k++,p+=step){
            RGB_HSV(l, k);//rgb->hsv
            sat_sum+=sat;
            val_sum+=val;
            pixel_number++;
            if(initialization == false){
                for(int j=0; j<nvalid; j++){
                    if((k > last_valid_blobs[j].x-sqrt(last_valid_blobs[j].size)/WINDOW_SCALING 
                        && k < last_valid_blobs[j].x+sqrt(last_valid_blobs[j].size)/WINDOW_SCALING) 
                        && (l > last_valid_blobs[j].y-sqrt(last_valid_blobs[j].size)/WINDOW_SCALING 
                            && l < last_valid_blobs[j].y+sqrt(last_valid_blobs[j].size)/WINDOW_SCALING)){
                        //This pixel is inside one of the windows
                        sat_sum_window+=sat;
                        val_sum_window+=val;
                        window_pixel_number++;
                        if(k == (int)last_valid_blobs[j].x && l == (int)last_valid_blobs[j].y){
                            if(hue > center_pixel_max_hue)
                                center_pixel_max_hue = hue;
                            if(hue < center_pixel_min_hue)
                                center_pixel_min_hue = hue;
                        }
                        if(hue > max_hue){
                            max_hue = hue;
                        }
                        if(hue < min_hue){
                            min_hue = hue;
                        }

                    }
                }
            }

            if(skip)
                continue;
            is_blob = threshold(_vl, _vh, _hl, _hh, _sl, _sh); //check color (need to check the threshold)
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
    #ifdef LIGHT_CALIBRATION
        printf("Total image Average sat: %d, Total image Average val: %d\n", sat_sum/pixel_number, val_sum/pixel_number);
        if(initialization == false && window_pixel_number != 0){
            printf("Windows Average sat: %d, Windows Average val: %d, Pixel number in windows: %d\n Max_hue: %d, Min_hue: %d Center_Max_hue: %d, Center_Min_hue:%d \n", sat_sum_window/window_pixel_number, val_sum_window/window_pixel_number, window_pixel_number, max_hue, min_hue, center_pixel_max_hue, center_pixel_min_hue);
            if(last_val_total_avg!=0 && abs(last_val_total_avg-val_sum/pixel_number)>12){
                printf("LIGHTS CHANGED!!! Correcting value (and saturation?)...!\n");
                printf("Value changed to: ");
                _vl = val_sum_window/window_pixel_number;
                _sl = sat_sum_window/window_pixel_number;
            }
            last_val_window_avg = val_sum_window/window_pixel_number;
            last_val_total_avg = val_sum/pixel_number;
            last_sat_window_avg = sat_sum_window/window_pixel_number;
            last_sat_total_avg = sat_sum/pixel_number;
        }
    #endif
    solve_links(); //deliver blobs and solve links -- after this variable no is the total number of blobs
    aggregate_blobs(); //aggregate blobs - (TODO use concept of covariance for thdp) 
    blob_threshold(); //Blob size threshold
    nvalid = 0;
    #ifdef BLOB_INFO
        printf("----------------------------------------------\n");
    #endif
    
    if(initialization == false){
        for(int j=0; j<nvalid; j++){
            #ifdef LIGHT_CALIBRATION
                printf("Window %d Vertex 1: (%f, %f), Vertex 2: (%f, %f), Vertex 3: (%f, %f), Vertex 4: (%f, %f)\n", j, 
                          last_valid_blobs[j].x-sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y+sqrt(last_valid_blobs[j].size),
                          last_valid_blobs[j].x-sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y-sqrt(last_valid_blobs[j].size),
                          last_valid_blobs[j].x+sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y-sqrt(last_valid_blobs[j].size),
                          last_valid_blobs[j].x+sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y+sqrt(last_valid_blobs[j].size)
                          ); 
            #endif
        }

    }
    #ifdef LIGHT_CALIBRATION 
        initialization = false;
    #endif
    size_avg = 0;
    
    for(int k = 0; k<no; k++){ //Get the new blobs
        if(blobs[k].valid){
            #ifdef BLOB_INFO //This was used for testing only
                printf("\t BLOB number: %d\n", k);
                printf("\t \t X: %f\n", blobs[k].x);
                printf("\t \t Y: %f\n", blobs[k].y);
                printf("\t \t Size: %f\n", blobs[k].size);
                printf("\t   Average size: %f\n", size_avg);
            #endif
            last_valid_blobs[nvalid] = blobs[k];
            nvalid++;
            
        }
    }
    nvalid = 0;
    for(int k = 0; k<no; k++){
    	if(blobs[k].valid){
            size_sum+=blobs[k].size;
            last_valid_blobs[nvalid] = blobs[k];
            nvalid++;
            
        }
    }
    size_avg = size_sum/nvalid;
    /*
    //RUDE BRUTE FORCE ATTEMPT
    if(nvalid == EXPECTED_BLOB_NUMBER || nvalid == EXPECTED_BLOB_NUMBER + 1){
        printf("Just right!\n");
        sl_adjust = 0;
        if(size_avg > prev_size_avg){
            prev_size_avg = size_avg;
        }
        

    }else if(nvalid < EXPECTED_BLOB_NUMBER-1){
        printf("Too Bright? %d !!!\n", _sl);
        sl_adjust = -2;
        hl_adjust = -1;
        hh_adjust = -1;
    }else if(nvalid > EXPECTED_BLOB_NUMBER+1){
        printf("Too dark?!!\n");
        sl_adjust =+ 2;
    }*/
    if(nvalid >= MIN_NUMBER_BLOBS){
        #ifdef LIGHT_CALIBRATION
            printf("More than 4 blobs detected!\n");
            if(nvalid%4 == 0){
               
                printf("Right number of blobs \n");
                
                sl_adjust = 0;
                vl_adjust = 0;
            }
            sl_adjust = 0;
        #endif
    }else{ //normally its even nvalid = 0
        #ifdef LIGHT_CALIBRATION
        printf("Less than 4 blobs detected!\n");
        printf("Too Bright? %d !!!\n", _sl);
        #endif
        //sl_adjust = -1;
        //hl_adjust = -1;
        //hh_adjust = -1;
        //vl_adjust = +2;
    }

    #ifdef BLOB_INFO
        printf("----------------------------------------------\n");
    #endif

    return no;
}
