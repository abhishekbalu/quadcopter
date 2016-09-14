//C++ Libraries
#include <iostream>
#include <ostream>
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

//Namespaces
using namespace std;
//Macros
//#define BLOB_INFO 1 //Comment or uncomment this to see blob info
#define LIGHT_CALIBRATION 1 //Comment or uncomment this to use the light calibration
//Constants
#define MAX_FEATURES 500
#define PI 3.14159
const int HIT_NUMBERS = 15;

const int SIZE_THRESHOLD = 15; //Value for 3m which is the max range
const double BLOB_CONSTANT = 2.25;
const int NOISE_ACTIVE = 0;
const double WINDOW_SCALING = 1; //Window scaling for the light adaption algorithm
const int BLUE = 1;
const int RED = 0;

int MIN_NUMBER_BLOBS = 4; //defaults to blue markers
int NUMBER_OF_QUADS = 1;
int MAX_NUMBER_BLOBS = 4; //defaults to blue markers

int last_colour = 1;

int no = 1;  //Blob number - assume there's an initial blob to start
int skip = 0;
int nvalid = 0;
int rnd;
unsigned char col[3]; //Array to hold RGB data 
unsigned char* p; 
int m,M,delta; //m is minimum RGB value, M is maximum RGB value, 
int hue,sat,val; //HSV computed values
int try_original=1;
int sat_sum = 0;
int val_sum = 0;
int sat_sum_window = 0;
int val_sum_window = 0;
int red = 0;
int blue = 0;
int last_sat_window_avg = 0;
int last_val_window_avg = 0;
int last_sat_total_avg = 0;
int last_val_total_avg = 0;
ofstream test_outputfile;
int window_pixel_number = 0;
int pixel_number = 0;
int max_hue = 0;
int min_hue = 1000;
int center_pixel_max_hue = 0;
int center_pixel_min_hue = 1000;

double blob_min_size = 1000000;

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
int hits = 0;

int sl_adjust = 0, hl_adjust = 0, vl_adjust = 0;
int sh_adjust = 0, hh_adjust = 0, vh_adjust = 0;

int _sl = 0, _hl = 0, _vl = 0;
int _sh = 0, _hh = 0, _vh = 0;

bool initialization = true;

/*------------------Auxiliary Functions(these can be used in other files)-----------------*/
ostream& operator<<(ostream& out, const blob& b){ //operator overloading to print blobs
    out << "\t \t x: " << b.x << "\n";
    out << "\t \t y: " << b.y << "\n";
    out << "\t \t size: " << b.size << "\n"; 
    return out;
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
int get_valid(){
    return nvalid;
}
double get_min_blob_size(){
    return blob_min_size;
}
/*------------------Blob Functions(these are internal)-----------------*/
inline void RGB_HSV(int l, int k){
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
inline int threshold(int vl, int vh, int hl, int hh, int sl, int sh){
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
inline void computeState(int l, int k){
    state=0;
    if((l>0)&&(k>0))
        state|=(simage[(l-1)*nx + (k-1)]>0);
    if(l>0)
        state|=((simage[(l-1)*nx + k]>0)<<1);
    if(k>0)
        state|=((simage[l*nx+(k-1)]>0)<<2);
}
inline void processState(int l, int k){
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
inline int init(int width, int height){
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
inline void solve_links(){
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

inline void apply_noise(int k){
    if((blobs[k].valid)&&(NOISE_ACTIVE)){
        blobs[k].x+=(-12+rand()%25)/100.0; //Put the blob center slightly off-center
        blobs[k].y+=(-12+rand()%25)/100.0; //Put the blob center slightly off-center
    }
}

inline void aggregate_blobs(){
    for(int k=0;k<no;k++)
      blobs[k].valid=1;
    for(int k=0;k<no;k++)
        if(blobs[k].valid)
            for(int l=k+1;l<no;l++)
                if(blobs[l].valid){
                    double rl=sqrt(std::max(blobs[k].size,blobs[l].size)/M_PI); //Compute blob radius
                    //Compute pixels to check if they are to be aggregated
                    if((blobs[k].x-blobs[l].x)*(blobs[k].x-blobs[l].x)+(blobs[k].y-blobs[l].y)*(blobs[k].y-blobs[l].y)<(BLOB_CONSTANT*rl*rl)){
                        blobs[k].x=blobs[k].x*(blobs[k].size/(blobs[k].size+blobs[l].size))+blobs[l].x*(blobs[l].size/(blobs[k].size+blobs[l].size));
                        blobs[k].y=blobs[k].y*(blobs[k].size/(blobs[k].size+blobs[l].size))+blobs[l].y*(blobs[l].size/(blobs[k].size+blobs[l].size));
                        blobs[k].size=blobs[k].size+blobs[l].size;
                        blobs[l].valid=0;
                    }
                }     
}
inline void blob_threshold(){
    for(int k=0;k<no;k++){ //threshold on the blob sizes
        blobs[k].valid=(blobs[k].size<SIZE_THRESHOLD?0:blobs[k].valid);
        apply_noise(k);
    }
}
/*-------------------------------Detect Blobs Function--------------------------------------------*/
int detect_blobs(unsigned char* buf, unsigned int step, int vl, int vh, int hl, int hh,
	int sl, int sh, int xi, int xf, int yi, int yf, int width, int height, int colour, int toggle){
    
    #ifdef VERBOSE
        printf("Toggle: %d\n", toggle);
    #endif

	int is_blob = init(width, height);
    if(colour != last_colour){
        initialization = true;
        last_colour = colour;
    }
    if(toggle == 0)
        initialization = true;

    if(initialization){
        test_outputfile.open("test_data.csv");
        _sl = sl;_hl = hl;_vl = vl;
        _sh = sh;_hh = hh;_vh = vh;
    }else{
        _sl=sl_adjust + _sl;_hl=hl_adjust + _hl;_vl=vl_adjust + _vl;
        _sh=sh_adjust + _sh;_hh=hh_adjust + _hh;_vh=vh_adjust + _vh;
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
            //We assume in the following computations that the correct value of blobs is found upon beggining
            sat_sum+=sat;
            val_sum+=val;
            pixel_number++;

            if(initialization == false){
                for(int j=0; j<nvalid; j++){
                    //Need to divide by PI since size is PI*radius^2
                    if((k > last_valid_blobs[j].x-sqrt(last_valid_blobs[j].size/PI)/WINDOW_SCALING 
                        && k < last_valid_blobs[j].x+sqrt(last_valid_blobs[j].size/PI)/WINDOW_SCALING) 
                        && (l > last_valid_blobs[j].y-sqrt(last_valid_blobs[j].size/PI)/WINDOW_SCALING 
                        && l < last_valid_blobs[j].y+sqrt(last_valid_blobs[j].size/PI)/WINDOW_SCALING)){
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
                        if(hue > max_hue)
                            max_hue = hue;
                        if(hue < min_hue)
                            min_hue = hue;

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
        #ifdef VERBOSE
		printf("Total image Average sat: %d, Total image Average val: %d\n", sat_sum/pixel_number, val_sum/pixel_number);
        #endif
	if(colour == BLUE){
            if(initialization == false && window_pixel_number != 0){
                
			printf("Windows Average sat: %d, Windows Average val: %d, Pixel number in windows: %d\n Max_hue: %d, Min_hue: %d Center_Max_hue: %d, Center_Min_hue:%d \n", sat_sum_window/window_pixel_number, val_sum_window/window_pixel_number, window_pixel_number, max_hue, min_hue, center_pixel_max_hue, center_pixel_min_hue);
            
            //DATA LOGGING
            if(nvalid >= MIN_NUMBER_BLOBS && nvalid < MAX_NUMBER_BLOBS+1){
                std::stringstream ss;
                ss << sat_sum_window/window_pixel_number << "," << val_sum_window/window_pixel_number << "," << val_sum/pixel_number << "," << sat_sum/pixel_number;
                std::string s = ss.str();
                test_outputfile << s << endl;
            }
            if(val_sum/pixel_number > val_sum_window/window_pixel_number){
                printf("Something is wrong, image value is too high...\n");

            }

		if( ((last_val_total_avg!=0) && (abs(last_val_total_avg-val_sum/pixel_number)>13)) || (hits > HIT_NUMBERS ) ){
                    #ifdef VERBOSE
			printf("LIGHTS CHANGED!!! Correcting value (and saturation?)...!\n");
                    #endif
                    
		            _vl = ((val_sum_window/window_pixel_number)+(vl))/2; //Average so the transition isn't too aggressive
                    _sl = ((sat_sum_window/window_pixel_number)+(sl))/2; //Average so the transition isn't too aggressive
		   
                    hits = 0;
                }
                last_val_window_avg = val_sum_window/window_pixel_number;
                last_val_total_avg = val_sum/pixel_number;
                last_sat_window_avg = sat_sum_window/window_pixel_number;
                last_sat_total_avg = val_sum/pixel_number;
            }
        }
    #endif
    solve_links(); //deliver blobs and solve links -- after this variable no is the total number of blobs
    aggregate_blobs(); //aggregate blobs - (TODO use concept of covariance for thdp) 
    blob_threshold(); //Blob size threshold

    #ifdef BLOB_INFO
        printf("----------------------------------------------\n");
    #endif
    /*
    if(initialization == false){
        for(int j=0; j<nvalid; j++){ //light cal windows
            #ifdef LIGHT_CALIBRATION
                printf("Window %d Vertex 1: (%f, %f), Vertex 2: (%f, %f), Vertex 3: (%f, %f), Vertex 4: (%f, %f)\n", j, 
                          last_valid_blobs[j].x-sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y+sqrt(last_valid_blobs[j].size),
                          last_valid_blobs[j].x-sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y-sqrt(last_valid_blobs[j].size),
                          last_valid_blobs[j].x+sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y-sqrt(last_valid_blobs[j].size),
                          last_valid_blobs[j].x+sqrt(last_valid_blobs[j].size), last_valid_blobs[j].y+sqrt(last_valid_blobs[j].size)
                          ); 
            #endif
        }

    }*/
    #ifdef LIGHT_CALIBRATION 
        if(toggle == 1)
            initialization = false;
    #endif

    nvalid = 0;
    for(int k = 0; k<no; k++){ //Get the new blobs
        if(blobs[k].valid){
            #ifdef BLOB_INFO //This was used for testing only
                printf("\t BLOB number: %d\n", k);
                cout << blobs[k] << endl; 
            #endif
            if(blobs[k].size < blob_min_size)
                blob_min_size = blobs[k].size;
            last_valid_blobs[nvalid] = blobs[k];
            nvalid++;
            
        }
    }
    if(initialization == false){
        if(colour == BLUE){
            if(nvalid >= MIN_NUMBER_BLOBS && nvalid < MAX_NUMBER_BLOBS+1){
                #ifdef LIGHT_CALIBRATION
		   #ifdef VERBOSE
                    	printf("Between %d  blobs detected and %d blobs detected\n", MIN_NUMBER_BLOBS, MAX_NUMBER_BLOBS);
                    #endif
		    if(nvalid%MIN_NUMBER_BLOBS == 0){
                       #ifdef VERBOSE
                        	printf("Right number of blobs \n");
                        #endif
			          sl_adjust = 0;
                        vl_adjust = 0;
                    }else{
		    	
		    }
                    sl_adjust = 0;
                #endif
            }else{ //normally it derails to nvalid = 0
                #ifdef LIGHT_CALIBRATION
                	#ifdef VERBOSE
				printf("Less than %d blobs  detected or more than %d detected! %d\n", MIN_NUMBER_BLOBS, MAX_NUMBER_BLOBS);
                	#endif
		#endif
                if(nvalid == 0 || try_original == 1){
                   #ifdef VERBOSE 
		   printf("RESETTING TO ORIGINAL PARAMS\n");
		   #endif
                    /*_sl = sl;_hl = hl;_vl = vl;
                    _sh = sh;_hh = hh;_vh = vh;*/
                    //_vl = val_sum_window/window_pixel_number;
                    //_sl = sat_sum_window/window_pixel_number;
		            try_original=0;
                }else{
                   hits++; 
                }
                
                
                //sl_adjust = -1;
                //hl_adjust = -1;
                //hh_adjust = -1;
                //vl_adjust = +2;
            }
        }
    }
    #ifdef LIGHT_CALIBRATION
        	printf("_sl: %d _vl: %d Hits: %d\n", _sl, _vl, hits);
   
    #endif

    #ifdef BLOB_INFO
        printf("----------------------------------------------\n");
    #endif

    return no;
}
