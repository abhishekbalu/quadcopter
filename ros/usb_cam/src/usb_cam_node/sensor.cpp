#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "p3p.hpp"
#include "sensor.hpp"
using namespace P3P_test;
using namespace std;

#define MAX_FEATURES 500
#define MAX_MARKERS 50
#define PIXEL_TH 4

#define ABS(x) (((x)>0)?(x):-(x))
#define SGN(x) (((x)>0)?1:-1)
#define ROUND(x) ((((x)-(int)(x))>0.5)?(int)(x)+1:(int)(x))
#define MIN(a,b) (((a)>(b))?(b):(a))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define PI 3.14159

//quadrotor light properties (to be inserted later on a configuration file)
int nquad_freq=4;
double quad_freq[]={0,1,2,3};
string quad_freq_name[]={"quad99","quad98","quad97","quad96"};
const char* unknown = "unknown";

//image paramters
//unsigned char buf[3*640*480];
unsigned char col[3];
unsigned char* p;
int m,M,delta;
int hue,sat,val;
int thb=25;

//blob detection parameters
int no=1;
double thdp=50; //50 for real camera
unsigned char* simage;
int nx,ny;
int lochm=170,lochM=230,locsm=40,locsM=101,locvm=40,locvM=101;
int objnumb[MAX_FEATURES];
int objlink[MAX_FEATURES];
unsigned long objx[MAX_FEATURES];
unsigned long objy[MAX_FEATURES];
blob blobs[MAX_FEATURES];
//int objns[MAX_FEATURES];
//double objnxt[MAX_FEATURES];
//double objnyt[MAX_FEATURES];
int objmark[MAX_FEATURES];
unsigned char state;
int localization;
ofstream floc,fid;
int save_blobs;

//marker extraction parameters (fx=230 fy=230)
double fx=230,fy=250,cx=160,cy=120;
double kx1=0.0000055,kx2=2e-10,ky1=0.0000020,ky2=0.0;
double theta_calib;
double thsz=1,thd=20*20;
//double objQuad[]={0.20,0.0,0.0,0.0,0.20,0.0,0.1,0.1,0.07,0.0,0.0,0.20,0.05,0.05,0.0}; //ideal quadrotor
//double objQuad[]={0.24,0.0,0.0,0.0,0.24,0.0,0.06,0.05,0.06,0.0,0.0,0.21,0.08,0.08,0.0}; //real quadrotor
double objQuad[]={0.24,0.0,0.0,0.0,0.24,0.0,0.04,0.055,0.07,-0.0141,-0.007,0.22,0.08,0.08,0.0}; //real quadrotor
double szx=0.4,szy=0.4, szLED=0.08;
Eigen::Matrix3d camR;
Eigen::Matrix3d new_frame;
Eigen::Vector3d camt;
double x1,x2,x3,x4,y1_,y2,y3,y4,sz1,sz2,sz3,sz4,avg,a1,a2,a3,b1,b2,b3,d1,d2,d3,vx[4],vy[4],radius,dx1,dx2,dx3,dx4,dy1,dy2,dy3,dy4;
int imx0[MAX_MARKERS],imy0[MAX_MARKERS],imxf[MAX_MARKERS],imyf[MAX_MARKERS];
int imx0_LED[MAX_MARKERS],imy0_LED[MAX_MARKERS],imxf_LED[MAX_MARKERS],imyf_LED[MAX_MARKERS];
int status,objcnt=0;
Eigen::Matrix3d feature_vectors, world_points;
Eigen::Vector4d markerh3D;
Eigen::Vector3d markerh3DProjected,markerh2D,markerh2DPredicted,trans,trans1;
Eigen::MatrixXd Rl(3,3),Tl(3,4),Tsol(3,4);
Eigen::MatrixXd* Tsols;
double cost,lcost;
Eigen::Vector3d zcam,obs_direction;
vector<marker> marker_list;
int nmarkers=0;
double dt_min;
int iter_count=0, count2 = 0; //debug
int prev_no = 0; //debug
int noise_active;

//debug variables
double aux1,aux2,aux3,aux4,aux1y,aux2y,aux3y,aux4y;
//int counter;
//Eigen::MatrixXd pMarkersSave(3,6);
//ofstream of;

int track_markers(unsigned char* buf,unsigned int step)
{
    int nlblobs=0,local_blobs[MAX_FEATURES],nlobjects,nerase=0,erase[MAX_MARKERS];
    
    //detect blobs
    localization=1;
    detect_blobs(buf,step,locvm,locvM,lochm,lochM,locsm,locsM,0,nx,0,ny);
    //detect_blobs(buf,step,40,101,170,230,40,101,0,nx,0,ny);
    /*
	// print the number of objects detected
    if(no-prev_no != 0){
        cout<<"no="<<no<<endl;
	prev_no = no;
    }
	*/
    //for each tracked object
    for(vector<marker>::iterator it = marker_list.begin() ; it != marker_list.end(); ++it){
    
    //activate the blobs seen in its proximity
	int max_sz = 0, max_ind = -1;
	for(int i = 0; i < 4; i++){ // looking for the size of the fourth smallest blob
		for(int l = 0; l < no; l++){ // loop through all the blobs
            if((blobs[l].x>it->imx0) && (blobs[l].y>it->imy0) && (blobs[l].x<it->imxf) && (blobs[l].y<it->imyf))
				if((blobs[l].sz > max_sz) && (blobs[l].valid==1)){ // find the biggest blob that is smaller than previous max
					max_sz = blobs[l].sz;
					max_ind = l;
				}
		}
	    if(max_ind>=0){ // to account for the case where the marker lags behind the blobs, and the blobs are no longer in the rectangle defined by the marker
		blobs[max_ind].valid = 2;
		local_blobs[nlblobs++] = max_ind;
	    }
	    max_sz = 0;
	}
	
	/*
	for(int l=0;l<no;l++)
            if(blobs[l].valid==1)
                if((blobs[l].x>it->imx0)&&(blobs[l].y>it->imy0)&&(blobs[l].x<it->imxf)&&(blobs[l].y<it->imyf)){
		    			blobs[l].valid=2;
                    	local_blobs[nlblobs++]=l;
					}
	*/
	
	/*
	// print the size of the valid objects which will be used to detect markers
	cout<<"----START----"<<endl;
	for(int k=0;k<no;k++)
	{
		cout<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<" "<<blobs[k].valid<<endl;
	}
	cout<<"----END----"<<endl;*/
        
        //detect possible markers
        nlobjects=detect_markers();
        //cout<<nlobjects << " nlobjects" << endl;
        //case successful select the detected marker that actually is closer to the tracked object
        if(nlobjects>0){
            double min_norm=(Tsols[0].col(3)-it->T.col(3)).norm();
            int min_ind=0;
            for(int l=1;l<nlobjects;l++)
                if((Tsols[l].col(3)-it->T.col(3)).norm()<min_norm){
                    min_norm=(Tsols[l].col(3)-it->T.col(3)).norm();
                    min_ind=l;
                }
            it->imx0=imx0[min_ind]; it->imx0_LED=imx0_LED[min_ind];
            it->imxf=imxf[min_ind]; it->imxf_LED=imxf_LED[min_ind];
            it->imy0=imy0[min_ind]; it->imy0_LED=imy0_LED[min_ind];
            it->imyf=imyf[min_ind]; it->imyf_LED=imyf_LED[min_ind];

            it->T=Tsols[min_ind];
            it->failures=0;
        }
        //case unsuccessful update failure variable on the structure
        else
            if(it->failures++>5) //delete target if not seen for a while
                erase[nerase++]=it-marker_list.begin();
	//cout << "pos matrix: " << endl << it->T << endl;

        //reset the blobs that were used for this object
        for(int l=0;l<nlblobs;l++)
            blobs[local_blobs[l]].valid=0;
    }
    
    //delete objects that were not tracked
    for(int k=nerase; k>0; k--){
        marker_list.erase(marker_list.begin()+erase[k-1]);
        nmarkers--;
    }
    //advance the remaining valid to 2
    for(int k=0;k<no;k++)
        if(blobs[k].valid==1)
            blobs[k].valid=2;
    
    //get potential new markers with the blobs that remain
    nlobjects=detect_markers();
    if(nlobjects>0){
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
    
    //cout<<marker_list.size() << " quads being tracked" << endl;
    
    //go after red balls
    for(vector<marker>::iterator it = marker_list.begin() ; it != marker_list.end(); ++it){
    
        //detect blobs
        //cout<<it->imx0<<" "<<it->imxf<<" "<<it->imy0<<" "<<it->imyf<<endl;
        localization=0;
        detect_blobs(buf,step,40,101,350,390,40,101,it->imx0_LED,it->imxf_LED,it->imy0_LED,it->imyf_LED);
        
        //if red blob was detected with a sufficient size
        int old_state=it->state;
        double rp,pmin;
        it->state=0;
        for(int l=0;l<no;l++)
            if(blobs[l].valid==1){
                //cout << "size: " << blobs[l].sz << endl;
                rp=(0.5*(fx+fy)/it->T(0,3))*0.02; //expected radious of observed red marker
                pmin=rp*rp*PI; //expected pixel count (we want a percentage of this ammount)
                //cout<<blobs[l].sz<<endl;
                if((double)blobs[l].sz>pmin*0.4){ //previous threshold was 15 - it was working fine for quad99
                    it->state=1;
                    break;
                }
            }
                
        //check transitions
        if(old_state!=it->state){
            double t_now = ros::Time::now().toSec();
            double dt = t_now - it->t_old; // time since last transition
            iter_count++;
            //if(iter_count>17){
            //    cout<<"t_now: "<<t_now<<" t_old: "<<it->t_old<<" dt: "<<dt<<" result: "<<ROUND(dt/dt_min-1)<<endl;
            //	iter_count=0;
            //}
            it->t_old = t_now; // update start time of new transition
            it->count = ROUND(dt/dt_min-1); // find index that corresponds to detected frequency
            if( (it->count < 10) && (it->count >= 0) ){ // ensure that frequency is valid
                it->frequencies[it->count]++; // increment element corresponding to detected frequency
                if(it->frequencies[it->count]>it->max)
                    it->max=it->frequencies[it->count];
                it->filter_count++;
                if(it->filter_count>4){ //filter frequency histogram, by decreasing the number of events counted previously, within periods
                    for(int l=0;l<10;l++)
                        if(it->frequencies[l]>0)
                            it->frequencies[l]--; //decrease amount of events of each frequency by one
                    it->filter_count=0; //reset filter counter
                    //if(it->max>0) it->max--;
                }
            }
            else{
                //cout << "detected id freq " << it->count << " out of range" << endl;
                ;
            }
        }
        //for(int l=0;l<10;l++)
        //  cout<<(unsigned int)it->frequencies[l]<<",";
        //cout<<endl;
        
        //check quadrotor if ready
        if(it->max>4){ //THRESHOLD
        
            //get frequency 
            double freq=0,total=0;
            for(int l=0;l<10;l++)
                if(it->frequencies[l]>4){
                    freq+=(double)it->frequencies[l]*l;
                    total+=(double)it->frequencies[l];
                }
            freq/=(double)total;
	    //count2++;
	    //if(count2 > 5){
	    //	cout << "avg freq for " << it->name << " = " << (double)freq << endl;
	    //	count2 = 0;
	    //}
            //cout<<freq<<endl;
            /*
            //compare with the quadrotors
            if(it->current_freq>=0){
                if(ABS(quad_freq[it->current_freq]-freq)<=0.5){
                    strcpy(it->name,quad_freq_name[it->current_freq].c_str());
                    //cout<<it->name<<endl;
                    continue;
                }
            }
                
            //find another quadrotor
            double min=ABS(quad_freq[0]-freq);
            int min_ind=0;
            for(int l=1;l<nquad_freq;l++)
               if(ABS(quad_freq[l]-freq)<min){
                   min_ind=l;
                   min=ABS(quad_freq[l]-freq);
               }
            if(min<=0.5){ //THRESHOLD
                strcpy(it->name,quad_freq_name[min_ind].c_str());
                it->current_freq=min_ind;
                //cout<<it->name<<endl;
            }
            */

        //get the quadrotor name from the computed frequency
        if(ROUND(freq)>3 || ROUND(freq)<0){
            //cout << "WARNING: selected frequency index out of bounds" << endl;
            ;
        }
        else{
            it->name = quad_freq_name[ROUND(freq)].c_str(); //this was not allocated!
            //cout << "quad id = " << it->name << endl;
        }
        
    }
    
    return nmarkers;
}

int detect_markers()
{
    //important variables for the algorithm and initializations
    Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;
    objcnt=0;
    int cnting=0;

    //undistort blobs
    for(int k=0;k<no;k++)
        if(blobs[k].valid==2){
            dx1=(blobs[k].x-cx);
            dy1=(blobs[k].y-cy); 
            radius=dx1*dx1+dy1*dy1;
            dx1=dx1*(1+1*kx1*radius+1*kx2*radius*radius)+cx;
            dy1=dy1*(1+1*ky1*radius+ky2*radius*radius)+cy;
            blobs[k].x=dx1;
            blobs[k].y=dy1;
            //cout<<"blob "<<k<<": ("<<dx1<<","<<dy1<<")"<<endl;
            //maybe I need to increase also the ball size

	    //inclination of the camera plane also
	    dx1=(blobs[k].x-cx);
	    dy1=(blobs[k].y-cy); 
	    dx1=dx1*cos(theta_calib)/(1-1*dx1*sin(theta_calib)/fx);
	    blobs[k].x=dx1+cx;
	    //blobs[k].y=dy1*fx/(fx+1*dx1*sin(theta_calib))+cy;//change sign maybe
	}

    //algorithm
    for(int i1=0;i1<no;i1++){
        if(blobs[i1].valid!=2) continue;
        x1=blobs[i1].x;
        y1_=blobs[i1].y;
        sz1=blobs[i1].sz;
        for(int i2=i1+1;i2<no;i2++){
            if(blobs[i2].valid!=2) continue;
            x2=blobs[i2].x;
            y2=blobs[i2].y;
            sz2=blobs[i2].sz;
            for(int i3=i2+1;i3<no;i3++){
                if(blobs[i3].valid!=2) continue;
                x3=blobs[i3].x;
                y3=blobs[i3].y;
                sz3=blobs[i3].sz;
                for(int i4=i3+1;i4<no;i4++){
                    if(blobs[i4].valid!=2) continue;
                    x4=blobs[i4].x;
                    y4=blobs[i4].y;
                    sz4=blobs[i4].sz;
                    cnting++;

                    //size threshold
                    avg=(sz1+sz2+sz3+sz4)/4;
                    
                    if((4*avg-sz1-sz2-sz3-sz4)/avg<thsz)
                    //if((abs(sz1/avg - avg)<thsz)&&(abs(sz2/avg - avg)<thsz)&&(abs(sz3/avg - avg)<thsz)&&(abs(sz3/avg - avg)<thsz))
                    {
                        //distance threshold
                        a1=x1-x2; b1=y1_-y2;
                        a2=x1-x3; b2=y1_-y3;
                        a3=x1-x4; b3=y1_-y4;
                        d1=a1*a1+b1*b1;
                        d2=a2*a2+b2*b2;
                        d3=a3*a3+b3*b3;
                        avg=(2*2)*avg/(PI*PI);
                        if((d1<avg*thd)&&(d2<avg*thd)&&(d3<avg*thd))
                        {
                            //evaluate hypothesis
                            cost=100000;
                            vx[0]=x1; vx[1]=x2; vx[2]=x3; vx[3]=x4;
                            vy[0]=y1_; vy[1]=y2; vy[2]=y3; vy[3]=y4;
                            for(int j1=0;j1<4;j1++){
                                for(int j2=0;j2<4;j2++){
                                    if(j2==j1) continue;
                                    for(int j3=0;j3<4;j3++){
                                       if(j3==j1) continue;
                                       if(j3==j2) continue;
                                       for(int j4=0;j4<4;j4++){
                                           if(j4==j1) continue;
                                           if(j4==j2) continue;
                                           if(j4==j3) continue;

                                           //get the undistorted points
                                           dx1=vx[j1]; dy1=vy[j1];
                                           dx2=vx[j2]; dy2=vy[j2];
                                           dx3=vx[j3]; dy3=vy[j3];
                                           dx4=vx[j4]; dy4=vy[j4];

                                           //points seen by the camera
                                           feature_vectors.col(0)<<(dx1-cx)/fx,(dy1-cy)/fy,1;
                                           feature_vectors.col(1)<<(dx2-cx)/fx,(dy2-cy)/fy,1;
                                           feature_vectors.col(2)<<(dx3-cx)/fx,(dy3-cy)/fy,1;
                                           feature_vectors.col(0) = feature_vectors.col(0)/feature_vectors.col(0).norm();
                                           feature_vectors.col(1) = feature_vectors.col(1)/feature_vectors.col(1).norm();
                                           feature_vectors.col(2) = feature_vectors.col(2)/feature_vectors.col(2).norm();
                                           /*feature_vectors.col(0)<<(vx[j1]-cx)/fx,(vy[j1]-cy)/fy,1;
                                           feature_vectors.col(1)<<(vx[j2]-cx)/fx,(vy[j2]-cy)/fy,1;
                                           feature_vectors.col(2)<<(vx[j3]-cx)/fx,(vy[j3]-cy)/fy,1;
                                           feature_vectors.col(0) = feature_vectors.col(0)/feature_vectors.col(0).norm();
                                           feature_vectors.col(1) = feature_vectors.col(1)/feature_vectors.col(1).norm();
                                           feature_vectors.col(2) = feature_vectors.col(2)/feature_vectors.col(2).norm();*/
                                           
                                           //perform algorithm
                                           status=P3P::computePoses(feature_vectors,world_points,solutions);

                                           //compute cost
                                           markerh3D<<objQuad[9],objQuad[10],objQuad[11],1;
                                           markerh2D<<dx4,dy4,1;
                                           //markerh2D<<vx[j4],vy[j4],1;
                                           //cout<<"COST_START"<<endl;
                                           for(unsigned l=0;l<4;l++){
                                               Rl.col(0)=solutions(l).col(0);
                                               Rl.col(1)=solutions(l).col(1);
                                               Rl.col(2)=solutions(l).col(2);
                                               Rl=Rl.inverse();
                                               trans=-Rl*solutions(l).col(3);
                                               Tl.col(0)=Rl.col(0); Tl.col(1)=Rl.col(1); Tl.col(2)=Rl.col(2); Tl.col(3)=trans;
                                               markerh3DProjected=Tl*markerh3D;
                                               markerh2DPredicted<<fx*(markerh3DProjected(0)/markerh3DProjected(2))+cx,fy*(markerh3DProjected(1)/markerh3DProjected(2))+cy,1;
                                               lcost=(markerh2DPredicted-markerh2D).norm();

                                               //is this the smallest cost so far?
                                               if(lcost<cost){
                                                    /*cout<<"Tsol:"<<endl;
                                                    cout<<Tl<<endl;
                                                    cout<<lcost<<endl;*/

                                                    //if it is, does it satisfy the conditions of a quadrotor flying? (looking at us, and not turned upside down)
                                                    //conditions set for a standard camera reference frame (x - right, y - down, z - front)
                                                    if(((Rl.col(0) + Rl.col(1)).dot(trans)<0) && (zcam.dot(Rl.col(2))<-0.5)){
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

			    //validating cost (the error is a function of (f/<distance to object>) )
                //also, if we have already too many markers, discard this marker
			    if((cost <= (0.5*(fx+fy)/Tsol(2,3))*0.10)&&(objcnt<MAX_MARKERS)){
    				double px,py,r,prx,pry;

    				//colect solution
                    //first update the number of objects that the list will contain
    				objcnt++;

        //cout << "flag1" << endl; //*FLAG
    				
    				//obtain the center of the object in the image, according to the distortion parameters
                    trans1=Tsol.col(3);
    				prx=trans1(0)*(fx/trans1(2));
    				pry=trans1(1)*(fy/trans1(2));
    				//pry=pry*(fx+prx*sin(theta_calib))/fx; //inclination of the camera plane
    				prx=prx/(cos(theta_calib)+1*sin(theta_calib)*prx/fx);
    				prx=cx+prx; px=prx-cx;
    				pry=cy+pry; py=pry-cy;
    				for(int k=0;k<5;k++){
    				    r=px*px + py*py;
    				    px=(prx-cx)/(1+r*kx1+r*r*kx2);
    				    py=(pry-cy)/(1+r*ky1+r*r*ky2);
    				}

                    //create the square around the detected object, with a size scaled by (f/<distance to object>) and centered on the detected object center
    				imx0[objcnt-1]=cx+px-szx*(fx/trans1(2));
    				imxf[objcnt-1]=cx+px+szx*(fx/trans1(2));
    				imy0[objcnt-1]=cy+py-szy*(fy/trans1(2));
    				imyf[objcnt-1]=cy+py+szy*(fy/trans1(2));
    				imx0[objcnt-1]=MAX(imx0[objcnt-1],0);
    				imxf[objcnt-1]=MIN(imxf[objcnt-1],nx);
    				imy0[objcnt-1]=MAX(imy0[objcnt-1],0);
    				imyf[objcnt-1]=MIN(imyf[objcnt-1],ny);

        //cout << "flag2" << endl; //*FLAG

    				//obtain the center of the ID LED in the image, according to the distortion parameters
    				trans1=Tsol.col(0)*objQuad[12]+Tsol.col(1)*objQuad[13]+Tsol.col(2)*objQuad[14]+Tsol.col(3);
    				prx=trans1(0)*(fx/trans1(2));
    				pry=trans1(1)*(fy/trans1(2));
    				//pry=pry*(fx+prx*sin(theta_calib))/fx; //inclination of the camera plane
    				prx=prx/(cos(theta_calib)+1*sin(theta_calib)*prx/fx);
    				prx=cx+prx; px=prx-cx;
    				pry=cy+pry; py=pry-cy;
    				for(int k=0;k<5;k++){
    				    r=px*px + py*py;
    				    px=(prx-cx)/(1+r*kx1+r*r*kx2);
    				    py=(pry-cy)/(1+r*ky1+r*r*ky2);
    				}

                    //create the square around the ID LED, with a size scaled by (f/<distance to object>) and centered on the ID LED center
    				imx0_LED[objcnt-1]=cx+px-szLED*(fx/trans1(2));
    				imxf_LED[objcnt-1]=cx+px+szLED*(fx/trans1(2));
    				imy0_LED[objcnt-1]=cy+py-szLED*(fy/trans1(2));
    				imyf_LED[objcnt-1]=cy+py+szLED*(fy/trans1(2));
    				imx0_LED[objcnt-1]=MAX(imx0_LED[objcnt-1],0);
    				imxf_LED[objcnt-1]=MIN(imxf_LED[objcnt-1],nx);
    				imy0_LED[objcnt-1]=MAX(imy0_LED[objcnt-1],0);
    				imyf_LED[objcnt-1]=MIN(imyf_LED[objcnt-1],ny);

        //cout << "flag3" << endl; //*FLAG
                               
    				//convert from the standard camera frame (x - right, y - down, z - front) to the used camera frame (x - front, y - left, z - front)
    				Tsol=camR*Tsol;
    				Tsol.col(3)=Tsol.col(3)+camt;
    				Tsols[objcnt-1]=Tsol;

                    //debug
                    if(objcnt==51){
                        for(int k=0;k<no;k++)
                        if(blobs[k].valid==2){
                            cout<<"blob: "<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<endl;
                        }
                    }

    				//cout<<camR<<endl<<camt<<endl;
    				//cout<<Tsol<<endl;
    				//cout<<" object "<<objcnt<<endl;
    				//cout<<Tsol<<endl;
    				//cout<<cost<<endl;
    				//cout<<aux1<<" "<<aux2<<" "<<aux3<<" "<<aux4<<endl;
    				//cout<<aux1y<<" "<<aux2y<<" "<<aux3y<<" "<<aux4y<<endl;
    				//cout<<"["<<imx0[objcnt-1]<<","<<imxf[objcnt-1]<<","<<imy0[objcnt-1]<<","<<imyf[objcnt-1]<<"]"<<endl;

    				//save markers and velocity (velocity is invalid here)
    				//pMarkersSave.col(0)=Tsol.col(0);
    				//pMarkersSave.col(1)=Tsol.col(1);
    				//pMarkersSave.col(2)=Tsol.col(2);
    				//pMarkersSave.col(3)=Tsol.col(3);
    				//pMarkersSave.col(4)<<0,0,0;
    				//pMarkersSave.col(5)<<counter,0,0;
    				//of<<pMarkersSave<<endl;
			    }
				else{
					/*for(int k=0;k<no;k++)
						if(blobs[k].valid==2){
							cout<<"blob: "<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<endl;
						}*/
				}
    //cout << "flag18" << endl; //*FLAG
                        }
                    }
                }
            }
        }
    }
    //cout<<"cnting:"<<cnting<<endl;
    //counter++;

    return objcnt;
}

//extract blobs from the image
int detect_blobs(unsigned char* buf,unsigned int step,int vl,int vh,int hl,int hh,int sl,int sh,int xi,int xf,int yi,int yf)
{
    int round=0,is_blob=0;
    memset(objx,0,MAX_FEATURES*sizeof(int));
    memset(objy,0,MAX_FEATURES*sizeof(int));
    memset(objnumb,0,MAX_FEATURES*sizeof(int));
    memset(objlink,0,MAX_FEATURES*sizeof(int));
    memset(objmark,0,MAX_FEATURES*sizeof(int));
    no=1; //it will define the number of the object (warning in C vectors start at 0 not 1)
    memset(blobs,0,MAX_FEATURES*sizeof(blob));
    //memset(simage,0,640*480*sizeof(char));
    //simage does not need to be initialize as we only check for pass pixels - which are already rewritten
    
    //initialize the color threshold
    if(hh>360){
        round=1;
        hh=hh-360;
        hl=hl;
    }
    else
        round=0;

    //extract blobs
    p=buf+(xi*step)+(yi*nx*step);
    //for(int l=0;l<ny-40;l++){
    //    for(int k=0;k<nx;k++,p+=step){
    for(int l=yi;l<yf;l++,p+=(nx-(xf-xi))*step){
        for(int k=xi;k<xf;k++,p+=step){

            //convert to hsv
            *(col)=*(p+0);
            m=M=0;
            *(col+1)=*(p+1); M=((*(col+1)>*(col+M))?1:M); m=((*(col+1)<*(col+m))?1:m);
            *(col+2)=*(p+2); M=((*(col+2)>*(col+M))?2:M); m=((*(col+2)<*(col+m))?2:m);
            delta=col[M]-col[m];
            if((((int)(*(col+M)))!=0)&&(delta>0))
            {
                hue=120*M+(60*((int)(col[(M+1)%3])-(int)(col[(M+2)%3])))/delta;
                hue+=((hue<0)?360:0);
                sat=(100*(col[M]-col[m]))/col[M];
                val=(100*col[M])/255;
            }
            else
                {simage[l*nx+k]=0; continue;}

            //check color (need to check the threshold)
            //if((hue>170)&&(hue<230)&&(sat>40)&&(val>40))
            if(round)
                is_blob=((hue>hl)||(hue<hh))&&(sat>sl)&&(sat<sh)&&(val>vl)&&(val<vh);
            else
                is_blob=(hue>hl)&&(hue<hh)&&(sat>sl)&&(sat<sh)&&(val>vl)&&(val<vh);
            if(is_blob)
            //if((MIN(hue,360-hue)<thb)&&(sat>40)) 
            {
                state=0;

                if((l>0)&&(k>0))
                    state|=(simage[(l-1)*nx + (k-1)]>0);
                if(l>0)
                    state|=((simage[(l-1)*nx + k]>0)<<1);
                if(k>0)
                    state|=((simage[l*nx+(k-1)]>0)<<2);

                //if new blob
                if(state==0){
                    simage[l*nx+k]=no;
                    objnumb[no]=1;
                    objx[no]+=k;
                    objy[no]+=l;
                    no++;
                }
                else
                {
                    //if connected to the pixel behind
                    if((state&4)!=0)
                    {
                        simage[l*nx+k]=simage[l*nx+(k-1)];
                        objnumb[simage[l*nx+(k-1)]]++;
                        objx[simage[l*nx+(k-1)]]+=k;
                        objy[simage[l*nx+(k-1)]]+=l;

                        //if also connected to the pixel above (merge objects)
                        if((state&2)!=0)
                        {
                            if(simage[l*nx+(k-1)] != simage[(l-1)*nx+k])
                                objlink[simage[l*nx+(k-1)]]=simage[(l-1)*nx+k];
                        }
                    }
                    //if connected to the upper pixel (I am a lonely pixel - joining the upper group)
                    else if((state&2)!=0)
                    {
                        simage[l*nx+k]=simage[(l-1)*nx+k];
                        objnumb[simage[(l-1)*nx+k]]++;
                        objx[simage[(l-1)*nx+k]]+=k;
                        objy[simage[(l-1)*nx+k]]+=l;
                    }
                    //if connected to pixel in diagonal (this should not happen)
                    else if((state&1)!=0)
                    {
                        simage[l*nx+k]=simage[(l-1)*nx+(k-1)];
                        objnumb[simage[(l-1)*nx+(k-1)]]++;
                        objx[simage[(l-1)*nx+(k-1)]]+=k;
                        objy[simage[(l-1)*nx+(k-1)]]+=l;
                    }
                    else
                        simage[l*nx+k]=0;

                }
            }
            else
                {simage[l*nx+k]=0; continue;}
        }
    }
    //cout<<"max_blobs:"<<no<<endl;

    //deliver blobs
    //solve links
    no=0; //no will serve as new object counter - to see the previous objects we just need to see the non-zero values
    for(int k=MAX_FEATURES-1;k>0;k--){
        //if there is an object and it was not treated yet
        if((objnumb[k]!=0)&&(objmark[k]==0)){
            blobs[no].sz+=objnumb[k];
            blobs[no].x+=objx[k];
            blobs[no].y+=objy[k];

            //possible connections that this object contains
            int l=objlink[k];
            while(l!=0){
                blobs[no].sz+=objnumb[l];
                blobs[no].x+=objx[l];
                blobs[no].y+=objy[l];
                objmark[l]=1; //exhaust it after extracting information
                l=objlink[l];
                if(objmark[l]==1)
                    break;
            }

            //do the average an procede to the next
            blobs[no].x/=blobs[no].sz; blobs[no].x++; //don't know if adding 1 to the pixels, due to the index effect in C, changes much
            blobs[no].y/=blobs[no].sz; blobs[no].y++;
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
	    double rl=MAX(blobs[k].sz,blobs[l].sz); rl=sqrt(rl/PI);
	    double a1=blobs[k].x, a2=blobs[k].y;
	    double b1=blobs[l].x, b2=blobs[l].y;
	    //if((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2)<thdp){
	    if((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2)<(2.25*rl*rl)){
	      double n=blobs[k].sz+blobs[l].sz, n1=blobs[k].sz/n, n2=blobs[l].sz/n;
              blobs[k].x=blobs[k].x*n1+blobs[l].x*n2;
              blobs[k].y=blobs[k].y*n1+blobs[l].y*n2;
              blobs[k].sz=n;
              blobs[l].valid=0;
            }
          }  
    
    //print blobs into file except the ones that were aggregated
    if(save_blobs){
	for(int k=0;k<no;k++)
	{
	    if(blobs[k].valid){
		if(localization==1)
		    floc<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<endl;
		else
		    fid<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<endl;
	    }
	}
	if(localization==1)
	    floc<<"end_scan"<<endl;
	else
	    fid<<"end_scan"<<endl;
    }

    //threshold on the blob sizes
    for(int k=0;k<no;k++){
      blobs[k].valid=(blobs[k].sz<PIXEL_TH?0:blobs[k].valid);
      if((blobs[k].valid)&&(noise_active)){
        blobs[k].x+=(-12+rand()%25)/100.0;
        blobs[k].y+=(-12+rand()%25)/100.0;
      }
    }

    int nvalid = 0;
    for(int k = 0; k<no; k++){
	if(blobs[k].valid)
	    nvalid++;
    }
    //cout << nvalid << " valid blobs" << endl;
 	/*   
    cout<<"----START----"<<endl;
    for(int k=0;k<no;k++)
    {
        cout<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<" "<<blobs[k].valid<<endl;
    }
    cout<<"----END----"<<endl;
    */
    return no;
}

/*Eigen::MatrixXd* sensor_init(int width,int height,
                             double fx_l,double fy_l,double cx_l,double cy_l,double kx1_l,double kx2_l,double ky1_l,double ky2_l,
                             double r,double p,double y,double camr,double camp,double camy,double camtx,double camty, double camtz)
*/
Eigen::MatrixXd* sensor_init(int width,int height,
                             double fx_l,double fy_l,double cx_l,double cy_l,double kx1_l,double kx2_l,double ky1_l,double ky2_l,double theta_l,
			     double dt_min_l,
			     int lochm_l,int lochM_l,int locsm_l,int locsM_l,int locvm_l,int locvM_l,
			     int save_blobs_l,int noise_active_l)
{
    //initialize algorithm parameters
    fx=fx_l;
    fy=fy_l;
    cx=cx_l;
    cy=cy_l;
    kx1=kx1_l;
    kx2=kx2_l;
    ky1=ky1_l;
    ky2=ky2_l;
    theta_calib=theta_l;
    nx=width;
    ny=height;
    lochm=lochm_l; lochM=lochM_l;
    locsm=locsm_l; locsM=locsM_l;
    locvm=locvm_l; locvM=locvM_l;
    save_blobs=save_blobs_l;
    noise_active=noise_active_l;
    dt_min = dt_min_l;

    cout<<"localization thresholds:\nhm="<<lochm<<" hM="<<lochM<<"\nsm="<<locsm<<" sM="<<locsM<<"\nvm="<<locvm<<" vM="<<locvM<<endl;

    //initialize blob files
    if(save_blobs){
	cout<<"openning blob files"<<endl;
	floc.open("/home/linaro/rosws/sandbox/usb_cam/localization_blobs.txt");
	fid.open("/home/linaro/rosws/sandbox/usb_cam/id_blobs.txt");
    }

    //initialize transformations
    //camR<<-0.6956,0,+0.718,0,-1,0,0.718,0,+0.6956;
    camR<<1,0,0,0,1,0,0,0,1;
    new_frame<<0,0,1,-1,0,0,0,-1,0;
    camR=new_frame*camR;
    
    camt<<0,0,0;
    zcam<<0,1,0;

    cout<<camR<<endl;
    cout<<camt<<endl;

    //theoretical points to be used in the p3p algorithm (static in this code)
    world_points.col(0)<<objQuad[0],objQuad[1],objQuad[2];
    world_points.col(1)<<objQuad[3],objQuad[4],objQuad[5];
    world_points.col(2)<<objQuad[6],objQuad[7],objQuad[8];
    
    //operational structures
    Tsols=new Eigen::MatrixXd[MAX_MARKERS];
    simage=(unsigned char*)malloc(nx*ny*sizeof(unsigned char));
//   of.open("/home/duartecdias/ros/blob_detection/rposes_quad.txt");
    //counter=0;
    return Tsols;
}

unsigned char* get_binary_image()
{
    return simage;
}

blob* get_blobs()
{
    return blobs;
}

void close_blob_files()
{
    if(save_blobs){
    cout<<"closing blob files"<<endl;
    floc.close();
    fid.close();
    }
}

std::vector<marker>* get_markers(){
    return &marker_list;
}
