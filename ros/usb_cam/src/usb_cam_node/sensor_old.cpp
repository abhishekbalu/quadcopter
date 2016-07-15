#include <iostream>
#include <fstream>
#include "p3p.hpp"
#include "sensor.hpp"
using namespace P3P_test;
using namespace std;

#define MAX_FEATURES 500
#define MAX_MARKERS 50
#define PIXEL_TH 4

#define ABS(x) (((x)>0)?(x):-(x))
#define MIN(a,b) (((a)>(b))?(b):(a))
#define PI 3.14159

//image paramters
//unsigned char buf[3*640*480];
unsigned char col[3];
unsigned char* p;
int m,M,delta;
int hue,sat,val;
int thb=25;

//blob detection parameters
int no=1;
double thdp=50;
unsigned char* simage;
int nx,ny;
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

//marker extraction parameters (fx=230 fy=230)
double fx=230,fy=250,cx=160,cy=120;
double kx1=0.0000055,kx2=2e-10,ky1=0.0000020,ky2=0.0;
double thsz=1,thd=20*20;
//double objQuad[]={0.2,0.0,0.0,0.0,0.2,0.0,0.1,0.1,0.07,0.0,0.0,0.2}; ideal quadrotor
double objQuad[]={0.24,0.0,0.0,0.0,0.24,0.0,0.06,0.06,0.07,0.0,0.0,0.19};
Eigen::Matrix3d camR;
Eigen::Matrix3d new_frame;
Eigen::Vector3d camt;
double x1,x2,x3,x4,y1_,y2,y3,y4,sz1,sz2,sz3,sz4,avg,a1,a2,a3,b1,b2,b3,d1,d2,d3,vx[4],vy[4],r,dx1,dx2,dx3,dx4,dy1,dy2,dy3,dy4;
int status,objcnt=0;
Eigen::Matrix3d feature_vectors, world_points;
Eigen::Vector4d markerh3D;
Eigen::Vector3d markerh3DProjected,markerh2D,markerh2DPredicted,trans;
Eigen::MatrixXd Rl(3,3),Tl(3,4),Tsol(3,4);
Eigen::MatrixXd* Tsols;
double cost,lcost;
Eigen::Vector3d zcam,obs_direction;

//debug variables
double aux1,aux2,aux3,aux4,aux1y,aux2y,aux3y,aux4y;
int counter;
Eigen::MatrixXd pMarkersSave(3,6);
ofstream f;

int detect_markers()
{
    //important variables for the algorithm and initializations
    Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;
    objcnt=0;
    int cnting=0;

    //undistort blobs
    for(int k=0;k<no;k++)
        if(blobs[k].valid){
            dx1=(blobs[k].x-cx);
            dy1=(blobs[k].y-cy); 
            r=dx1*dx1+dy1*dy1;
            dx1=dx1*(1+1*kx1*r+1*kx2*r*r)+cx;
            dy1=dy1*(1+1*ky1*r+ky2*r*r)+cy;
            blobs[k].x=dx1;
            blobs[k].y=dy1;
            //maybe I need to increase also the ball size
        }

    //algorithm
    for(int i1=0;i1<no;i1++){
        if(!blobs[i1].valid) continue;
        x1=blobs[i1].x;
        y1_=blobs[i1].y;
        sz1=blobs[i1].sz;
        for(int i2=i1+1;i2<no;i2++){
            if(!blobs[i2].valid) continue;
            x2=blobs[i2].x;
            y2=blobs[i2].y;
            sz2=blobs[i2].sz;
            for(int i3=i2+1;i3<no;i3++){
                if(!blobs[i3].valid) continue;
                x3=blobs[i3].x;
                y3=blobs[i3].y;
                sz3=blobs[i3].sz;
                for(int i4=i3+1;i4<no;i4++){
                    if(!blobs[i4].valid) continue;
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
                                               if(((Rl.col(0) + Rl.col(1)).dot(trans)<0) && (zcam.dot(Rl.col(2))<-0.5)){
                                                   if(lcost<cost){
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

                            //display solution
                            objcnt++;
                            //cout<<camR<<endl<<camt<<endl;
                            //cout<<Tsol<<endl;
                            Tsol=camR*Tsol;
                            Tsol.col(3)=Tsol.col(3)+camt;
                            Tsols[objcnt-1]=Tsol;
                            //cout<<" object "<<objcnt<<endl;
                            //cout<<Tsol<<endl;
                            //cout<<cost<<endl;
                            //cout<<aux1<<" "<<aux2<<" "<<aux3<<" "<<aux4<<endl;
                            //cout<<aux1y<<" "<<aux2y<<" "<<aux3y<<" "<<aux4y<<endl;

                            //save markers and velocity (velocity is invalid here)
                            pMarkersSave.col(0)=Tsol.col(0);
                            pMarkersSave.col(1)=Tsol.col(1);
                            pMarkersSave.col(2)=Tsol.col(2);
                            pMarkersSave.col(3)=Tsol.col(3);
                            pMarkersSave.col(4)<<0,0,0;
                            pMarkersSave.col(5)<<counter,0,0;
                            f<<pMarkersSave<<endl;
                        }
                    }
                }
            }
        }
    }
    //cout<<"cnting:"<<cnting<<endl;
    counter++;

    return objcnt;
}

//extract blobs from the image
int detect_blobs(unsigned char* buf,unsigned int step)
{
    memset(objx,0,MAX_FEATURES*sizeof(int));
    memset(objy,0,MAX_FEATURES*sizeof(int));
    memset(objnumb,0,MAX_FEATURES*sizeof(int));
    memset(objlink,0,MAX_FEATURES*sizeof(int));
    memset(objmark,0,MAX_FEATURES*sizeof(int));
    no=1; //it will define the number of the object (warning in C vectors start at 0 not 1)
    memset(blobs,0,MAX_FEATURES*sizeof(blob));
    //memset(simage,0,640*480*sizeof(char));
    //simage does not need to be initialize as we only check for pass pixels - which are already rewritten

    //extract blobs
    p=buf;
    for(int l=0;l<ny;l++){
        for(int k=0;k<nx;k++,p+=step){

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
            if((hue>170)&&(hue<230)&&(sat>40)&&(val>40))            	
            //if((MIN(ABShue-,360-hue)<thb)&&(sat>40))
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
            double a1=blobs[k].x, a2=blobs[k].y;
            double b1=blobs[l].x, b2=blobs[l].y;
            if((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2)<thdp){
              double n=blobs[k].sz+blobs[l].sz, n1=blobs[k].sz/n,n2=blobs[l].sz/n;
              blobs[k].x=blobs[k].x*n1+blobs[l].x*n2;
              blobs[k].y=blobs[k].y*n1+blobs[l].y*n2;
              blobs[k].sz=n;
              blobs[l].valid=0;
            }
          }  
    
    //threshold on the blob sizes
    for(int k=0;k<no;k++)
      blobs[k].valid=(blobs[k].sz<PIXEL_TH?0:blobs[k].valid);

    /*for(int k=0;k<no;k++)
    {
        cout<<blobs[k].x<<" "<<blobs[k].y<<" "<<blobs[k].sz<<" "<<blobs[k].valid<<endl;
    }*/
    return no;
}

/*Eigen::MatrixXd* sensor_init(int width,int height,
                             double fx_l,double fy_l,double cx_l,double cy_l,double kx1_l,double kx2_l,double ky1_l,double ky2_l,
                             double r,double p,double y,double camr,double camp,double camy,double camtx,double camty, double camtz)
*/
Eigen::MatrixXd* sensor_init(int width,int height,
                             double fx_l,double fy_l,double cx_l,double cy_l,double kx1_l,double kx2_l,double ky1_l,double ky2_l)
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
    nx=width;
    ny=height;

    //initialize transformations
    //camR<<-0.6956,0,+0.718,0,-1,0,0.718,0,+0.6956;
    camR<<1,0,0,0,1,0,0,0,1;
    new_frame<<0,0,1,-1,0,0,0,-1,0;
    camR=new_frame*camR;
    /*Eigen::Matrix3d Rroll,Rpitch,Ryaw;
    cout<<r<<" "<<p<<" "<<y<<endl;
    Rroll<<1,0,0,0,cos(r),-sin(r),0,sin(r),cos(r);  //0.174
    Rpitch<<cos(p),0,-sin(p),0,1,0,sin(p),0,cos(p); //-0.05
    Ryaw<<cos(y),-sin(y),0,sin(y),cos(y),0,0,0,1;   //-0.018
    camR=Ryaw*Rpitch*Rroll*camR;
    Rroll<<1,0,0,0,cos(camr),-sin(camr),0,sin(camr),cos(camr);
    Rpitch<<cos(camp),0,-sin(camp),0,1,0,sin(camp),0,cos(camp);
    Ryaw<<cos(camy),-sin(camy),0,sin(camy),cos(camy),0,0,0,1;
    camR=Ryaw*Rpitch*Rroll*camR;
    camt<<camtx,camty,camtz;*/
    camt<<0,0,0;
    zcam<<0,1,0;
    //camR=new_frame.inverse().eval()*camR;
    //camt=new_frame.inverse().eval()*camt;
    //camR=camInc3*camInc2*camInc1*camR;
    //camt=camInc3*camInc2*camInc1*camt;
    cout<<camR<<endl;
    cout<<camt<<endl;

    //theoretical points to be used in the p3p algorithm (static in this code)
    world_points.col(0)<<objQuad[0],objQuad[1],objQuad[2];
    world_points.col(1)<<objQuad[3],objQuad[4],objQuad[5];
    world_points.col(2)<<objQuad[6],objQuad[7],objQuad[8];
    
    //operational structures
    Tsols=new Eigen::MatrixXd[MAX_MARKERS];
    simage=(unsigned char*)malloc(nx*ny*sizeof(unsigned char));
    f.open("/home/duartecdias/ros/blob_detection/rposes_quad.txt");
    counter=0;
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
