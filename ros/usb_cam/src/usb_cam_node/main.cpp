#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>


#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <blob_detection/detections.h>
#include <geometry_msgs/Point.h>

#include "blob.h"



static uint8_t* t_img = NULL;

static ros::Publisher debug_image_pub;
static ros::Publisher detections_pub;

static std::vector<Blob*> blob_vector; 

static int blob_count;

void image_reception_callback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv,"blob_detector");

	ros::NodeHandle nh;

	debug_image_pub=nh.advertise<sensor_msgs::Image>("blob/debug",1);

	ros::Subscriber image_sub=nh.subscribe("usb_cam/image_raw", 1, image_reception_callback); // only 1 in buffer size to drop other images if processing is not finished

	detections_pub=nh.advertise<blob_detection::detections>("blob/detections_by_pc",1);

	
	
	while(ros::ok())
	{

		ros::spinOnce();
		ros::Duration(0.01).sleep();
		

	}

	delete[] t_img;
}


void image_reception_callback(const sensor_msgs::ImageConstPtr& msg)
{
	const sensor_msgs::Image img = *msg;
	sensor_msgs::Image debug_img = *msg;

	int r, g, b, h = 0, s, M, m, c;

	printf("%d %d \n",img.step, img.height);

	if(t_img == NULL)
		t_img = new uint8_t[img.step*img.height];

	double t0 =ros::Time::now().toSec();

	for(unsigned int i = 0; i < img.width; i++)
	for(unsigned int j = 0; j < img.height; j++)
	{
		// do rgb to hsv conversion 
	
		r = img.data[3*i + img.step*j];
		g = img.data[3*i + 1 + img.step*j];
		b = img.data[3*i + 2 + img.step*j];

		if(r > b && r > g)
		{
			M = r;

			if(g > b)
				m = b;
			else
				m = g;

			c = M - m;

			if(c != 0)
			{
				h = (128*(g-b))/(3*c);
				if(h < 0) h += 256;
			}
			else
				h = 0;
		}
		else if(g > b)
		{
			M = g;

			if(r > b)
				m = b;
			else
				m = r;

			c = M - m;

			if(c != 0)
				h = (128*(b-r))/(3*c) + 256/3;
			else
				h = 0;
		}
		else
		{
			M = b;

			if(r > g)
				m = g;
			else
				m = r;

			c = M - m;

			if(c != 0)
				h = (128*(r-g))/(3*c) + 2*256/3;
			else
				h = 0;
		}

		//printf("%f ", h);
		if(M != 0)
			s = (256*c)/M;
		else
			s = 0;
		// apply theshold
		if( (h < 130 && h > 60) && s > 50 && s < 120 && M > 90 && M < 180)
		//if( (h < 300 && h > 150) && s > 100 && v > 150)
			t_img[3*i + img.step*j] = 255;
		else
			t_img[3*i + img.step*j] = 0;

		//printf("HSV %d %d %d \n", h, s, M);

/*		if( r > 150 && g < 110 && b < 110)
			t_img[3*i + img.step*j] = 255;
		else
			t_img[3*i + img.step*j] = 0;*/

		t_img[3*i + 1 + img.step*j] = -1;
		//t_img[3*i + 2 + img.step*j] = 0;
	}

	//printf("time to compute HSV %f [ms]\n", (ros::Time::now().toSec() - t0)*1000);

	blob_vector.clear();
	blob_count = 0;

	//printf("time to clear vector %f [ms]\n", (ros::Time::now().toSec() - t0)*1000);

	


	// search for blobs
	for(unsigned int j = 1; j < img.height-1; j++)
	for(unsigned int i = 1; i < img.width-1; i++)
	{
		if(t_img[3*i + img.step*j] != 0)
		{ // this pixel belongs to a blob

			if(t_img[3*(i-1) + img.step*j] != 0 && i != 1 && j != 1)
			{ // pixel belongs to the blob left
				t_img[3*i + 1 + img.step*j] = t_img[3*(i-1) + 1 + img.step*j];

				blob_vector[t_img[3*i + 1 + img.step*j]]->add_pixel(i, j);

				if(t_img[3*i + img.step*(j-1)] != 0 &&
						t_img[3*(i-1) + 1 + img.step*j] != t_img[3*i + 1 + img.step*(j-1)])
				{ // the pixel is touching two different blobs
					//if(blob_vector[t_img[3*i + 1 + img.step*j]]->is_connected_to() != -1)
					//{ // the blob on the left is already linked, the two blobs need to be linked
						int id_blob_1 = t_img[3*(i-1) + 1 + img.step*j];
						int id_blob_2 = t_img[3*i + 1 + img.step*(j-1)];
						/*printf("start");
						printf("connection: %d %d \n", id_blob_1, blob_vector[id_blob_1]->is_connected_to());
						printf("connection: %d %d \n", id_blob_2, blob_vector[id_blob_2]->is_connected_to());

						ros::Duration d = ros::Duration(0.1, 0);*/
						
						while(blob_vector[id_blob_1]->is_connected_to() != -1)
						{
							id_blob_1 = blob_vector[id_blob_1]->is_connected_to();
							/*printf("ici id: %d %d \n", id_blob_1, blob_vector[id_blob_1]->is_connected_to());
							printf("blob %d at %f, %f \n", id_blob_1,
								blob_vector[id_blob_1]->x(), blob_vector[id_blob_1]->y());	
							d.sleep();	*/				
						}

						
						while(blob_vector[id_blob_2]->is_connected_to() != -1)
						{
							id_blob_2 = blob_vector[id_blob_2]->is_connected_to();
							//printf("la ");
						}

						if(id_blob_1 != id_blob_2)
						{ // if the two blobs are not linked, connect the one with largest id to the other one
							if(id_blob_1 > id_blob_2)
								blob_vector[id_blob_2]->connect_to(id_blob_1);
							else
								blob_vector[id_blob_1]->connect_to(id_blob_2);
						}
						//printf("connection: %d %d \n", id_blob_1, blob_vector[id_blob_1]->is_connected_to());
						//printf("connection: %d %d \n", id_blob_2, blob_vector[id_blob_2]->is_connected_to());
					/*}
					else{
						// the blob on the left is not connected \o/
						blob_vector[t_img[3*i + 1 + img.step*j]]->connect_to(t_img[3*i + 1 + img.step*(j-1)]);
						printf("connection left: %d %d \n", t_img[3*i + 1 + img.step*j], blob_vector[t_img[3*i + 1 + img.step*j]]->is_connected_to());
						printf("connection left: %d %d \n", t_img[3*i + 1 + img.step*(j-1)], blob_vector[t_img[3*i + 1 + img.step*(j-1)]]->is_connected_to());
					}*/
				}
			}
			else if(t_img[3*i + img.step*(j-1)] != 0  && j != 1)
			{ // pixel belongs to the blob above
				t_img[3*i + 1 + img.step*j] = t_img[3*i + 1 + img.step*(j-1)];
				blob_vector[t_img[3*i + 1 + img.step*j]]->add_pixel(i, j);
			}
			else
			{ // new blob !
				blob_vector.push_back(new Blob(i, j, blob_count));
				
				t_img[3*i + 1 + img.step*j] = blob_count;
				//printf("new blob %d at %d, %d \n", blob_count, i, j);
				//printf("connection: %d %d \n", blob_count, blob_vector[blob_count]->is_connected_to());
				blob_count++;
				
			}
		}
	}

	blob_detection::detections det_msg;

	det_msg.header = img.header;

	for(int i = blob_vector.size()-1; i > -1; i--)
	{

		if(blob_vector[i]->is_connected_to() == -1)
		{ // the blob is connected to no other one thus is ready
			if(blob_vector[i]->size() > 3)
			{ // only take blobs large enough
				printf("blob %d of size %d at %f, %f \n", i,
					blob_vector[i]->size(), blob_vector[i]->x(), blob_vector[i]->y());
				geometry_msgs::Point position;
				position.x = blob_vector[i]->x();
				position.y = blob_vector[i]->y();
				position.z = blob_vector[i]->size();
				det_msg.blob.push_back(position);


			}
		}else
		{ // if the blob is connected, add it to the other one
			/*printf("hop\n");
			printf("blob %d of size %d at %f, %f will be added to %d \n", i,
				blob_vector[i]->size(), blob_vector[i]->x(),
				 blob_vector[i]->y(), blob_vector[i]->is_connected_to());*/
			blob_vector[blob_vector[i]->is_connected_to()]->assemble_Blob(blob_vector[i]);
		}
	}

	detections_pub.publish(det_msg);


	printf("time to compute blobs %f [ms]\n", (ros::Time::now().toSec() - t0)*1000);
	
	fillImage(debug_img, "rgb8", img.height, img.width, img.step, t_img);


	debug_image_pub.publish(debug_img);
		
}




