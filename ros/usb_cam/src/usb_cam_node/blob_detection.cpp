

#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <list>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "blob_detection.h"

namespace
{
	uint8_t* t_img = NULL;

	std::vector<Blob*> blob_vector; 

	int blob_count;
}

void detect_blobs(usb_cam_camera_image_t* img, usb_cam::detections &blobs, BlobParams &blob_params, ros::Publisher det_pub, bool publish_debug)
{
	//printf("hap\n");

	uint8_t r, g, b;
	int h = 0, s, M, c;
	int bottom_bondary = img->height/4, upper_bondary = 3*img->height/4;

	//printf("%d %d \n",img->width, img->height);

	if(t_img == NULL)
	{
		t_img = new uint8_t[3*img->width*img->height];
		for(unsigned int i = 0; i < img->width; i++)
		for(unsigned int j = 0; j < img->height; j++)
		{
			t_img[3*i + 3*img->width*j] = 0;
			t_img[3*i + 1 + 3*img->width*j] = 0;
			t_img[3*i + 2 + 3*img->width*j] = 0;
		}

	}

	//double t0 =ros::Time::now().toSec();

	for(unsigned int i = 0; i < img->width; i++)
	for(unsigned int j = bottom_bondary; j < upper_bondary; j++)
	{
		// do rgb to hsv conversion 
	
		r = img->image[3*i + 3*img->width*j];
		g = img->image[3*i + 1 + 3*img->width*j];
		b = img->image[3*i + 2 + 3*img->width*j];

		if(r > b && r > g)
		{
			M = r;

			if(g > b)
				c = M - b;
			else
				c = M - g;

			

			if(c != 0)
			{
				h = (43*(g-b))/(c);
				if(h < 0) h += 256;
			}
			else
				h = 0;
		}
		else if(g > b)
		{
			M = g;

			if(r > b)
				c = M - b;
			else
				c = M - r;

			if(c != 0)
				h = (43*(b-r))/(c) + 256/3;
			else
				h = 0;
		}
		else
		{
			M = b;

			if(r > g)
				c = M - g;
			else
				c = M - r;

			if(c != 0)
				h = (43*(r-g))/(c) + 2*256/3;
			else
				h = 0;
		}

		//printf("%f ", h);
		if(M != 0)
			s = (256*c)/M;
		else
			s = 0;
		// apply theshold
		if( (h < blob_params.max_hue && h > blob_params.min_hue) &&
				 s > blob_params.min_sat && s < blob_params.max_sat &&
				 M > blob_params.min_val && M < blob_params.max_val)
		//if( (h < 300 && h > 150) && s > 100 && v > 150)
			t_img[3*i + 3*img->width*j] = 255;
		else
			t_img[3*i + 3*img->width*j] = 0;

/*		if( r > 150 && g < 110 && b < 110)
			t_img[3*i + img->width*j] = 255;
		else
			t_img[3*i + img->width*j] = 0;*/

		t_img[3*i + 1 + 3*img->width*j] = -1;
		//t_img[3*i + 2 + 3*img->width*j] = 0;

	}

	//printf("time to compute HSV %f [ms]\n", (ros::Time::now().toSec() - t0)*1000);

	for(std::vector<Blob*>::iterator it=blob_vector.begin(); it != blob_vector.end(); ++it)
	{
		delete (*it); // avoid leaks
	}
	
	blob_vector.clear();
	blob_count = 0;

	//printf("hip\n");

	//printf("time to clear vector %f [ms]\n", (ros::Time::now().toSec() - t0)*1000);

	
	// search for blobs
	for(unsigned int j = bottom_bondary + 1; j < upper_bondary-1; j++)
	for(unsigned int i = 1; i < img->width-1; i++)
	{
		//printf("0\n");
		if(t_img[3*i + 3*img->width*j] != 0)
		{ // this pixel belongs to a blob
			//printf("0b\n");

			if(t_img[3*(i-1) + 3*img->width*j] != 0 && i != 1 && j != bottom_bondary + 1)
			{ // pixel belongs to the blob left
				//printf("1\n");
				t_img[3*i + 1 + 3*img->width*j] = t_img[3*(i-1) + 1 + 3*img->width*j];

				blob_vector[t_img[3*i + 1 + 3*img->width*j]]->add_pixel(i, j);

				if(t_img[3*i + 3*img->width*(j-1)] != 0 &&
						t_img[3*(i-1) + 1 + 3*img->width*j] != t_img[3*i + 1 + 3*img->width*(j-1)])
				{ 
						int id_blob_1 = t_img[3*(i-1) + 1 + 3*img->width*j];
						int id_blob_2 = t_img[3*i + 1 + 3*img->width*(j-1)];

						
						while(blob_vector[id_blob_1]->is_connected_to() != -1)
						{
							id_blob_1 = blob_vector[id_blob_1]->is_connected_to();
		
						}

						
						while(blob_vector[id_blob_2]->is_connected_to() != -1)
						{
							id_blob_2 = blob_vector[id_blob_2]->is_connected_to();
							//printf("la ");
						}

						if(id_blob_1 != id_blob_2)
						{ // if the two blobs are not linked, connect the one with largest id to the other one
							if(id_blob_1 < id_blob_2)
								blob_vector[id_blob_2]->connect_to(id_blob_1);
							else
								blob_vector[id_blob_1]->connect_to(id_blob_2);
						}
				}
			}
			else if(t_img[3*i + 3*img->width*(j-1)] != 0  && j != bottom_bondary + 1)
			{ // pixel belongs to the blob above
				//printf("2\n");
				t_img[3*i + 1 + 3*img->width*j] = t_img[3*i + 1 + 3*img->width*(j-1)];
				//printf("2b\n");
				//printf("search for blob %d when vector size is %d \n", t_img[3*i + 1 + 3*img->width*j], blob_vector.size());
				blob_vector[t_img[3*i + 1 + 3*img->width*j]]->add_pixel(i, j);
				//printf("2c\n");
			}
			else
			{ // new blob !
				//printf("3\n");
				blob_vector.push_back(new Blob(i, j, blob_count));
				
				t_img[3*i + 1 + 3*img->width*j] = blob_count;
				//printf("new blob %d at %d, %d \n", blob_count, i, j);
				//printf("connection: %d %d \n", blob_count, blob_vector[blob_count]->is_connected_to());
				blob_count++;
				
			}
		}
	}


	//printf("hop\n");

	for(int i = blob_vector.size()-1; i > -1; i--)
	{
		//printf("we have a partial blob %d of size %d at %f, %f connected to %d \n", i,
		//			blob_vector[i]->size(), blob_vector[i]->x(), blob_vector[i]->y(), blob_vector[i]->is_connected_to());
		if(blob_vector[i]->is_connected_to() == -1)
		{ // the blob is connected to no other one thus is ready
			if(blob_vector[i]->size() >= blob_params.min_size)
			{ // only take blobs large enough
				//printf("blob %d of size %d at %f, %f \n", i,
					//blob_vector[i]->size(), blob_vector[i]->x(), blob_vector[i]->y());

				//blobs.push_back(new Blob(blob_vector[i]));
				geometry_msgs::Point position;
				position.x = blob_vector[i]->x();
				position.y = blob_vector[i]->y();
				position.z = blob_vector[i]->size();
				blobs.blob.push_back(position);

			}
		}else
		{ // if the blob is connected, add it to the other one
			blob_vector[blob_vector[i]->is_connected_to()]->assemble_Blob(blob_vector[i]);
		}
	}

	//printf("time to assemble blobs %f [ms]\n", (ros::Time::now().toSec() - t0)*1000);

	if(publish_debug)
	{
		sensor_msgs::Image debug_img;
		fillImage(debug_img, "rgb8", img->height, img->width, 3*img->width, t_img); 
		det_pub.publish(debug_img);
	}


}


