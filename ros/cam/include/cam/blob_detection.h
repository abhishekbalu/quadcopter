#ifndef _COMMON_BLOB_DETECTION_
#define _COMMON_BLOB_DETECTION_
//Blob structure
typedef struct _blob{
	double x;
	double y;
	double size;
	int valid;
}blob;
//Main function in this module
int detect_blobs(unsigned char* buf, unsigned int step, int vl, int vh, int hl, int hh,
	int sl, int sh, int xi, int xf, int yi, int yf, int width, int height);
//Return binary image simage
unsigned char* get_binary_image();
//Return blob array
blob* get_blobs();
//Alloc memory for the binary image array, the rest is allocated at debug time
void init_binary_img(int width, int height);
int get_valid();

#endif