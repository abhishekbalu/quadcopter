#include "cam/blob.h"

Blob::Blob(int x, int y, int ID){ //Constructor
	_sum_x = x;
	_sum_y = y;
	_id = ID;
	_similar_to = -1;
	_n_pix = 1;
}

void Blob::copy(Blob* blob){ //Copy a blob
	_sum_x = blob->sum_x();
	_sum_y = blob->sum_y();
	_id = blob->id();
	_similar_to = -1;
	_n_pix = n_pix();
}

void Blob::connect_to(int id_other){ //Set the id of the connected blob
	_similar_to = id_other;
}

int Blob::is_connected_to(){ //Get the id of the connected blob
	return _similar_to;
}

void Blob::add_pixel(int x, int y){
	_sum_x += x;
	_sum_y += y;
	_n_pix++;
}

void Blob::assemble_Blob(Blob* blob){
	_sum_x += blob->sum_x();
	_sum_y += blob->sum_y();
	_n_pix += blob->n_pix();
}

float Blob::x(){
	return _sum_x / _n_pix;
}

float Blob::y(){
	return _sum_y / _n_pix;
}

float Blob::sum_x(){
	return _sum_x;
}

float Blob::sum_y(){
	return _sum_y;
}
int Blob::n_pix(){ //Get the size
	return _n_pix; 
}

int Blob::id(){ 
	return _id; 
}