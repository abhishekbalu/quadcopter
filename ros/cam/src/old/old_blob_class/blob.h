#ifndef _COMMON_BLOB_
#define _COMMON_BLOB_

class Blob{
	private:
		int _id;
		int _similar_to;
		float _sum_x;
		float _sum_y;
		int _n_pix;
	public:
		Blob(int x, int y, int ID);
		void copy(Blob* blob);
		void connect_to(int id_other);
		int is_connected_to();
		void add_pixel(int x, int y);
		void assemble_Blob(Blob* blob);
		float x();
		float y();
		float sum_x();
		float sum_y();
		int n_pix();
		int id();
};

#endif