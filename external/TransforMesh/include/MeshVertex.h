//
// C++ Interface: Vertex
//
// Description: 
//
//
// Author: Andrei Zaharescu <zaharesc@octans>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef MESH_VERTEX_H
#define MESH_VERTEX_H
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Color.h>

#include "Kernels.h"
#include "MeshItems.h"

/* MeshVertex operation are added in the Mesh class, since we do not know
anything about those data structures */

template <class Refs, class T, class P, class Norm>
class MeshVertex : public CGAL::HalfedgeDS_vertex_base<Refs, T, P> {
//   typedef typename Refs::Vertex_handle Local_Vertex_handle;
//   typedef typename Refs::Halfedge_handle Local_Halfedge_handle;
//   typedef typename Refs::Face_handle Local_Facet_handle;

    Norm  norm;
    Norm  laplac; //Laplacian
    Norm  laplac_deriv; //Laplacian deriv
    float mean_curv; // Mean Curvature
    float gaussian_curv; // Gaussian curvature

public:
    float qual; // item to be copied from mean curvature, guassian curvature..
		// also used to store geodesic integral value at the vertex
    Norm qual_vect;	
	
    Norm delta;
    float delta_normalization;	
    Norm prev_delta;

	std::vector<float> values;
	
    float weight; // In the mesh modification procedure tells how close the vertex is to convergence.
		// In GeoSegmentation :: tells the internal geoIntegral value of the vertex within its segment

    float vh_dist; //distance from visual hull ; also used for quality_prev when performing convolutions
    Vector motion_vec; //used by totalDisplacementComputation	
    int id; // do not trust it if the mesh is being modified by splits and joins .. 
    float color[3];
    //std::vector<int> labelsCount;
   // std::vector<int> labelsCount;
    int labelsCount[100];

    #define NFLAGS 4
    bool flag[NFLAGS]; // 0 - used for euler operations; shared with OpenGL DrawGeometry mode 'S' - SilhouettePauTerm
		  // 1 - remeshing with threshold; border edge; MeshMatching when computing the detector in scale space
		  // 2 - used for visibility; MeshMatching when computing the detector in scale space
		  // 3 - used in MeshMathing


    Norm delta_tmp;	
    float tmp_float;

   typedef Norm Normal_3;
    void _init() {
		flag[0]=false;	flag[1]=false; flag[2]=true;
		delta = Norm(0,0,0);
		prev_delta = Norm(0,0,0);
		vh_dist = 0;
		weight=1; 
//		draw_offset=Vector(0,0,0);
		motion_vec=Vector(0,0,0);
		id=-1;
		for(int i=0;i<3;i++) color[i]=1.0;
    }

    void transferData (const MeshVertex & other) {
		if (this != &other) {
			for(int i=0;i<NFLAGS;i++) flag[i]=other.flag[i];
			delta = other.delta;
			prev_delta = other.prev_delta;
			vh_dist = other.vh_dist;
			
			weight=other.weight;
//			draw_offset=other.draw_offset;
			motion_vec=other.motion_vec;
			//labelsCount = other.labelsCount;
			id=other.id;
			for(int i=0;i<3;i++) color[i]=other.color[i];
			this->point()=other.point();
		}
    }
	
    MeshVertex() {_init();} // repeat mandatory constructors
    MeshVertex( const P& pt) : CGAL::HalfedgeDS_vertex_base<Refs, T, P>(pt){_init();}

/*	MeshVertex(const MeshVertex& v) {
		//the id should not be copied by the copy constructor
//		_init();
		transferData(v);
		id=-1;
//		std::cout << "copy constructor INVOKED" << std::endl;		
	}
 */
	
    MeshVertex & operator = (const MeshVertex & other) {
		int id_init=id;
		transferData(other);
		id=id_init;
/*		if (this != &other) {
			for(int i=0;i<NFLAGS;i++) flag[i]=other.flag[i];
			delta = other.delta;
			prev_delta = other.prev_delta;
			vh_dist = other.vh_dist;
		
			weight=other.weight;
//			draw_offset=other.draw_offset;
			motion_vec=other.motion_vec;
//			id=other.id;
			for(int i=0;i<3;i++) color[i]=other.color[i];
		}
        // by convention, always return *this
 */
	std::cout << "operator = INVOKED" << std::endl;
        return *this;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    Normal_3&       normal()       { return norm; }
    const Normal_3& normal() const { return norm; }

    Norm&       tmp_normal()       { return laplac_deriv; }
    const Norm& tmp_normal() const { return laplac_deriv; }

	
    Norm&       laplacian()       { return laplac; }
    const Norm& laplacian() const { return laplac; }

    Norm&       laplacian_deriv()       { return laplac_deriv; }
    const Norm& laplacian_deriv() const { return laplac_deriv; }

    float&       mean_curvature()       { return mean_curv; }
    const float& mean_curvature() const { return mean_curv; }

    float&       gaussian_curvature()       { return gaussian_curv; }
    const float& gaussian_curvature() const { return gaussian_curv; }

    float&       quality()       { return qual; }
    const float& quality() const { return qual; }

    float&       quality_imported()       { return tmp_float; }
    const float& quality_imported() const { return tmp_float; }

	
    float&       quality_prev()       { return vh_dist; }
    const float& quality_prev() const { return vh_dist; }

    bool&       border()       { return flag[1]; }
    const bool& border() const { return flag[1]; }
	
    float grayscale() {
	return (color[0] + color[1] + color[2])/3;
    }

    void move( const Vector& offset) { 
		if (border()) return;
		this->point() = this->point() + offset; 
		motion_vec = motion_vec + offset;
   }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    void set_color(float r, float g, float b){
		color[0] = r; color[1]=g; color[2]=b;
    }

    bool getVisibility() {
	return flag[2];
    }
    
   void setVisibility(bool status) {
	flag[2]=status;
   }
};
#endif
