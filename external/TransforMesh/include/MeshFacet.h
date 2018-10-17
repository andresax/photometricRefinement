#ifndef MESH_FACET_H
#define MESH_FACET_H

#include <CGAL/Polyhedron_3.h>
#include "Kernels.h"


template <class Kernel, class Refs, class T, class Norm>
class MeshFacet : public CGAL::HalfedgeDS_face_base<Refs, T> {
    Norm  norm;
public:
	
    typedef typename MeshFacet::Face_handle Facet_handle;
    typedef Norm Normal_3;
	//    typedef typename CGAL::Segment_3<Kernel> Segment_3;
	//    typedef typename Kernel::Triangle_3 Triangle_3;

//`	Plane_3 plane;

#if MVSTEREO_FACETS==1	
    bool has_color;
    float color[3];

    Norm delta;
    float delta_normalization;

	
    bool is_visible;
    float weight_priors;
    float weight;
#endif
	
    //segmentation related
    int id;

    //intersection related
    std::vector<MeshFacet * > inter_facets;
    std::vector<KernelExact::Segment_3 * > inter_segments;

    bool has_intersections;
	
    //self intersection removal
    CDT* cdt; //Constrainted Delaunay Triangulation
    char removal_status; // for self intersection removal: U -  unvisited; 'P' - partially valid; V - valid
						 // for connected components: U - unvisited; V - visited
    int label;
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // no constructors to repeat, since only default constructor mandatory
    MeshFacet() {
#if MVSTEREO_FACETS==1	
		has_color=false;
		is_visible=true; 
		delta=Norm(0,0,0);
		weight=1;
		weight_priors=1;
#endif
	has_intersections=false; 
	cdt=NULL;
	removal_status='U';

	}

	~MeshFacet() {
		if (cdt!=NULL) delete cdt;
	}
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    void addIntersection(Facet_handle f, KernelExact::Segment_3* s) {
		inter_facets.push_back(&(*f)); 
		inter_segments.push_back(s);
		has_intersections=true;
	}
    void setIntersectionStatus(bool stat) { has_intersections = stat;}
    bool hasIntersections() { return has_intersections;}
    void clearIntersections() { inter_facets.clear(); inter_segments.clear(); has_intersections=false;}
	
	
    Normal_3&       normal()       { return norm; }
    const Normal_3& normal() const { return norm; }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    void computeNormal() {
		if (this->halfedge()->facet()->facet_degree() <3) {
			std::cout << "facet degree = " << this->halfedge()->facet()->facet_degree() << std::endl;
			return;
		}
        typename MeshFacet::Halfedge_handle h = this->halfedge();
		Vector v1=h->next()->vertex()->point() - h->vertex()->point();
		Vector v2=h->prev()->vertex()->point() - h->vertex()->point();
		
		if ((v_norm(v1)==0) || (v_norm(v2)==0)) {
			std::cout << "\r WARNING: points coinciding"; //<<std::endl;
			this->normal() = Vector(0,0,1);
			return;
		}
		else {
			v1 = v_normalized(v1);
			v2 = v_normalized(v2);
			this->normal() = Vector(0,0,1);
		}
		
		typename MeshFacet::Normal_3 normal = CGAL::cross_product(v1,v2);
		if ((std::isnan(v_norm(normal))==false) && (v_norm(normal)!=0)) this->normal() = normal / v_norm(normal);
		else {
			std::cout << "\r WARNING: normal computation of a degenerate triangle -> v_norm(normal)=" << v_norm(normal); //<< std::endl;
//			std::cout << h->vertex()->point() << " " << h->next()->vertex()->point() << " " << h->prev()->vertex()->point() << std::endl;
			this->normal() = Vector(0,0,0);
		}
    }

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	Plane_3 supporting_plane(){
		return Plane_3(get_point(0),get_point(1),get_point(2));
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
    Triangle_3 triangle() {
		return Triangle_3(get_point(0),get_point(1),get_point(2));
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    KernelExact::Triangle_3 triangleExact() {
		return KernelExact::Triangle_3(TO_POINT3_EXACT(get_point(0)),
									   TO_POINT3_EXACT(get_point(1)),
									   TO_POINT3_EXACT(get_point(2)));
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    Point center() {
		return baricentric(0.333,0.333);
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    Point baricentric(float u1, float u2) {
/*		Vector p[3];
		Vector result;
		p[0] = this->halfedge()->vertex()->point() - CGAL::ORIGIN;
		p[1] = this->halfedge()->next()->vertex()->point() - CGAL::ORIGIN;
		p[2] = this->halfedge()->next()->next()->vertex()->point() - CGAL::ORIGIN;
		double sum = v_norm(p[0]) + v_norm(p[1]) + v_norm(p[2]);
		
		result = p[0]*(v_norm(p[0])/sum) + p[1]*(v_norm(p[1])/sum) + p[2]*(v_norm(p[2])/sum);
		return Point(result.x(),result.y(),result.z());
*/
		Point p[3];
		Point result;
		p[0] = get_point(0);
		p[1] = get_point(1);
		p[2] = get_point(2);
		return Point(p[0].x()*u1 + p[1].x()*u2 + p[2].x()*(1 - u1 -u2),
					 p[0].y()*u1 + p[1].y()*u2 + p[2].y()*(1 - u1 -u2),
					 p[0].z()*u1 + p[1].z()*u2 + p[2].z()*(1 - u1 -u2));
		
	}
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    double in_radius() {
		return 2 * area() / ( edge_size(this->halfedge()) + edge_size(this->halfedge()->next()) + edge_size(this->halfedge()->prev()));
    }
	
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
	void setVerticesFlag(int flag_no, bool status) { //starts @ 0
		CGAL_precondition(flag_no<=1);
		this->halfedge()->vertex()->flag[flag_no] = status;
		this->halfedge()->next()->vertex()->flag[flag_no] = status;
		this->halfedge()->next()->next()->vertex()->flag[flag_no] = status;
	}
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    Point point() {
		return baricentric(0.333,0.333);
    }
	
	inline Point get_point(int index) {
		if (index == 0) return this->halfedge()->vertex()->point();
		if (index == 1) return this->halfedge()->next()->vertex()->point();
		if (index == 2) return this->halfedge()->next()->next()->vertex()->point();
		std::cout << "invalid index" << std::endl;
	}
	
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    double edgeStatistics(int mode=1) { // 1 - min; 2-avg; 3-max
		CGAL_precondition((mode>=0) && (mode<=2));
		double e1 = v_norm(get_point(0)-get_point(1));
		double e2 = v_norm(get_point(0)-get_point(2));
		double e3 = v_norm(get_point(1)-get_point(2));
		if (mode==0) return std::min(e1,std::min(e2,e3));
		else if (mode==1) return (e1+e2+e3) / 3;
		else if (mode==2) return std::max(e1,std::max(e2,e3));	
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    double area() {
		return t_area(get_point(0),get_point(1),get_point(2));
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    bool operator == (const MeshFacet &w) const {
		return (triangle() == w.triangle());
    } 

#if MVSTEREO_FACETS==1	
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    void set_color(float r, float g, float b){
		color[0] = r; color[1]=g; color[2]=b; has_color=true;
    }
#endif
};


#endif
