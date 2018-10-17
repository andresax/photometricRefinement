
#ifndef MY_KERNELS_H
#define MY_KERNELS_H

#include <vector>
#include <cmath>

//#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
//#include <CGAL/MP_Float.h>
//#include <CGAL/Gmpq.h>

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Fixed_precision_nt.h>
//#include <CGAL/Gmpq.h>

#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#define MVSTEREO_FACETS 0

// *** Kernel ***** - FAST but not exact
//typedef CGAL::Cartesian<double>		               Kernel;
//typedef CGAL::Cartesian<CGAL::MP_Float>		       KernelExact;
//typedef CGAL::Cartesian<CGAL::MP_Float>                Kernel;
//typedef CGAL::Cartesian<double>		               Kernel;
//typedef CGAL::Cartesian<CGAL::Fixed_precision_nt>      Kernel;
//typedef CGAL::Cartesian<double>		               Kernel;
typedef CGAL::Simple_cartesian<double>		               Kernel;

// *** ExactKernel ***** - EXACT
struct KernelExact : CGAL::Exact_predicates_exact_constructions_kernel {};
//struct KernelExact : CGAL::Exact_predicates_inexact_constructions_kernel {};
//struct Kernel : CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt {};
//typedef CGAL::Cartesian<double>		               KernelExact;

#define TO_POINT3_EXACT(A) KernelExact::Point_3(A[0],A[1],A[2])
#define FROM_POINT3_EXACT(A) Kernel::Point_3(CGAL::to_double(A[0]),CGAL::to_double(A[1]),CGAL::to_double(A[2]))	
#define TO_VECTOR3_EXACT(A) (KernelExact::Point_3(A.x(),A.y(),A.z())-CGAL::ORIGIN)
#define FROM_VECTOR3_EXACT(A) (Kernel::Point_3(CGAL::to_double(A.x()),CGAL::to_double(A.y()),CGAL::to_double(A.z()))-CGAL::ORIGIN)


typedef Kernel::Point_2                                Point_2;
typedef Kernel::Point_3                                Point;
typedef Kernel::Segment_3                              Segment_3;
typedef Kernel::Triangle_3                             Triangle_3;
typedef Kernel::Plane_3	                               Plane_3;
typedef Kernel::Vector_3                               Vector;

#define v_norm(A)  CGAL::sqrt(CGAL::to_double((A)*(A)))  // operator * is overloaded as dot product
#define t_area(A,B,C) (v_norm(CGAL::cross_product(B-A,C-A))/2)
#define v_normalized(A) (((A)*(A))==0?A:((A) / v_norm(A)))
#define v_angle(A,B) std::acos(std::max(-1.0,std::min(1.0,v_normalized(A)*v_normalized(B))))
#define p_angle(A,B,C) v_angle((A-B),(C-B))
#define edge_size(h) v_norm((h)->vertex()->point()- (h)->next()->next()->vertex()->point())
#define GAUSSIAN(SIGMA,X) exp(-(X)*(X)/(2*(SIGMA)*(SIGMA)))/(sqrt(2*PI)*(SIGMA))

const float PI = 3.14159265358979323846;   //!< Definition of the mathematical constant PI

double fix_sine(double sine);
double compute_angle_rad(const Vector& u, const Vector& v);
double compute_angle_rad(const Point& P, const Point& Q, const Point& R);
			   
//typedef CGAL::Cartesian<CGAL::Gmpq> 		       KernelExact;
enum FacetIDType { SUBCONSTRAINT_VERTEX, CONSTRAINT_VERTEX, TRIANGLE_EDGES };

/* A CDT facet with an additionnal info*/
template < class Gt, class Vb = CGAL::Triangulation_vertex_base_2<Gt> >
class My_vertex_base : public  Vb {
  typedef Vb                              Base;
public:
  typedef typename Vb::Vertex_handle      Vertex_handle;
  typedef typename Vb::Face_handle        Face_handle;
  typedef typename Vb::Point              Point;

  template < typename TDS2 >
  struct Rebind_TDS {
    typedef typename Vb::template Rebind_TDS<TDS2>::Other    Vb2;
    typedef My_vertex_base<Gt,Vb2>                           Other;
  };
  enum FacetIDType facet_id_type; // 0 = subconstraint vertex (default); 1 = original constraint; 2=triangle edges
  int facet_id; // facet_id_type=2 - the index of the vertex as obtained by iterating over halfedges
		// facet_id_type=1 - the index in the inter_facets vector
		// facet_id_type=0 - non relevant
  KernelExact::Point_3 point_3d;
public:
  void init() {facet_id_type=SUBCONSTRAINT_VERTEX; point_3d=KernelExact::Point_3(10,10,10);}
  My_vertex_base() : Base() {init();}
  My_vertex_base(const Point & p) : Base(p) {init();}
  My_vertex_base(const Point & p, Face_handle f) : Base(f,p) {init();}
  My_vertex_base(Face_handle f) : Base(f) {init();}
};

//struct CDT_K : CGAL::Exact_predicates_exact_constructions_kernel {};
//struct CDT_K : CGAL::Exact_predicates_inexact_constructions_kernel {};
//typedef CGAL::Cartesian<CGAL::Fixed_precision_nt>                              CDT_K;
//typedef CGAL::Cartesian<CGAL::Gmpq>                              CDT_K;

//struct CDT_K : CGAL::Exact_predicates_exact_constructions_kernel {};

typedef KernelExact                                                            CDT_K;
typedef My_vertex_base<CDT_K>                                                  CDT_Vb;
typedef CGAL::Constrained_triangulation_face_base_2<
	CDT_K,CGAL::Triangulation_face_base_with_info_2<bool,CDT_K> >          CDT_Fb;
typedef CGAL::Triangulation_data_structure_2<CDT_Vb,CDT_Fb>                    CDT_TDS;
typedef CGAL::Exact_predicates_tag                                             CDT_Itag;
//typedef CGAL::Exact_intersections_tag                                          CDT_Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<
              CDT_K,CDT_TDS, CDT_Itag>                                         CDT_base;
typedef CGAL::Constrained_triangulation_plus_2<CDT_base>       		       CDT;
typedef CDT::Point          					               CDT_Point;

#endif