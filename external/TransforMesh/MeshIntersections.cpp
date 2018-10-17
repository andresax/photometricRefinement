/*
 *  MeshIntersections.cpp
 *  src
 *
 *  This file implements all the SelfIntersection removal code
 *
 *  Created by Andrei Zaharescu on 28/11/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */
// #include <CGAL/AABB_intersections.h>
#include <CGAL/AABB_tree.h> // must be inserted before kernel
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_polyhedron_triangle_primitive.h>

#include "Mesh.h"


//#include "Camera.h"
#include "MeshHelper.h"
//#include "ColorMap.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <cctype>


#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Object.h>
#include <CGAL/intersections.h>

#include <CGAL/squared_distance_2.h>
#include <CGAL/Timer.h>
#include <queue>
#include <map>

#include <CGAL/Convex_hull_traits_3.h>
#include <CGAL/convex_hull_3.h>

#define CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION 1
#include <CEP/intersection/Triangle_3_Triangle_3.h>

#define DEBUG_MESH 0
#define DEBUG_SELF_INTERSECTIONS 0 // 0 - none; 1 - self intersections only ; 2 - 1 + choosing startup-triangle
#define DEBUG_TESSELATE 0

#define STRONG_EDGE_COLLAPSE_CHECK 0
#define TRIANGLE_INTERSECTION_METHOD 1 // 1 - CEP ; 2 - graphics FAQ ; 3 - combined
#define MESH_BUILDER_WITH_HASHING 1
#define SELF_INTER_REMOVAL_POSITIVE_NORMALS_ONLY 1
#define SEED_METHOD 4 // 1 - visual hull; 2 - line; 3 - winding (slow); // 4 - winding (fast - using AABB trees)
#define COMPUTE_CURVATURE_STATS 0

#define TO_DEG(X_RAD) (X_RAD*180.0/PI)



void Mesh::testStuff ( float someParam )
{
	computeMeshStats();
	float desiredMaxEdge = someParam;
	//	cout << "edge_max= " << edge_max << " desired max edge= " << desiredMaxEdge << endl;
	//	ensureEdgeSizes(0, edge_max/2, true);//edge_avg
	//	ensureEdgeSizes(0, desiredMaxEdge);//edge_avg
	return;
	
	KernelExact::Point_3 p1 ( 1,0,0 );
	KernelExact::Point_3 p2 ( 0,1,0 );
	KernelExact::Point_3 p3 ( 0,0,1 );
	
	KernelExact::Point_3 p4 ( 0.1,0.2,0.3 );
	KernelExact::Point_3 p5 ( 1,0,1 );
	KernelExact::Point_3 p6 ( 0,1,1 );
	
	KernelExact::Triangle_3 a ( p1,p2,p3 ), b ( p4,p5,p6 );
	//	Triangle_3
	
	KernelExact::Point_3 s1_1 ( 0.625,0, 0.375 );
	KernelExact::Point_3 s1_2 ( 0, 0.625, 0.375 );
	KernelExact::Point_3 s2_1 ( 0.5,0.125, 0.375 );
	KernelExact::Point_3 s2_2 ( 0.125,0.5, 0.375 );
	
	cout << "collinear : " << CGAL::collinear ( s1_1,s1_2,s2_1 ) << endl;
	
	/*	CGAL::Object obj = CEP::intersection::intersection(a,b);
	 cout << "t1: " << a << " t2: " << b << endl;
	 if (const KernelExact::Triangle_3 * t = CGAL::object_cast<KernelExact::Triangle_3>(&obj)) {
	 cout << "intersection is a triangle: " << *t << endl;
	 }	
	 else
	 if (const KernelExact::Segment_3 * t =  CGAL::object_cast<KernelExact::Segment_3>(&obj)) {
	 cout << "intersection is a segment: " << *t << endl;
	 }
	 else
	 if (CGAL::object_cast<KernelExact::Point_3>(&obj)) {
	 cout << "intersection is a point: " << endl;
	 }
	 else {
	 cout << "no intersection" << endl;
	 }
	 cout << "---------------------------------------------------" << endl;
	 //	cout << "interesect:" << doTrianglesIntersect(a,b) << endl;
	 //	cout << "interesection:" << intersectTriangles(a,b) << endl;
	 
	 cout << "----FIX MESH-----------------------------------------------" << endl;
	 */
}




//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::doFacetsIntersect ( const Box *b, const Box *c, bool computeSegments )
{
	Halfedge_handle h = b->handle()->halfedge();
	// check for shared egde --> no intersection
	if ( h->opposite()->facet() == c->handle()
		|| h->next()->opposite()->facet() == c->handle()
		|| h->next()->next()->opposite()->facet() == c->handle() )
		return false;
	// check for shared vertex --> maybe intersection, maybe not
	Halfedge_handle g = c->handle()->halfedge();
	Halfedge_handle v;
	if ( h->vertex() == g->vertex() )
		v = g;
	if ( h->vertex() == g->next()->vertex() )
		v = g->next();
	if ( h->vertex() == g->next()->next()->vertex() )
		v = g->next()->next();
	if ( v == Halfedge_handle() )
	{
		h = h->next();
		if ( h->vertex() == g->vertex() )
			v = g;
		if ( h->vertex() == g->next()->vertex() )
			v = g->next();
		if ( h->vertex() == g->next()->next()->vertex() )
			v = g->next()->next();
		if ( v == Halfedge_handle() )
		{
			h = h->next();
			if ( h->vertex() == g->vertex() )
				v = g;
			if ( h->vertex() == g->next()->vertex() )
				v = g->next();
			if ( h->vertex() == g->next()->next()->vertex() )
				v = g->next()->next();
		}
	}
	if ( v != Halfedge_handle() )
	{
		// found shared vertex:
		CGAL_assertion ( h->vertex() == v->vertex() );
		// geomtric check if the opposite segments intersect the triangles
		Triangle_3 t1 ( h->vertex()->point(),
					   h->next()->vertex()->point(),
					   h->next()->next()->vertex()->point() );
		Triangle_3 t2 ( v->vertex()->point(),
					   v->next()->vertex()->point(),
					   v->next()->next()->vertex()->point() );
		Segment_3  s1 ( h->next()->vertex()->point(),
					   h->next()->next()->vertex()->point() );
		Segment_3  s2 ( v->next()->vertex()->point(),
					   v->next()->next()->vertex()->point() );
		if ( CGAL::do_intersect ( t1, s2 ) )
		{
			//cerr << "Triangles intersect (t1,s2):\n    T1: " << t1
			//     << "\n    T2 :" << t2 << endl;
			if ( !computeSegments ) return true;
			inter_segments.push_back ( intersectTrianglesExact ( t1, t2 ) );
			if ( inter_segments.back()->squared_length() ==0 )
			{
				delete inter_segments.back();
				inter_segments.pop_back();
				return false;
			}
			else
			{
				h->facet()->addIntersection ( v->facet(), inter_segments.back() );
				v->facet()->addIntersection ( h->facet(), inter_segments.back() );
				return true;
			}
		}
		else if ( CGAL::do_intersect ( t2, s1 ) )
		{
			//cerr << "Triangles intersect (t2,s1):\n    T1: " << t1
			//     << "\n    T2 :" << t2 << endl;
			if ( !computeSegments ) return true;
			inter_segments.push_back ( intersectTrianglesExact ( t1, t2 ) );
			if ( inter_segments.back()->squared_length() ==0 )
			{
				delete inter_segments.back();
				inter_segments.pop_back();
				return false;
			}
			else
			{
				h->facet()->addIntersection ( v->facet(), inter_segments.back() );
				v->facet()->addIntersection ( h->facet(), inter_segments.back() );
				return true;
			}
		}
		return false;
	}
	// check for geometric intersection
	Triangle_3 t1 ( h->vertex()->point(),
				   h->next()->vertex()->point(),
				   h->next()->next()->vertex()->point() );
	Triangle_3 t2 ( g->vertex()->point(),
				   g->next()->vertex()->point(),
				   g->next()->next()->vertex()->point() );
	if ( CGAL::do_intersect ( t1, t2 ) )
	{
		//cerr << "Triangles intersect:\n    T1: " << t1 << "\n    T2 :"
		//     << t2 << endl;
		if ( !computeSegments ) return true;
		inter_segments.push_back ( intersectTrianglesExact ( t1, t2 ) );
		if ( inter_segments.back()->squared_length() ==0 )
		{
			delete inter_segments.back();
			inter_segments.pop_back();
			return false;
		}
		else
		{
			h->facet()->addIntersection ( g->facet(), inter_segments.back() );
			g->facet()->addIntersection ( h->facet(), inter_segments.back() );
			
			return true;
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
struct Intersect_facets_wrapper
{
	void operator() ( const Box* b, const Box* c ) const
	{
		if ( m->doFacetsIntersect ( b,c,computeSegments ) && ( !computeSegments ) )
		{
			b->handle()->setIntersectionStatus ( true );
			c->handle()->setIntersectionStatus ( true );
		}
	}
public:
	Mesh * m;
	bool computeSegments;
	Intersect_facets_wrapper ( Mesh* tmpM, bool tmpComputeSegments ) {m = tmpM; computeSegments=tmpComputeSegments;}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
int Mesh::checkIntersections ( bool computeSegments )
{
	CGAL::Timer time_intersection;
	time_intersection.start();
	
	for ( std::vector<KernelExact::Segment_3*>::iterator j = inter_segments.begin(); j != inter_segments.end(); ++j )
		delete *j;
	inter_segments.clear();

	
	for ( Facet_iterator f = p.facets_begin(); f != p.facets_end(); ++f )
		f->clearIntersections();
#if DEBUG_SELF_INTERSECTIONS==1
	cout << "CHECKING INTERSECTIONS" << endl;
#endif
	std::vector<Box> boxes;
	boxes.reserve ( p.size_of_facets() );
	for ( Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i )
	{
		boxes.push_back (
						 Box ( i->halfedge()->vertex()->point().bbox()
							  + i->halfedge()->next()->vertex()->point().bbox()
							  + i->halfedge()->next()->next()->vertex()->point().bbox(),
							  i ) );
	}
	std::vector<const Box*> box_ptr;
	box_ptr.reserve ( p.size_of_facets() );
	for ( std::vector<Box>::iterator j = boxes.begin(); j != boxes.end(); ++j )
	{
		box_ptr.push_back ( &*j );
	}
	CGAL::box_self_intersection_d ( box_ptr.begin(), box_ptr.end(), Intersect_facets_wrapper ( this,computeSegments ) , std::ptrdiff_t ( 2000 ) );
	
	//iterate all the triangles
#if DEBUG_SELF_INTERSECTIONS==1
	if ( computeSegments ) cout << "COMPUTING CONSTRAINED TRIANGULATIONS" << endl;
#endif
	int intersections = 0;
	for ( Facet_iterator f = p.facets_begin(); f != p.facets_end(); ++f )
		if ( f->hasIntersections() )
		{
#if DEBUG_SELF_INTERSECTIONS==1
			f->set_color ( 0,0,1 );
#endif
			if ( computeSegments ) {
				if (f->cdt) delete f->cdt;
				f->cdt =  getTriangulation ( f );
			}
			/*		cout << " seg. " << std::distance(p.facets_begin(),f) << " has " << f->inter_facets.size() << " facet intersections; ";
			 cout << " sub_edges=" << std::distance(f->cdt->finite_edges_begin(),f->cdt->finite_edges_end()) ;
			 cout << " sub_triang=" << std::distance(f->cdt->finite_faces_begin(),f->cdt->finite_faces_end()) ;
			 cout << endl;
			 */		intersections++;
			
		}
		else
		{
#if DEBUG_SELF_INTERSECTIONS==1
			f->set_color ( 1,1,1 );
#endif
		}
	
#if DEBUG_MESH>1
	
	cout << " - taken : " << time_intersection.time() << " seconds" << endl;
#endif
	
	
	/*   for ( std::vector<Halfedge_handle>::iterator j = facet_intersections.begin(); j != facet_intersections.end(); ++j){
	 (*j)->facet()->set_color(0,0,1);
	 }
	 */
#if DEBUG_MESH>0	
	cout << "FOUND " << intersections << " INTERSECTIONS" << endl;
#endif
	return intersections;
}

KernelExact::Segment_3* Mesh::intersectTrianglesExact ( Triangle_3 a, Triangle_3 b )
{
	
	return intersectTriangles ( a,b );
	
	
	KernelExact::Point_3 p1[3],p2[3];
	for ( int i=0;i<3;i++ )
	{
		p1[i] = KernelExact::Point_3 ( a[i][0],a[i][1],a[i][2] );
		p2[i] = KernelExact::Point_3 ( b[i][0],b[i][1],b[i][2] );
	}
	KernelExact::Triangle_3 t1 ( p1[0],p1[1],p1[2] ), t2 ( p2[0],p2[1],p2[2] );
	
	
	CGAL::Object result = CEP::intersection::intersection ( t1,t2 );
	if ( result.is_empty() )
	{
#if DEBUG_SELF_INTERSECTIONS==1
		cout << "OBject is empty" << endl;
#endif
		return new KernelExact::Segment_3 ( KernelExact::Point_3 ( 0.0f,0.0f,0.0f ),KernelExact::Point_3 ( 0.0f,0.0f,0.0f ) );
	}
	if ( const KernelExact::Segment_3 *segment = CGAL::object_cast<KernelExact::Segment_3> ( &result ) )
	{
		return new KernelExact::Segment_3 ( *segment );
	}
	else
		if ( const KernelExact::Point_3 *segment = CGAL::object_cast<KernelExact::Point_3> ( &result ) )
		{
#if DEBUG_SELF_INTERSECTIONS==1
			cout << "Intersection is a POINT!!!!" <<  endl;
#endif
			return new KernelExact::Segment_3 ( KernelExact::Point_3 ( 0.0f,0.0f,0.0f ),KernelExact::Point_3 ( 0.0f,0.0f,0.0f ) );
		}
		else
		{
#if DEBUG_SELF_INTERSECTIONS==1
			cout << "Does not know what the intersection object is!!!!" <<  endl;
#endif
			return new KernelExact::Segment_3 ( KernelExact::Point_3 ( 0,0,0 ),KernelExact::Point_3 ( 0,0,0 ) );
		}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
KernelExact::Segment_3* Mesh::intersectTriangles ( Triangle_3 a, Triangle_3 b )
{
	//  return Segment_3(Point(0,0,0),Point(0,0,0));
#if TRIANGLE_INTERSECTION_METHOD==1
	KernelExact::Point_3 p1[3],p2[3];
	for ( int i=0;i<3;i++ )
	{
		p1[i] = KernelExact::Point_3 ( a[i][0],a[i][1],a[i][2] );
		p2[i] = KernelExact::Point_3 ( b[i][0],b[i][1],b[i][2] );
	}
	KernelExact::Triangle_3 t1 ( p1[0],p1[1],p1[2] ), t2 ( p2[0],p2[1],p2[2] );
	
	
	CGAL::Object result = CEP::intersection::intersection ( t1,t2 );
#if DEBUG_SELF_INTERSECTIONS==1
	if ( result.is_empty() ) cout << "OBject is empty" << endl;
#endif
	if ( const KernelExact::Segment_3 *segment = CGAL::object_cast<KernelExact::Segment_3> ( &result ) )
	{
		return new KernelExact::Segment_3 ( *segment );
	}
	else
	{
#if DEBUG_SELF_INTERSECTIONS==1
		cout << "COPLANAR triangles in interesection!!!!" <<  endl;
#endif
		return new KernelExact::Segment_3 ( KernelExact::Point_3 ( 0,0,0 ),KernelExact::Point_3 ( 0,0,0 ) );
	}
	
#elif TRIANGLE_INTERSECTION_METHOD==2
	
	int coplanar;
	float u[9], v[9], s[6];
	for ( int i=0;i<3;i++ )
		for ( int j=0;j<3;j++ )
		{
			u[3*i+j] = ( float ) a[i][j];
			v[3*i+j] = ( float ) b[i][j];
		}
	MeshHelper::tri_tri_intersect_with_isectline ( u,u+3,u+6,v,v+3,v+6,&coplanar,s,s+3 );
	
	if ( coplanar==1 )
	{
#if DEBUG_SELF_INTERSECTIONS==1
		cout << "COPLANAR ";
#endif
		return new KernelExact::Segment_3 ( KernelExact::Point_3 ( 0,0,0 ),KernelExact::Point_3 ( 0,0,0 ) );
	}
	else
		return new KernelExact::Segment_3 ( KernelExact::Point_3 ( s[0],s[1],s[2] ),KernelExact::Point_3 ( s[3],s[4],s[5] ) );
#else
	int coplanar;
	float u[9], v[9], s[6];
	for ( int i=0;i<3;i++ )
		for ( int j=0;j<3;j++ )
		{
			u[3*i+j] = ( float ) a[i][j];
			v[3*i+j] = ( float ) b[i][j];
		}
	MeshHelper::tri_tri_intersect_with_isectline ( u,u+3,u+6,v,v+3,v+6,&coplanar,s,s+3 );
	if ( coplanar==1 )
	{ // the slow one
		
		KernelExact::Point_3 p1[3],p2[3];
		for ( int i=0;i<3;i++ )
		{
			p1[i] = KernelExact::Point_3 ( a[i][0],a[i][1],a[i][2] );
			p2[i] = KernelExact::Point_3 ( b[i][0],b[i][1],b[i][2] );
		}
		KernelExact::Triangle_3 t1 ( p1[0],p1[1],p1[2] ), t2 ( p2[0],p2[1],p2[2] );
		
		CGAL::Object result = CEP::intersection::intersection ( t1,t2 );
#if DEBUG_SELF_INTERSECTIONS==1
		if ( result.is_empty() ) cout << "OBject is empty" << endl;
#endif
		if ( const KernelExact::Segment_3 *segment = CGAL::object_cast<KernelExact::Segment_3> ( &result ) )
		{
			return new KernelExact::Segment_3 ( *segment );
		}
		else
		{
#if DEBUG_SELF_INTERSECTIONS==1
			cout << "COPLANAR triangles in interesection!!!!" <<  endl;
#endif
			return new KernelExact::Segment_3 ( KernelExact::Point_3 ( 0,0,0 ),KernelExact::Point_3 ( 0,0,0 ) );
		}
		
	}
	else
		return new KernelExact::Segment_3 ( KernelExact::Point_3 ( s[0],s[1],s[2] ),KernelExact::Point_3 ( s[3],s[4],s[5] ) );
	
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Facet*> Mesh::getNeighbours ( Facet *f )
{
	vector<Facet*> result;
	result.push_back ( f );
	//	cout << "Triangle " << f->triangle() << endl;
	
	HF_circulator hc = f->facet_begin();
	do
	{
		HV_circulator hv = hc->vertex()->vertex_begin();
		do
		{
			if ( find ( result.begin(),result.end(),& ( *hv->facet() ) ) ==result.end() )
			{
				result.push_back ( & ( *hv->facet() ) );
				//				cout << "- neigh " << hv->facet()->triangle() << endl;
				//				hv->facet()->set_color(0.0,1.0,0.0);
			}
		}
		while ( ++hv != hc->vertex()->vertex_begin() );
	}
	while ( ++hc != f->facet_begin() );
	
	//	f->set_color(1,0,0);
	return result;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////
// assumes that all the intersections are added in inter_facets
CDT* Mesh::getTriangulation ( Vertex::Facet_handle f )
{
	
	//Facet_handle f = h->facet();
	CDT* cdt = new CDT();
	
	KernelExact::Plane_3 plane_eq ( TO_POINT3_EXACT ( f->triangle() [0] ),TO_POINT3_EXACT ( f->triangle() [1] ),TO_POINT3_EXACT ( f->triangle() [2] ) );
	//	KernelExact::Plane_3 plane_eq= f->triangle().supporting_plane();
	
	HF_circulator h = f->facet_begin();
	CDT::Point t_p1 = plane_eq.to_2d ( TO_POINT3_EXACT ( h->vertex()->point() ) );
	CDT::Point t_p2 = plane_eq.to_2d ( TO_POINT3_EXACT ( h->next()->vertex()->point() ) );
	CDT::Point t_p3 = plane_eq.to_2d ( TO_POINT3_EXACT ( h->next()->next()->vertex()->point() ) );
	
	CDT::Vertex_handle v1 = cdt->insert ( t_p1 );
	CDT::Vertex_handle v2 = cdt->insert ( t_p2 );
	CDT::Vertex_handle v3 = cdt->insert ( t_p3 );
	// convention to say that these vertices are the original edges of the triangle
	v1->facet_id_type=TRIANGLE_EDGES; v1->facet_id=0; v1->point_3d = TO_POINT3_EXACT ( h->vertex()->point() );
	v2->facet_id_type=TRIANGLE_EDGES; v2->facet_id=1; v2->point_3d = TO_POINT3_EXACT ( h->next()->vertex()->point() );
	v3->facet_id_type=TRIANGLE_EDGES; v3->facet_id=2; v3->point_3d = TO_POINT3_EXACT ( h->next()->next()->vertex()->point() );
	cdt->insert_constraint ( v1,v2 );
	cdt->insert_constraint ( v2,v3 );
	cdt->insert_constraint ( v3,v1 );
	
	
	for ( std::vector<KernelExact::Segment_3 *>::iterator j = f->inter_segments.begin(); j != f->inter_segments.end(); ++j )
	{
		KernelExact::Segment_3& tmpSeg = **j;
		
		CDT::Point p1 = plane_eq.to_2d ( tmpSeg.target() );
		CDT::Point p2 = plane_eq.to_2d ( tmpSeg.source() );
		
		CDT::Vertex_handle v1 = cdt->insert ( p1 );
		CDT::Vertex_handle v2 = cdt->insert ( p2 );
		
		// convention to say that these vertices are parts of input constraints
		v1->facet_id_type=CONSTRAINT_VERTEX;
		v2->facet_id_type=CONSTRAINT_VERTEX;
		int j_dist= std::distance ( f->inter_segments.begin(),j );
		v1->facet_id = j_dist; v1->point_3d = tmpSeg.target();
		v2->facet_id = j_dist; v2->point_3d = tmpSeg.source();
		cdt->insert_constraint ( v1,v2 );
	}
	
	
	//	if (!cdt->is_valid())
	
	int count=0;
	for ( CDT::Finite_edges_iterator eit = cdt->finite_edges_begin(); eit != cdt->finite_edges_end(); ++eit )
	{
		if ( cdt->is_constrained ( *eit ) ) ++count;
		
		CDT::Edge pp = *eit;
		CDT::Face_handle f = pp.first;
		int fedge = pp.second;
		
		// if the vertices are sub-contraints... we need to set the point_3d
		if ( f->vertex ( cdt->cw ( fedge ) )->facet_id_type==SUBCONSTRAINT_VERTEX )
			f->vertex ( cdt->cw ( fedge ) )->point_3d = plane_eq.to_3d ( f->vertex ( cdt->cw ( fedge ) )->point() );
		
		if ( f->vertex ( cdt->ccw ( fedge ) )->facet_id_type==SUBCONSTRAINT_VERTEX )
			f->vertex ( cdt->ccw ( fedge ) )->point_3d = plane_eq.to_3d ( f->vertex ( cdt->ccw ( fedge ) )->point() );
		
//		triang_segments.push_back ( Segment_3 ( FROM_POINT3_EXACT ( f->vertex ( cdt->cw ( fedge ) )->point_3d ),FROM_POINT3_EXACT ( f->vertex ( cdt->ccw ( fedge ) )->point_3d ) ) );
		
	}
	
	for ( CDT::Finite_faces_iterator eif = cdt->finite_faces_begin(); eif != cdt->finite_faces_end(); ++eif )
		eif->info() = false; // mark all subtriangles as not visited - used in self-intersection removal
	
	
	//	std::cout << "The number of resulting constrained edges is  ";
	//	std::cout <<  count << std::endl;
	
	return cdt;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// A modifier creating a triangle with the incremental builder.
template <class HDS, class K>
class Build_New_Mesh : public CGAL::Modifier_base<HDS>
{
private:
	vector<typename K::Triangle_3> new_triangles;
	vector<Facet_handle> new_triangles_orig_facet;
	
	vector<Vertex_handle> vertex_orig_vertex_map;
	std::map <typename K::Point_3, int> vertex_map;
	
	
	//additional structures required in order to be able to perform vertex splits
	vector< vector<int> > vertex_triangle_map; // vertex triangle iterator
	vector<typename K::Point_3> vertices;
	vector<bool>	vertices_new_flag;
	vector<bool>	triangle_status;
	vector< vector<int> > triangle_vertex_map; //triangle vertex iterator
	
	bool error;
	bool copy_original_facet_data;
	
	// assumes that the additional data structures have been initialized
	bool _findTriangleWithinVertex ( int vertex_id, int & triangle_id, int & other_vertex_id_1, int & other_vertex_id_2 )
	{
		triangle_id=-1;
		for ( int t=0;t<vertex_triangle_map[vertex_id].size();t++ )
			if ( triangle_status[vertex_triangle_map[vertex_id][t]] == false ) {
				triangle_id = vertex_triangle_map[vertex_id][t];
				triangle_status[triangle_id] = true;
				for ( int i=0;i<3;i++ )
					if ( triangle_vertex_map[triangle_id][i]==vertex_id ) {
						other_vertex_id_1 = triangle_vertex_map[triangle_id][ ( i+1 ) %3];
						other_vertex_id_2 = triangle_vertex_map[triangle_id][ ( i+2 ) %3];						
						return true;
					}
			}
		if ( triangle_id!=-1 )
		{
			cout << "_findTriangleWithinVertex failed !" << endl;
		}
		return false;
		
	}
	bool _triangleContainsVertex ( int triangle_id, int vertex_id )
	{
		for ( int v=0;v<3;v++ )
			if ( triangle_vertex_map[triangle_id][v] == vertex_id ) return true;
		return false;
	}
	
	bool _chooseNextTriangle ( int vertex_id, int triangle_id, int other_vertex_id,  int & new_triangle_id, int & new_other_vertex_id )
	{
		new_triangle_id=-1;
		for ( int t=0;t<vertex_triangle_map[other_vertex_id].size();t++ )
			if ( ( triangle_status[vertex_triangle_map[other_vertex_id][t]] == false ) && _triangleContainsVertex ( vertex_triangle_map[other_vertex_id][t],vertex_id ) )
			{
				new_triangle_id = vertex_triangle_map[other_vertex_id][t];
				triangle_status[new_triangle_id] = true;
				for ( int i=0;i<3;i++ )
					if ( ( triangle_vertex_map[new_triangle_id][i] !=vertex_id ) && ( triangle_vertex_map[new_triangle_id][i] != other_vertex_id ) )
					{
						new_other_vertex_id = triangle_vertex_map[new_triangle_id][i];
						return true;
					}
			}
		if ( new_triangle_id!=-1 )
		{
			cout << "_chooseNextTriangle failed !" << endl;
		}
		return false;
	}
	
public:
	////////////////////////////////////////////////////////////////////////////////////
	Build_New_Mesh()
	{
		error = false;
		copy_original_facet_data = true;
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void addTriangle ( typename K::Triangle_3 tmpT )
	{
		new_triangles.push_back ( tmpT );
		copy_original_facet_data = false;
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void addTriangle ( typename K::Triangle_3 tmpT, Triangle_Job &job )
	{
		
		if ( job.updated_norm_factor == 1 )
		{
			new_triangles.push_back ( tmpT );
			
			for ( int i=0;i<3;i++ )
			{
				if ( vertex_map.find ( tmpT[i] ) ==vertex_map.end() )
				{
					//							vertex_map[tmpT[i]] = vertex_id++;
					//							vertex_orig_vertex_map[tmpT[i]] = job.facet->
				}
			}
			
		}
		else
		{
			//		cout << "Added inverted element" << endl;
			//new_triangles.push_back(tmpT);
			new_triangles.push_back ( typename K::Triangle_3 ( tmpT[2],tmpT[1],tmpT[0] ) );
			//		cout << "Added triangle " << new_triangles.size()-1 << " with normal_factor=" << job.updated_norm_factor << endl;
			//		new_triangles.push_back(Triangle_3(tmpT[2],tmpT[1],tmpT[0]));
			
		}
		new_triangles_orig_facet.push_back ( job.facet );
		
		//	cout << "Now " << new_triangles.size() << " ==> " << tmpT << endl;
		
	}
	
	void operator() ( HDS& hds )
	{
#if DEBUG_MESH>0		
		cout << "STITCHING TOGETHER THE NEW MESH "; flush ( cout );
#endif		
#if MESH_BUILDER_WITH_HASHING==1
		
		vector<int> tmpVector;
		
		
		// build the map with all the vertices
		int vertex_id=0;
		
		for ( typename vector<typename K::Triangle_3>::iterator it=new_triangles.begin(); it!=new_triangles.end();it++ )
		{
			for ( int i=0;i<3;i++ )
			{
				if ( vertex_map.find ( it->vertex ( i ) ) ==vertex_map.end() )
				{
					vertex_map[it->vertex ( i ) ] = vertex_id++;
				}
			}
		}
		
		// Need to create additional data structures in order to
		//   process each vertex and split the doberman ears like structures,
		//   where a vertex connects two separated components.
		
		triangle_status.reserve ( new_triangles.size() );
		triangle_vertex_map.reserve ( new_triangles.size() );
		vertex_triangle_map.reserve ( vertex_map.size() );
		vertices.reserve ( vertex_id );
		vertices_new_flag.reserve ( vertex_id );
		
		//create structures
		for ( typename vector<typename K::Triangle_3>::iterator it=new_triangles.begin(); it!=new_triangles.end();it++ ) {
			triangle_vertex_map.push_back ( tmpVector );
		}
		
		for ( typename std::map <typename K::Point_3, int>::iterator v_it=vertex_map.begin();v_it!=vertex_map.end();v_it++ )
		{
			vertices.push_back ( new_triangles[0].vertex ( 0 ) );
			vertices_new_flag.push_back ( false );
			vertex_triangle_map.push_back ( tmpVector );
		}
		
		//populate structures
		for ( typename std::map <typename K::Point_3, int>::iterator v_it=vertex_map.begin();v_it!=vertex_map.end();v_it++ )
		{
			vertices[v_it->second] = v_it->first;
		}
		for ( int t=0;t < new_triangles.size(); t++ )
		{
			triangle_status.push_back ( false );
			for ( int i=0;i<3;i++ )
			{
				triangle_vertex_map[t].push_back ( vertex_map[new_triangles[t].vertex ( i ) ] );
				vertex_triangle_map[vertex_map[new_triangles[t].vertex ( i ) ]].push_back ( t );
			}
		}
		int old_vertices_size = vertices.size();
		int singular_vertex=0;
		for ( int v=0;v<old_vertices_size;v++ ) {
			// set all triangles as false
			for ( int t=0;t<vertex_triangle_map[v].size();t++ )
				triangle_status[vertex_triangle_map[v][t]] = false;
			int triangle_count = vertex_triangle_map[v].size();
			
			int cur_triangle_id, next_triangle_id, next_triangle_id_orig;
			int cur_other_vertex_id, next_other_vertex_id, next_other_vertex_id_1,next_other_vertex_id_2;
			bool first_time=true;
			queue<int> trianglesForNewVertex;
			int new_v=-1;
			while ( _findTriangleWithinVertex ( v,next_triangle_id_orig,next_other_vertex_id_1,next_other_vertex_id_2 ) )
			{
				if ( !first_time )
				{
					singular_vertex++;
					vertices.push_back ( vertices[v] );
					vertices_new_flag[v] = true;
					vertices_new_flag.push_back ( true ); // it is a new vertex - move it a bit
					new_v = vertices.size()-1;
					vertex_triangle_map.push_back ( tmpVector );
				}
				//			cout << "seeder vertex: " << v;
				
				//it makes sense to traverse in both directions for open surfaces!
				// 1st direction
				next_other_vertex_id = next_other_vertex_id_1;
				next_triangle_id = next_triangle_id_orig;
				do {
					cur_triangle_id = next_triangle_id;
					cur_other_vertex_id = next_other_vertex_id;
					if ( new_v!=-1 ) {
						trianglesForNewVertex.push ( cur_triangle_id );
					}
				} while ( _chooseNextTriangle ( v,cur_triangle_id,cur_other_vertex_id,next_triangle_id,next_other_vertex_id ) );
				// 2nd direction
				next_other_vertex_id = next_other_vertex_id_2;
				next_triangle_id = next_triangle_id_orig;
				do {
					cur_triangle_id = next_triangle_id;
					cur_other_vertex_id = next_other_vertex_id;
					if ( new_v!=-1 ) {
						trianglesForNewVertex.push ( cur_triangle_id );
					}
				} while ( _chooseNextTriangle ( v,cur_triangle_id,cur_other_vertex_id,next_triangle_id,next_other_vertex_id ) );
				
				
				// we need to move all the triangles to its new papa vertex
				while ( !trianglesForNewVertex.empty() )
				{
					cur_triangle_id = trianglesForNewVertex.front(); trianglesForNewVertex.pop();
					vertex_triangle_map[new_v].push_back ( cur_triangle_id );
					
					vector<int> &localVector = vertex_triangle_map[v];
					
					localVector.erase ( remove_if ( localVector.begin(), localVector.end(), bind2nd ( equal_to<int>(), cur_triangle_id ) ), localVector.end() );
					
					for ( int ii=0;ii<3;ii++ )
						if ( triangle_vertex_map[cur_triangle_id][ii]==v )
							triangle_vertex_map[cur_triangle_id][ii] = new_v;
				}
				first_time = false;
			}
		}
#if DEBUG_MESH>0				
		cout << "(found " << singular_vertex << " singular vertices)." << endl;
#endif		
		// Postcondition: `hds' is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B ( hds, true );
		B.begin_surface ( vertex_map.size(), new_triangles.size(), 6 );
		
		
		for ( int i=0;i<vertices.size();i++ )
		{
			Vertex_handle v = B.add_vertex ( FROM_POINT3_EXACT ( vertices[i] ) );
			if ( vertices_new_flag[i] ) v->flag[0]=true;
		}
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;
		for ( int t=0;t<triangle_vertex_map.size();t++ )
		{
			// test if adding this facet violates the manifold constraints
			std::vector< std::size_t> newFacetIdx;
			for ( int i=0;i<3;i++ )
				newFacetIdx.push_back( triangle_vertex_map[t][i] );
			
			Halfedge_handle h=NULL;
			if (B.test_facet(newFacetIdx.begin(),newFacetIdx.end()))
				h = B.add_facet(newFacetIdx.begin(),newFacetIdx.end());
			else
				cout << "\r WARNING: facet " << t << " violates the manifold constraint (ignoring it)" << endl;
			
/*			
			B.begin_facet();
			// copy information from parent mesh
			for ( int i=0;i<3;i++ )
				B.add_vertex_to_facet ( triangle_vertex_map[t][i] );
			Halfedge_handle h = B.end_facet();
			
*/			
			if ( (h!=NULL) && copy_original_facet_data )
			{
#if MVSTEREO_FACETS==1
				h->facet()->weight_priors = new_triangles_orig_facet[t]->weight_priors;
				h->facet()->weight = new_triangles_orig_facet[t]->weight;
#endif				
				Halfedge_handle orig_h = new_triangles_orig_facet[t]->facet_begin();
				h->facet()->removal_status=new_triangles_orig_facet[t]->removal_status;
				for ( int i=0;i<3;i++ ) {
					if (vertices[triangle_vertex_map[t][i]]==TO_POINT3_EXACT(new_triangles_orig_facet[t]->get_point(i))) {
						( h->vertex() )->transferData(*(orig_h->vertex()));
					}
					else {
						h->vertex()->weight = 0.5;
						//							cout << vertices[triangle_vertex_map[t][i]] << "!=" << new_triangles_orig_facet[t]->get_point(i) <<endl;
					}
					h = h->next();
					orig_h = orig_h->next();
				}
					
			}
		}
		
		if ( B.check_unconnected_vertices() )
		{
			cout << "There are unconnected vertices!" << endl;
			B.remove_unconnected_vertices();
		}
		
		if ( B.error() )
		{
			cout << "Error encountered while building the surface with hashin. Undoing operation. Rebuilding without hashing." << endl;
			B.rollback();
			B.begin_surface ( new_triangles.size() *3, new_triangles.size(), 6 );
			for ( typename vector<typename K::Triangle_3>::iterator it=new_triangles.begin(); it!=new_triangles.end();it++ )
			{
				
				B.add_vertex ( FROM_POINT3_EXACT ( it->vertex ( 0 ) ) );
				B.add_vertex ( FROM_POINT3_EXACT ( it->vertex ( 1 ) ) );
				B.add_vertex ( FROM_POINT3_EXACT ( it->vertex ( 2 ) ) );
				B.begin_facet();
				B.add_vertex_to_facet ( hds.size_of_vertices()-3 );
				B.add_vertex_to_facet ( hds.size_of_vertices()-2 );
				B.add_vertex_to_facet ( hds.size_of_vertices()-1 );
				B.end_facet();
			}
			B.end_surface();
		}
		else
			B.end_surface();
	}
#else
	cout << "Mesh Builder WITHOUT Hashing invoked" << endl;
	// Postcondition: `hds' is a valid polyhedral surface.
	CGAL::Polyhedron_incremental_builder_3<HDS> B ( hds, true );
	B.begin_surface ( new_triangles.size() *3, new_triangles.size() );
	
	typedef typename HDS::Vertex   Vertex;
	typedef typename Vertex::Point Point;
	for ( vector<Triangle_3>::iterator it=new_triangles.begin(); it!=new_triangles.end();it++ )
	{
		
		B.add_vertex ( it->vertex ( 0 ) );
		B.add_vertex ( it->vertex ( 1 ) );
		B.add_vertex ( it->vertex ( 2 ) );
		B.begin_facet();
		B.add_vertex_to_facet ( hds.size_of_vertices()-3 );
		B.add_vertex_to_facet ( hds.size_of_vertices()-2 );
		B.add_vertex_to_facet ( hds.size_of_vertices()-1 );
		B.end_facet();
	}
	B.end_surface();
	}
#endif
	}; //Build_New_Mesh
	
	
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// helper  function for self intersection
	// --------------------------------------
	Facet_handle Mesh::_getSeedTriangle() {

// see the SEED_METHOD legent @ the top		
#if SEED_METHOD==4
		
		typedef Kernel::Point_3 Point;
		typedef Kernel::Plane_3 Plane;
		typedef Kernel::Vector_3 Vector;
		typedef Kernel::Segment_3 Segment;
		typedef CGAL::AABB_polyhedron_triangle_primitive<Kernel,Polyhedron> AABB_Primitive;
		typedef CGAL::AABB_traits<Kernel, AABB_Primitive> AABB_Traits;
		typedef CGAL::AABB_tree<AABB_Traits> AABB_Tree;
		typedef AABB_Tree::Object_and_primitive_id Object_and_primitive_id;
		typedef AABB_Tree::Primitive_id Primitive_id;	
		
//		AABB_Tree tree(p.facets_begin(),p.facets_end());
		
		if (_AABB_tree==NULL) //allocate the tree first time
			_AABB_tree = new AABB_Tree(p.facets_begin(),p.facets_end());		
#endif		
		
		
		//TODO: should also check that it's on the visual hull
		for ( Facet_iterator f=p.facets_begin();f!=p.facets_end(); ++f )
			if ( ( !f->hasIntersections() ) && ( f->removal_status=='U' ) ) {
				
#if SEED_METHOD==1			
				Plane_3 the_plane ( f->facet_begin()->vertex()->point() +f->normal() *edge_avg/2,f->normal() );
				//			Kernel::Line_3 line_1(vi->point(), vi->point() + vi->normal());
				
				bool visual_hull_triangle = true;
				for ( Vertex_iterator v=p.vertices_begin();v!=p.vertices_end(); ++v )
					if ( the_plane.has_on_positive_side ( v->point() ) )
					{
						visual_hull_triangle = false;
						break;
					}
				if ( visual_hull_triangle )
					return f;
#elif SEED_METHOD==2
				Kernel::Point_3 pp,p1,p2;
				Kernel::Vector_3 n;
				pp=(f->center());
				n=(f->normal());
				
				p1=pp;
				p2=pp + n*edge_avg*100;
				Kernel::Ray_3 ray(p1,p2);
				bool seed_triangle = true;
				for ( Facet_iterator ff=p.facets_begin();ff!=p.facets_end(); ++ff ) {
					if (do_intersect(ff->triangle(),ray)) {
						if (ff->center()==pp) continue;
						seed_triangle=false;
						//cout << "tr:" << f << " intersects" << ff << endl;
						break;
					}
				}
				if (seed_triangle)
					return f;
#elif SEED_METHOD==3
				Kernel::Point_3 pp,p1,p2;
				Kernel::Vector_3 n;
				pp=(f->center());
				n=(f->normal());
				
				p1=pp; //+ n*edge_avg*0.01;
				p2=pp + n*edge_avg*100;
				Kernel::Ray_3 ray(p1,p2);
				int inter_1=0;
				int inter_2=0; // to counter-act for the source triangle
				bool seed_triangle = true;
				Vector ray_normal = v_normalized(p2-p1);
				for ( Facet_iterator ff=p.facets_begin();ff!=p.facets_end(); ++ff ) {
					if ((f!=ff) && do_intersect(ff->triangle(),ray)) {
						float sign_dot_prod= ff->normal()*ray_normal;
						if (sign_dot_prod > 0) 
							inter_1++;
						else
							inter_2++;
					}
				}
				if (inter_1==inter_2)
					return f;
#elif SEED_METHOD==4
				
				Kernel::Point_3 pp,p1,p2;
				Kernel::Vector_3 n;
				pp=(f->center());
				n=(f->normal());
				
				p1=pp; //+ n*edge_avg*0.01;
				p2=pp + n*edge_avg*1000000;
				Kernel::Ray_3 ray_query(p1,p2);
				//Kernel::Segment_3 segment_query(p1,p2);
				int inter_1=0;
				int inter_2=0; // to counter-act for the source triangle
				bool seed_triangle = true;
				Vector ray_normal = v_normalized(p2-p1);
				
				
				std::list<Object_and_primitive_id> intersections;
				((AABB_Tree*)_AABB_tree)->all_intersections(ray_query, std::back_inserter(intersections));
				//boost::optional<Object_and_primitive_id> intersection = tree.any_intersection(segment_query);
				
				for(std::list<Object_and_primitive_id>::iterator i=intersections.begin(); i!=intersections.end();i++) {
					Object_and_primitive_id op = *i;
					CGAL::Object object = op.first;
					
					Point point;
					if (CGAL::assign(point,object)) { //if intersection is a point
						Facet_handle other_face = op.second;
						if (f!=other_face) {
							float sign_dot_prod= other_face->normal()*ray_normal;
							if (sign_dot_prod > 0) 
								inter_1++;
							else
								inter_2++;
						}
					}
					
				}
				
				if (inter_1==inter_2)
					return f;
				
#endif		
			}
		
		return NULL;
		
	}
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// helper  function for self intersection
	// --------------------------------------
	// We need to iterate the Constrainted Delaunay triangulation and find
	// the triangle which contains the entrance edge removal_entrance_segment.
	// We need to use the removal_updated_norm which at this time represents the norm of
	// the entrance triangle.
	
	
	bool Mesh::_findStartupCDTriangle ( Triangle_Job &job )
	{
		Facet_handle f = job.facet;
		CDT::Face_handle startup_triangle=NULL;
		bool found_sol=false;
		
		// some preparations in order to choose the triangle on the good side of the edge
		//	if (job.updated_norm_factor==-1) job.entrance_segment = job.entrance_segment.opposite();
		KernelExact::Point_3 p_ref = job.entrance_segment.target();
		
		KernelExact::Point_3 p_prev_opposite = job.entrance_opposite_point;
		KernelExact::Point_3 p_prev_with_norm = p_ref + job.entrance_norm;
		KernelExact::Line_3 l_prev ( p_ref,p_prev_with_norm );
		KernelExact::Plane_3 pl_prev = l_prev.perpendicular_plane ( p_ref );
		CGAL::Oriented_side good_side_prev = pl_prev.oriented_side ( p_prev_with_norm );
		
		
		//	cout << "no of edges is: " << std::distance(f->cdt->finite_edges_begin(),f->cdt->finite_edges_end()) << endl;
		for ( CDT::Finite_edges_iterator eit = f->cdt->finite_edges_begin(); eit != f->cdt->finite_edges_end(); ++eit )
		{
			if ( f->cdt->is_constrained ( *eit ) )
			{
				CDT::Edge pp = *eit;
				CDT::Face_handle faces[2];
				faces[0] = pp.first; // the current face
				int tmp_v_id = pp.second; // the opposite vertex
				
				KernelExact::Point_3 pt1 = faces[0]->vertex ( f->cdt->ccw ( tmp_v_id ) )->point_3d;
				KernelExact::Point_3 pt2 = faces[0]->vertex ( f->cdt->cw ( tmp_v_id ) )->point_3d;
				KernelExact::Point_3 pt3 = faces[0]->vertex ( tmp_v_id )->point_3d; // opposite side
				KernelExact::Segment_3 seg ( pt1,pt2 );
				
				// if it is just one segment (just entering a partially valid from a valid triangle)
				if ( ( seg == job.entrance_segment ) || ( seg.opposite() == job.entrance_segment ) )
				{
					
					faces[1] = faces[0]->neighbor ( tmp_v_id );
#if DEBUG_SELF_INTERSECTIONS>1
					cout << "- found segment;";
#endif
					if ( f->cdt->is_infinite ( faces[0] ) )
					{
#if DEBUG_SELF_INTERSECTIONS>1
						cout << " boundary segment;";
#endif
						startup_triangle = faces[1];
					}
					else
						if ( f->cdt->is_infinite ( faces[1] ) )
						{
#if DEBUG_SELF_INTERSECTIONS>1
							cout << " boundary segment;";
#endif
							startup_triangle = faces[0];
						}
						else
						{ // it's a segment in the middle.. choose the good one using the normal
#if DEBUG_SELF_INTERSECTIONS>1
							cout << " choosing correct side;";
#endif
							KernelExact::Point_3 p_with_norm = p_ref + TO_VECTOR3_EXACT ( f->normal() );
							KernelExact::Line_3 l ( p_ref,p_with_norm );
							KernelExact::Plane_3 pl = l.perpendicular_plane ( p_ref );
							CGAL::Oriented_side good_side = pl.oriented_side ( p_with_norm );
							
							job.updated_norm_factor = 1;
							if ( pl_prev.oriented_side ( pt3 ) ==good_side_prev )
								startup_triangle = faces[0];
							else
							{
								
								startup_triangle = faces[1];
							}
#if SELF_INTER_REMOVAL_POSITIVE_NORMALS_ONLY==1
							if ( pl.oriented_side ( p_prev_opposite ) !=good_side )
							{ //this means we need to negate the prev. choice
								if ( pl_prev.oriented_side ( pt3 ) ==good_side_prev )
									startup_triangle = faces[1];
								else
								{
									
									startup_triangle = faces[0];
								}
							}
#endif
							
						}
#if DEBUG_SELF_INTERSECTIONS>1
					cout << endl;
#endif
					found_sol = true;
					break;
				}
			}
		}
		
		if ( !found_sol )
		{
			cout << "ERROR: _findStartupCDTriangle failed for:" << endl;
			cout << "    - triangle : "<< f->triangle() << endl;
			cout << "    - segment : "<< job.entrance_segment << endl;
		}
		job.start_subfacet=startup_triangle;
		return found_sol;
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// helper  function for self intersection
	// --------------------------------------
	void Mesh::_updateNormal ( Triangle_Job & job )
	{
		// TODO: take into account f->removal_entrance_norm;
		job.updated_norm = TO_VECTOR3_EXACT ( job.facet->normal() ) *job.updated_norm_factor; //TO_VECTOR3_EXACT(
		
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::removeSelfIntersections()
	{
#if DEBUG_MESH>0			
		cout << "REMOVING SELF INTERSECTIONS" << endl;
#endif	
		queue<Triangle_Job> S_queue;
		queue<Triangle_Job> P_queue;
		
		Polyhedron p_new;
		Build_New_Mesh<HalfedgeDS,KernelExact> builder;
		
		bool had_partial_triangles;
		
		// this should mark all the intersections + compute all the delaunay triangulations for the intersections + mark all substriangles as unvisited
		no_intersections = checkIntersections ( true );
		if ( no_intersections==0 )
		{
#if DEBUG_MESH>0
			cout << "NO SELF INTERSECTIONS FOUND." << endl;
#endif		
			return;
		}
#if DEBUG_MESH>0			
		cout << "FINDING EXTERNAL TRIANGLES" << endl;
#endif	
		// mark all triangles as unvisited
		for ( Facet_iterator f=p.facets_begin(); f!=p.facets_end(); ++f )
			f->removal_status = 'U';

		
#if SEED_METHOD==4
		if (_AABB_tree!=NULL) { //if allocated, erase the tree
			delete _AABB_tree;
			_AABB_tree=NULL;
		}
#endif
		// add original seed triangle
		Facet_handle seeder = _getSeedTriangle();
		if ( seeder==NULL )
		{
#if DEBUG_MESH>0		
			cout << "No seed triangle could be found" << endl;
#endif		
			return;
		}

		seeder->removal_status='S';
		Triangle_Job job ( seeder );
		job.entrance_norm = TO_VECTOR3_EXACT ( seeder->normal() );
		S_queue.push ( job );

		do
		{
			while ( !S_queue.empty() )
			{ // process valid triangles
				Triangle_Job currentJob = S_queue.front(); S_queue.pop();
				Facet_handle f =  currentJob.facet;
				_updateNormal ( currentJob );
				builder.addTriangle ( f->triangleExact(),currentJob );
				//			Halfedge_handle f_h1 = f->facet_begin();
				//			Halfedge_handle f_h2 = f_h1->next();
				//			Halfedge_handle f_h3 = f_h2->next();
				
				
				//			builder.addTriangle ( KernelExact::Triangle_3 ( TO_POINT3_EXACT ( f_h1->vertex()->point() ), TO_POINT3_EXACT ( f_h2->vertex()->point() ), TO_POINT3_EXACT ( f_h3->vertex()->point() ) ));
				
				
				HF_circulator h_edge = f->facet_begin();
				do {
					if (h_edge->is_border_edge()) continue;
					Facet_handle f_neigh= h_edge->opposite()->facet();
					if ( f_neigh!=Facet_handle() && f_neigh->removal_status!='V'  && f_neigh->removal_status!='S') {
						Triangle_Job new_job ( f_neigh );
						new_job.entrance_norm = currentJob.updated_norm;
						new_job.updated_norm_factor = currentJob.updated_norm_factor;
						new_job.entrance_segment = KernelExact::Segment_3 ( TO_POINT3_EXACT ( h_edge->vertex()->point() ),TO_POINT3_EXACT ( h_edge->prev()->vertex()->point() ) );
						new_job.entrance_opposite_point = TO_POINT3_EXACT ( h_edge->next()->vertex()->point() );
						
						if ( !f_neigh->hasIntersections() )
						{
							f_neigh->removal_status='V';
							S_queue.push ( f_neigh );
						}
						else
						{
							f_neigh->removal_status='P';
							if ( !_findStartupCDTriangle ( new_job ) )
							{
								cout << "RemoveSelfIntersection: Error while trying to find the the good startup subtriangle." << endl;
								return;
							}
							
							if ( new_job.start_subfacet->info() == false )
							{
								new_job.start_subfacet->info() = true;
								P_queue.push ( new_job );
							}
						}
					}
					
				} while ( ++h_edge != f->facet_begin() );
				
			}
			
			had_partial_triangles = false;
			while ( !P_queue.empty() ) { // process partially valid triangle
				had_partial_triangles = true;
				
				Triangle_Job currentJob = P_queue.front(); P_queue.pop();
				Facet_handle f =  currentJob.facet;
				// update possible duplicates
				_updateNormal ( currentJob );
				
				queue<CDT::Face_handle> CDT_queue;
				CDT_queue.push ( currentJob.start_subfacet );
				
				while ( !CDT_queue.empty() )
				{
					CDT::Face_handle CDT_f = CDT_queue.front(); CDT_queue.pop();
					builder.addTriangle ( KernelExact::Triangle_3 ( CDT_f->vertex ( 0 )->point_3d,CDT_f->vertex ( 1 )->point_3d,CDT_f->vertex ( 2 )->point_3d ),currentJob );
					
					for ( int i=0;i<3;i++ )
					{
						CDT::Edge CDT_e;
						CDT_e.first = CDT_f;
						CDT_e.second = i;
						
						if ( f->cdt->is_constrained ( CDT_e ) )
						{ // gotto see which original constraint belongs to
							KernelExact::Plane_3 plane_eq ( TO_POINT3_EXACT ( f->triangle() [0] ),TO_POINT3_EXACT ( f->triangle() [1] ),TO_POINT3_EXACT ( f->triangle() [2] ) );
							//KernelExact::Plane_3 plane_eq= f->triangle().supporting_plane();
							

							CDT::Vertex_handle CDT_v1,CDT_v2;
							CDT_v1 = CDT_f->vertex ( f->cdt->cw ( i ) );
							CDT_v2 = CDT_f->vertex ( f->cdt->ccw ( i ) );
							
							KernelExact::Segment_3 CDT_seg ( CDT_v1->point_3d,CDT_v2->point_3d );
							KernelExact::Segment_2 CDT_seg_2 (CDT_v1->point(),CDT_v2->point());
							KernelExact::Point_3 CDT_opposite = CDT_f->vertex ( i )->point_3d;
							Facet_handle neigh_facet;
							
							
							int CDT_pos=0;
							// try to see if it is on a CONSTRAINED_VERTEX
							for ( std::vector<KernelExact::Segment_3 *>::iterator j = f->inter_segments.begin(); j != f->inter_segments.end(); ++j )
							{
								KernelExact::Segment_3& tmpSeg = **j;
								KernelExact::Segment_2 tmpSeg_2 = KernelExact::Segment_2(plane_eq.to_2d(tmpSeg.source()),plane_eq.to_2d(tmpSeg.target()));

								if ( tmpSeg.has_on ( CDT_v1->point_3d ) && tmpSeg.has_on ( CDT_v2->point_3d ) )
//								if ( tmpSeg_2.has_on (CDT_v1->point()) && tmpSeg_2.has_on (CDT_v2->point()) )
								{
									neigh_facet = f->inter_facets[CDT_pos]->halfedge()->facet();
									break;
								}
								CDT_pos++;
							}
							
							if ( neigh_facet==NULL )
							{ //if not, it's gotto be a Triangle EDGE
								HF_circulator hf =f->facet_begin();
								do
								{
									KernelExact::Segment_3 tmpSeg ( TO_POINT3_EXACT ( hf->vertex()->point() ),TO_POINT3_EXACT ( hf->prev()->vertex()->point() ) );
									KernelExact::Segment_2 tmpSeg_2 ( plane_eq.to_2d(TO_POINT3_EXACT( hf->vertex()->point())),plane_eq.to_2d(TO_POINT3_EXACT( hf->prev()->vertex()->point())) );
//									if ( tmpSeg_2.has_on (CDT_v1->point()) && tmpSeg_2.has_on(CDT_v2->point()) )
									if ( tmpSeg.has_on ( CDT_v1->point_3d ) && tmpSeg.has_on ( CDT_v2->point_3d ) )
									{
										neigh_facet = hf->opposite()->facet();
										break;
									}
								}
								while ( ++hf != f->facet_begin() );
							}
							
							if ( neigh_facet==NULL ) {
								cout << "RemoveSelfIntersection: Subconstraint not found within all original constraints + triangle edges." << endl;
								std::map<KernelExact::Point_2,int> local_mapping;
								int local_counter=0;
								int tmp_i1,tmp_i2;
								#define SET_INDEX(X,Y) if(local_mapping.find(X)==local_mapping.end()) {local_mapping[X]=++local_counter;Y=local_counter;} else {Y=local_mapping[X];}
								#define LOCAL_PROJECTION(X) plane_eq.to_2d(X)
								#define PRINT_SEGMENT(X,Y) SET_INDEX(X,tmp_i1); SET_INDEX(Y,tmp_i2); cout << X << " (" << (char)('A'+tmp_i1-1) << ") ==> " <<  Y << " (" << (char)('A'+tmp_i2-1) << ")" << endl;
								cout << "Facet id="<<std::distance(p.facets_begin(),f) + 1<< endl;
								cout << "Intersection Segments:" << endl;
								for ( std::vector<KernelExact::Segment_3 *>::iterator j = f->inter_segments.begin(); j != f->inter_segments.end(); ++j ) {
									PRINT_SEGMENT(LOCAL_PROJECTION((*j)->source()),LOCAL_PROJECTION((*j)->target()))
								}
								cout << "Triangle:" << endl;
								HF_circulator hf =f->facet_begin();
								do {
									PRINT_SEGMENT(LOCAL_PROJECTION(TO_POINT3_EXACT(hf->vertex()->point())),LOCAL_PROJECTION(TO_POINT3_EXACT(hf->prev()->vertex()->point())))
								}
								while ( ++hf != f->facet_begin() );
								cout << "Delaunay Triangulation:" << endl;
								for ( CDT::Finite_edges_iterator eit = f->cdt->finite_edges_begin(); eit != f->cdt->finite_edges_end(); ++eit )
								{
									CDT::Edge pp = *eit;
									CDT::Face_handle ff = pp.first;
									int fedge = pp.second;
									
									KernelExact::Point_3 p1 = ff->vertex ( f->cdt->cw ( fedge ) )->point_3d;
									KernelExact::Point_3 p2 = ff->vertex ( f->cdt->ccw ( fedge ) )->point_3d;
				
									if ( f->cdt->is_constrained ( *eit ) ) 
										cout << " (*) ";
									else 
										cout << " (-) ";
									PRINT_SEGMENT(LOCAL_PROJECTION(p1), LOCAL_PROJECTION(p2))
								}
								cout << "Problematic constraint: ";
								PRINT_SEGMENT(LOCAL_PROJECTION(CDT_v1->point_3d),LOCAL_PROJECTION(CDT_v2->point_3d))

								saveFormat("error_mesh.off");
								return;
							}
							
							//time to add neigh_facet
							Triangle_Job new_job ( neigh_facet );
							new_job.entrance_segment = CDT_seg;
							new_job.entrance_opposite_point = CDT_opposite;
							new_job.updated_norm_factor = currentJob.updated_norm_factor;
							new_job.entrance_norm = currentJob.updated_norm;
							
							if ( neigh_facet->removal_status!='V' &&  neigh_facet->removal_status!='S') {
								
								if ( neigh_facet->hasIntersections()==false ) {
									neigh_facet->removal_status='V';
									S_queue.push ( new_job );
								}
								else {
									neigh_facet->removal_status='P';
									if ( !_findStartupCDTriangle ( new_job ) )
									{
										cout << "RemoveSelfIntersection: Error while trying to find the the good startup subtriangle." << endl;
										saveFormat("error_mesh.off");
										return;
									}
									
									if (new_job.start_subfacet->info() ==false)
									{
										new_job.start_subfacet->info() = true;
										P_queue.push ( new_job );
									}
								}
								
							}
							
						} //if is contrainted
						else
						{ // not constrainted - safe to process the neighbor
							if ( CDT_f->neighbor ( i )->info() ==false )
							{
								CDT_f->neighbor ( i )->info() = true;
								CDT_queue.push ( CDT_f->neighbor ( i ) );
							}
						}
						
					}
				}
			}
			if ( ! ( ( !S_queue.empty() ) || ( !P_queue.empty() ) || had_partial_triangles ) )
			{
				Facet_handle seeder = _getSeedTriangle();
				if ( seeder!=NULL )
				{
					seeder->removal_status='V';
					Triangle_Job job ( seeder );
					job.entrance_norm = TO_VECTOR3_EXACT ( seeder->normal() );
					S_queue.push ( job );
				}
			}
			
		}
		while ( ( !S_queue.empty() ) || ( !P_queue.empty() ) || had_partial_triangles );
		
		p_new.delegate ( builder );
		if ( !p_new.is_valid ( false ) )
		{
			cout << "new mesh is not valid. abandoning." << endl;
			saveFormat("error_mesh.off");
		}
		else
		{
			lock();
			p = p_new;
			inter_segments.clear();
			unlock();
		}
		
		updateMeshData();
		
		// smooth a bit the singular vertices!!
		lock();
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
			if ( vi->flag[0] )
				vi->point() = vi->point() + vi->laplacian() *0.25;// - vi->laplacian_deriv()*0.4;
		unlock();
		updateMeshData();
		
		//	smooth(0.02,4);
		
	}
	

	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::mergeTriangleSoup() {
		
		Build_New_Mesh<HalfedgeDS,KernelExact> builder;
		for(Facet_iterator f=p.facets_begin(); f !=p.facets_end(); f++)
			builder.addTriangle(f->triangleExact());
		
		lock();
		Polyhedron p_new;
		p_new.delegate ( builder );
		p = p_new;
		unlock();
		updateMeshData();
		
	}
	

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	bool Mesh::loadAsTriangleSoup(const char *filename) {
		ifstream is ( filename );
		//is.open(filename);
		is.width (4);
		char mesh_type[4];
		is>> mesh_type;
		
		Build_New_Mesh<HalfedgeDS,KernelExact> builder;
		
		if ( (strcmp(mesh_type,"OFF")==0) || (strcmp(mesh_type,"off")==0) ) {
			cout << "Importing OFF file as a triangle soup." << endl;
			is.width(1000);
			float tmpValue;
			int no_vertices,no_facets, no_junk;	
			is >> no_vertices; is >> no_facets; is >> no_junk;
			
			vector<KernelExact::Point_3> points;
			float coord[3];
			for ( int i=0;i<no_vertices;i++ ) {
				is >> coord[0]; is >> coord[1]; is >> coord[2];
				points.push_back(KernelExact::Point_3(coord[0],coord[1],coord[2]));
				
			}
			for ( int j=0;j<no_facets;j++ ) {
				int no_edges;
				int t_e[3];
				is >> no_edges;
				if (no_edges!=3) {
					cout << "only triangular meshes supported at this time." << endl;
					return false;
				}
				for(int t=0;t<no_edges;t++) 
					is >> t_e[t];
				builder.addTriangle(KernelExact::Triangle_3(points[t_e[0]],points[t_e[1]],points[t_e[2]]));
			}
			
			/*		for(Facet_iterator f=p.facets_begin(); f !=p.facets_end(); f++)
			 builder.addTriangle(f->triangleExact());
			 */	
			lock();
			Polyhedron p_new;
			p_new.delegate ( builder );
			p = p_new;
			unlock();
			updateMeshData();
			return true;
		}
		else {
			cout << "Format '" << mesh_type << "' not supported. We support only OFF files at this time." << endl;
			return false;
		}
	}
