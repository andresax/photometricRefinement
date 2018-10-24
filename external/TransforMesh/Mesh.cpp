/*
 *  Mesh.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 28/11/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include <CGAL/AABB_tree.h> // must be inserted before kernel
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include "Mesh.h"


//#include "Camera.h"
#include "MeshHelper.h"
//#include "ColorMap.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <cctype>

#include <CGAL/Unique_hash_map.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
//#include <CGAL/Subdivision_method_3.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Object.h>
#include <CGAL/intersections.h>

#include <CGAL/squared_distance_2.h>
#include <CGAL/Timer.h>
#include <queue>
#include <map>

//#include "PolygonTesselator.h"
#include "Histogram.h"

#include <CGAL/Convex_hull_traits_3.h>
#include <CGAL/convex_hull_3.h>

#include <CGAL/Aff_transformation_3.h>
typedef CGAL::Aff_transformation_3<Kernel> Aff_trans;

#define CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION 1
#include <CEP/intersection/Triangle_3_Triangle_3.h>

#ifdef WITH_CGAL_BLAS
#include <CGAL/Lapack/Linear_algebra_lapack.h>
typedef CGAL::Lapack_matrix Linear_system_matrix;
typedef CGAL::Lapack_vector Linear_system_vector;		
typedef CGAL::Lapack_svd    Linear_system_solver;				
#endif


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

#define v_signed_angle(A,B) atan2(v_norm(CGAL::cross_product(v_normalized(A),v_normalized(B)))/v_norm(B)/v_norm(A), v_normalized(A) * v_normalized(B))
#define ENDS_WITH(S,A) (strcmp(S+(strlen(S)-strlen(A)),A)==0)

float vector_angle_2d ( const Kernel::Vector_2& a, const Kernel::Vector_2& b )
{
	float cosine = a*b / ( v_norm ( a ) * v_norm ( b ) );
	// rounding errors might make dotproduct out of range for cosine
	if ( cosine > 1 ) cosine = 1;
	else if ( cosine < -1 ) cosine = -1;
	
	if ( ( a.x() * b.y() - a.y() * b.x() ) < 0 )
		return -acos ( cosine );
	else
		return acos ( cosine );
}

float vector_angle ( Vector a3, Vector b3 )
{
	/*  Kernel::Point_3 op3 = Kernel::Point_3(0,0,0);
	 Kernel::Point_3 ap3 = Kernel::Point_3(a3.x(),a3.y(),a3.z());
	 Kernel::Point_3 bp3 = Kernel::Point_3(b3.x(),b3.y(),b3.z());
	 Kernel::Plane_3 pl = Kernel::Plane_3(op3,CGAL::cross_product(a3,b3));
	 Kernel::Vector_2 a = pl.to_2d(ap3) - pl.to_2d(op3);
	 Kernel::Vector_2 b = pl.to_2d(bp3) - pl.to_2d(op3);
	 cout << "projected a=" << a << " and b=" << b << endl;
	 return vector_angle_2d(a,b);*/
	
	a3 = v_normalized ( a3 );
	b3 = v_normalized ( b3 );
	
	Vector u = CGAL::cross_product ( a3,b3 ); u = v_normalized ( u );
	Vector n = CGAL::cross_product ( u,a3 ); n = v_normalized ( n );
	
	double angle=v_angle ( a3,b3 );
	
	
	cout << "a=" << a3 << endl;
	cout << "b=" << b3 << endl;
	cout << "u=" << u << endl;
	cout << "n=" << n << endl;
	cout << "angle=" << angle << endl;
	cout << "n*b=" << n*b3 << endl;
	
	if ( n*b3 > 0 ) return angle;
	else return -angle;
}

int vector_angle_sign ( const Vector& a3, const Vector& b3 )
{
	Kernel::Point_3 op3 = Kernel::Point_3 ( 0,0,0 );
	Kernel::Point_3 ap3 = Kernel::Point_3 ( a3.x(),a3.y(),a3.z() );
	Kernel::Point_3 bp3 = Kernel::Point_3 ( b3.x(),b3.y(),b3.z() );
	Kernel::Plane_3 pl = Kernel::Plane_3 ( op3,b3 );
	
	if ( pl.has_on_positive_side ( ap3 ) ) return 1;
	else return -1;
	/*
	 Kernel::Vector_2 a = pl.to_2d(ap3) - pl.to_2d(op3);
	 Kernel::Vector_2 b = pl.to_2d(bp3) - pl.to_2d(op3);
	 cout << "projected a=" << a << " and b=" << b << endl;
	 return vector_angle_2d(a,b);
	 */
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
Mesh::Mesh()
#ifdef USE_MESH_LOCKING
:_mutex(QMutex::Recursive)
#endif	

{
	//	CGAL::force_ieee_double_precision();
	//	CGAL::Fixed_precision_nt::init(-30);
	//	cout << "initialized the fixed precision unit value with " <<  CGAL::Fixed_precision_nt::unit_value () << endl;
	loaded_filename = NULL;
	loaded_file_type = NULL;
	loaded_vect_filename = NULL;
	tmpFix = 5;
	
	is_search_init = false;
	is_mapping_init = false;
	curv_comp_method = Curv_Approx;
	curv_comp_neigh_rings = 1; // currently only used by Dong
	close_color_sigma = 0.99;
	
	interactive_colors_max_facet_no=-1;
#if SEED_METHOD==4
	_AABB_tree=NULL;
#endif	
	qual_mode = Qual_No_Computation;
	//qual_mode = Qual_Color_Deriv;
	/*
	 Vector v1=v_normalized(Vector(1,1,1));
	 Vector v2=v_normalized(Vector(1,2,3));
	 
	 cout << "signed angle=" << vector_angle(v1,v2) << endl;
	 cout << "inverse signed angle=" << vector_angle(v2,v1) << endl;
	 
	 Kernel::Vector_2 v1_2d=v_normalized(Kernel::Vector_2(0,1));
	 Kernel::Vector_2 v2_2d=v_normalized(Kernel::Vector_2(3,1));
	 
	 cout << "2D - signed angle=" << vector_angle_2d(v1_2d,v2_2d) << endl;
	 cout << "2D - inverse signed angle=" << vector_angle_2d(v2_2d,v1_2d) << endl;
	 */
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
Mesh::Mesh(const Mesh& other) {
	p=other.p;


	if (other.loaded_filename!=NULL)  {
		loaded_filename = new char[strlen (other.loaded_filename) +1]; strcpy (loaded_filename,other.loaded_filename);
	}
	else
		loaded_filename = NULL;

	if (other.loaded_file_type !=NULL)  {
		loaded_file_type = new char[strlen (other.loaded_file_type) +1]; strcpy (loaded_file_type,other.loaded_file_type);
	}
	else
		loaded_file_type = NULL;

	if (other.loaded_vect_filename !=NULL) {
		loaded_vect_filename = new char[strlen (other.loaded_vect_filename) +1]; strcpy (loaded_vect_filename,other.loaded_vect_filename);
	}
	else
		loaded_vect_filename = NULL;

	tmpFix = other.tmpFix;
	is_search_init = other.is_search_init ;
	is_mapping_init = other.is_mapping_init;
	curv_comp_method = other.curv_comp_method;
	curv_comp_neigh_rings = other.curv_comp_neigh_rings;
	close_color_sigma = other.close_color_sigma;
	unlock();
	updateMeshData();
}


const Mesh& Mesh::operator=(const Mesh& other) { 
	lock();
	p=other.p;


	if (other.loaded_filename!=NULL)  {
		loaded_filename = new char[strlen (other.loaded_filename) +1]; strcpy (loaded_filename,other.loaded_filename);
	}
	else
		loaded_filename = NULL;

	if (other.loaded_file_type !=NULL)  {
		loaded_file_type = new char[strlen (other.loaded_file_type) +1]; strcpy (loaded_file_type,other.loaded_file_type);
	}
	else
		loaded_file_type = NULL;

	if (other.loaded_vect_filename !=NULL) {
		loaded_vect_filename = new char[strlen (other.loaded_vect_filename) +1]; strcpy (loaded_vect_filename,other.loaded_vect_filename);
	}
	else
		loaded_vect_filename = NULL;

	tmpFix = other.tmpFix;
	is_search_init = other.is_search_init ;
	is_mapping_init = other.is_mapping_init;
	curv_comp_method = other.curv_comp_method;
	curv_comp_neigh_rings = other.curv_comp_neigh_rings;
	close_color_sigma = other.close_color_sigma;
	unlock();
	updateMeshData();
	return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
Mesh::~Mesh()
{
	clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::clear ( bool cleanFilenames ) {
	lock();
	//std::cout<<"lock ENDED"<<std::endl;
	p.clear();
	//std::cout<<"p.clear ENDED"<<std::endl;
	for ( int i=0;i<inter_segments.size();i++ ) delete inter_segments[i];
	inter_segments.clear();

	
	tmpFix = 5;
	if ( (loaded_filename!=NULL) && cleanFilenames ) delete[] loaded_filename;
	if ( (loaded_file_type!=NULL) && cleanFilenames ) delete[] loaded_file_type;
	if ( (loaded_vect_filename!=NULL) && cleanFilenames ) delete[] loaded_vect_filename;
	unlock();
}

void Mesh::makeHole(Vertex *el, int noRings)
{
	// get the list of neighs to remove
	std::vector< std::pair<Vertex*,float> > elems = getGeodesicNeighbourhood(*el,edge_avg*noRings,true);
	std::map<Facet*,int> facet_map;
	
	for(int i=0;i<elems.size();i++) 
	{
		HV_circulator c = elems[i].first->vertex_begin();
		HV_circulator d = c;
		CGAL_For_all ( c, d )
		{
			if (c->is_border()==false) facet_map[&(*c->facet())] = 1;
		}
	}
	for (std::map<Facet*,int>::iterator it=facet_map.begin();it!=facet_map.end();it++)
		p.erase_facet(it->first->halfedge());
	
	updateMeshData();
	resetSimplexIndices();
	
}



void Mesh::displayInfo() {
	
	cout << "Mesh Info" << endl;
	cout << " - sizeOf(Vertex)" << sizeof ( Vertex ) << endl;
	cout << " - sizeOf(Facet)" << sizeof ( Facet ) << endl;
	cout << " - sizeOf(Mesh)" << sizeof ( this ) << endl;
	cout << " - facets = " << p.size_of_facets() << "; vertices = " << p.size_of_vertices() << ";  border_edges = " << p.size_of_border_edges() << endl ;
	cout << " - is_valid = " << p.is_valid(0,1) << "; is_triangular = " << p.is_pure_triangle() << endl;
	cout << " - edge stats: min = " << edge_min <<  "; avg = " << edge_avg << "; max = " << edge_max << endl;
	cout << " - area stats: min = " << area_min <<  "; avg = " << area_avg << "; max = " << area_max << endl;
	cout << " - laplacian stats: min = " << laplacian_min <<  "; avg = " << laplacian_avg <<  ";  max = " << laplacian_max << endl;
#ifdef COMPUTE_CURVATURE_STATS
    cout << " - mean_curvature stats: min = " << curvature_min <<  "; avg = " << curvature_avg << "; std_dev=" << curvature_std_dev << ";  max = " << curvature_max << endl;
#endif
	
	bool has_nan = false;
	for ( Vertex_iterator v=p.vertices_begin();v!=p.vertices_end();v++ )
		if ( isnan ( v_norm ( v->point() - CGAL::ORIGIN ) ) ) has_nan=true;
	if ( !has_nan )
		cout << " - all vertices are valid numbers" << endl;
	else
		cout << " - there are vertices with NAN" << endl;
	float *bbox = getBoundingBox();
	cout << " - bounding box: X=[" << bbox[0] << "," << bbox[3] << "]" << endl;
	cout << " - bounding box: Y=[" << bbox[1] << "," << bbox[4] << "]" << endl;
	cout << " - bounding box: Z=[" << bbox[2] << "," << bbox[5] << "]" << endl;
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//assumes that it has already loaded the mesh once
void Mesh::reload() {
	lock();
	char * tmp_filename, *tmp_file_type, *tmp_vect_filename;
	if ( loaded_filename )
	{
		tmp_filename = loaded_filename;
		loaded_filename = NULL;
		tmp_file_type = loaded_file_type;
		loaded_file_type = NULL;
		loadFormat ( tmp_filename,true );
		delete[] tmp_filename;
		delete[] tmp_file_type;
		if ( loaded_vect_filename )
		{
			tmp_vect_filename = loaded_vect_filename;
			loadVectorField ( tmp_vect_filename,true );
			delete[] tmp_vect_filename;
		}
	}
	else
	{
		printf ( "empty file name\n" );
	}
	unlock();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
float* Mesh::getBoundingBox() {
	
	//init the bounding box with the first vertex	
	Point first_point(0,0,0);
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		if (vi->getVisibility()) {
			first_point = vi->point();
			break;
		}
	
	
	for ( int i=0;i<6;i++ )
		bounding_box[i] = first_point[i%3];
	lock();
	//do a min/max
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		if (vi->getVisibility()==false) continue;
		for ( int j=0;j<3;j++ )
		{
			if ( bounding_box[j] > TO_FLOAT ( vi->point() [j] ) ) bounding_box[j] = TO_FLOAT ( vi->point() [j] );
			if ( bounding_box[j+3] < TO_FLOAT ( vi->point() [j] ) ) bounding_box[j+3] = TO_FLOAT ( vi->point() [j] );
		}
	}
	unlock();
	return bounding_box;
}


//////////////////////////////////////////////////////////////////////////////////////////
std::vector< std::pair<Vertex*,int> > Mesh::getRingNeighbourhood(std::vector<Vertex*> vertices,int ring_size, bool include_original) {

	std::map<Vertex*,int> orig_vertices;
	
	std::map<Vertex*,int> neigh_map;
	std::map<Vertex*,int>::iterator iter;	
	
	std::queue<Vertex*> elems;
	std::vector< std::pair<Vertex*,int> > result;
		
	Vertex* el;
	Vertex* next_el;

	if (ring_size < 0) return result;	
	// add base level	

	for(int i=0;i<vertices.size();i++) {
		Vertex *pV = vertices[i];
		orig_vertices[pV] = 1;	// set the original vertices 
		elems.push(pV);
		neigh_map[pV]=0;
		//	pV->set_color(0,0,0);
		
	}
	
	while (elems.size() > 0) {
		el = elems.front(); elems.pop();
		
		if ((orig_vertices.find(el)!=orig_vertices.end()) || include_original) {
			result.push_back(pair<Vertex*,int>(el,neigh_map[el]));
		}
		
		//		el->set_color((1.0/ring_size)*(neigh_map[el]),0,0);
		if (neigh_map[el]==ring_size) continue;
		//circulate one ring neighbourhood
		HV_circulator c = el->vertex_begin();
		HV_circulator d = c;
		CGAL_For_all ( c, d ) {
			Vertex & next_el_v = *c->opposite()->vertex();
 			next_el = &(*(c->opposite()->vertex()));
			iter=neigh_map.find(next_el);
			if( iter == neigh_map.end() ) { // if the vertex has not been already taken
				elems.push(next_el);
				neigh_map[next_el]=neigh_map[el]+1;
			}
		}
	}
	
	return result;
}



//////////////////////////////////////////////////////////////////////////////////////////
std::vector< std::pair<Vertex*,int> > Mesh::getRingNeighbourhood(Vertex& v,int ring_size, bool include_original) {
	std::map<Vertex*,int> neigh_map;
	std::map<Vertex*,int>::iterator iter;	

	std::queue<Vertex*> elems;
	std::vector< std::pair<Vertex*,int> > result;

	Vertex* el;
	Vertex* next_el;
	// add base level	
	elems.push(&v);
	neigh_map[&v]=0;
//	v.set_color(0,0,0);
	
	if (ring_size < 0) return result;

	while (elems.size() > 0) {
		el = elems.front(); elems.pop();
		if ((el != &v) || include_original) {
			result.push_back(pair<Vertex*,int>(el,neigh_map[el]));
		}
			
//		el->set_color((1.0/ring_size)*(neigh_map[el]),0,0);
		if (neigh_map[el]==ring_size) continue;
		//circulate one ring neighbourhood
		HV_circulator c = el->vertex_begin();
		HV_circulator d = c;
		CGAL_For_all ( c, d ) {
			Vertex & next_el_v = *c->opposite()->vertex();
 			next_el = &(*(c->opposite()->vertex()));
			iter=neigh_map.find(next_el);
			if( iter == neigh_map.end() ) { // if the vertex has not been already taken
				elems.push(next_el);
				neigh_map[next_el]=neigh_map[el]+1;
			}
		}
	}

	return result;
}


//////////////////////////////////////////////////////////////////////////////////////////
std::vector< std::pair<Vertex*,float> > Mesh::getGeodesicNeighbourhood(Vertex& v,float geo_radius, bool include_original) {
	std::map<Vertex*,float> neigh_map;
	std::map<Vertex*,float>::iterator iter;	
	std::vector< std::pair<Vertex*,float> > result;

	std::queue<Vertex*> elems;
	
	Vertex* el;
	Vertex* next_el;
	// add base level	
	elems.push(&v);
	neigh_map[&v]=0;
	
	while (elems.size() > 0) {
		el = elems.front(); elems.pop();
		if ((el != &v) || include_original) {
			result.push_back(pair<Vertex*,float>(el,0)); // the geo distance is added @ the end
		}
		
//		if (v.id==0) el->set_color((neigh_map[el]/geo_radius),0,0);

		//circulate one ring neighbourhood
		HV_circulator c = el->vertex_begin();
		HV_circulator d = c;
		CGAL_For_all ( c, d ) {
			Vertex & next_el_v = *c->opposite()->vertex();
 			next_el = &(*(c->opposite()->vertex()));
			float dist = neigh_map[el] + v_norm(next_el->point()-el->point());						
			iter=neigh_map.find(next_el);
			if( iter == neigh_map.end()) { // if the vertex has not been already taken
				if (dist<geo_radius) {
					elems.push(next_el);
					neigh_map[next_el]=dist;
				}
			}
			else {
				if (neigh_map[next_el] > dist)
					neigh_map[next_el] = dist;
			}
		}
	}
	
	// meanwhile we might have found a shorter path to get there
	for(int i=0;i<result.size();i++)
		result[i].second = neigh_map[result[i].first];
	
	return result;
}


void Mesh::create_center_vertex ( Facet_iterator f )
{
	lock();
	Vector vec ( 0.0, 0.0, 0.0 );
	std::size_t order = 0;
	HF_circulator h = f->facet_begin();
	do
	{
		vec = vec + ( h->vertex()->point() - CGAL::ORIGIN );
		++ order;
	}
	while ( ++h != f->facet_begin() );
	CGAL_assertion ( order >= 3 ); // guaranteed by definition of polyhedron
	Point center =  CGAL::ORIGIN + ( vec / order );
	Halfedge_handle new_center = p.create_center_vertex ( f->halfedge() );
	new_center->vertex()->point() = center;
	unlock();
}


Point Mesh::smooth_old_vertex ( Vertex_handle v ) const
{
	CGAL_precondition ( ( CGAL::circulator_size ( v->vertex_begin() ) & 1 ) == 0 );
	std::size_t degree = v->vertex_degree();
	double alpha = ( 4.0 - 2.0 * std::cos ( 2.0 * CGAL_PI / degree ) ) / 9.0;
	Vector vec = ( v->point() - CGAL::ORIGIN ) * ( 1.0 - alpha );
	HV_circulator h = v->vertex_begin();
	do
	{
		vec = vec + ( h->prev()->vertex()->point() - CGAL::ORIGIN )
		* alpha / degree;
		++ h;
		CGAL_assertion ( h != v->vertex_begin() ); // even degree guaranteed
		++ h;
	}
	while ( h != v->vertex_begin() );
	return ( CGAL::ORIGIN + vec );
}


void Mesh::remeshWithThreshold ( float edge_threshold )
{
	
	if ( p.size_of_facets() == 0 ) return;
	
	cout << "REMESH with edge_threshold=" << edge_threshold << endl;
	
	for ( Vertex_iterator v=p.vertices_begin(); v!=p.vertices_end(); v++ )
		v->flag[1] = false; //reset all the flags and set them to false
	
	// We use that new vertices/halfedges/facets are appended at the end.
	std::size_t nv = p.size_of_vertices();
	Vertex_iterator last_v = p.vertices_end();
	-- last_v;  // the last of the old vertices
	Edge_iterator last_e = p.edges_end();
	-- last_e;  // the last of the old edges
	Facet_iterator last_f = p.facets_end();
	-- last_f;  // the last of the old facets
	
	
	int new_vertices=0;
	Facet_iterator f = p.facets_begin();    // create new center vertices
	do
	{
		float edgeStats = ( float ) f->edgeStatistics ( 0 ); //min
		if ( edgeStats > edge_threshold )
		{
			new_vertices++;
			f->setVerticesFlag ( 1,true );
			create_center_vertex ( f );
		}
		
	}
	while ( f++ != last_f );
	if ( new_vertices==0 ) return; // no changes needed
	
	std::vector<Point> pts;   // smooth the old vertices
	pts.reserve ( nv );  // get intermediate space for the new points
	++ last_v; // make it the past-the-end position again
	int flagged_vertices=0;
	for ( Vertex_iterator v=p.vertices_begin(); v!=last_v; v++ )
		if ( v->flag[1] )
		{
			flagged_vertices++;
			pts.push_back ( v->point() + v->laplacian() *0.20 );
		}
		else
			pts.push_back ( v->point() );
	
	std::copy ( pts.begin(), pts.end(), p.points_begin() );
	Edge_iterator e = p.edges_begin();              // flip the old edges
	++ last_e; // make it the past-the-end position again
	
	int count=0;
	while ( e != last_e )
	{
		Halfedge_handle h = e;
		++e; // careful, incr. before flip since flip destroys current edge
		if ( h->vertex()->flag[1] && canFlipEdge ( h ) ) { p.flip_edge ( h ); count++; }
	};
	improveVertexValence();
	cout << "new vertices: " << new_vertices << "; flagged vertices: " << flagged_vertices << "; edge flips: " << count << endl;
	updateMeshData();
	CGAL_postcondition ( p.is_valid() );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::remesh ( int method, int steps )
{
	lock();
	//computeMeshStats();
	//remeshWithThreshold(edge_max/2);
	//	CGAL::Subdivision_method_3::Loop_subdivision(p,1);
	//	CGAL::Subdivision_method_3::CatmullClark_subdivision(p,1);
	//	CGAL::Subdivision_method_3::DooSabin_subdivision(p,1);
	
	//	ensureEdgeSizes(0.0005,MAXFLOAT);
	switch (method) {
			
		case Remesh_Sqrt3:
			CGAL::Subdivision_method_3::Sqrt3_subdivision ( p, steps );
			break;
		case Remesh_Loop:
			CGAL::Subdivision_method_3::Loop_subdivision ( p, steps );
			break;
		case Remesh_CatmullClark:
			CGAL::Subdivision_method_3::CatmullClark_subdivision ( p, steps );
			break;
		case Remesh_DooSabin:
			CGAL::Subdivision_method_3::DooSabin_subdivision ( p, steps );
			break;
		default:
			printf("Unknown subdivision method !\n");
			break;
			
	}
	updateMeshData();
	unlock();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::saveFormat ( const char *filename) {
	
	string name(filename);
	int pos = name.rfind(".");
	if ((pos<0) || (pos > name.length())) {
		cout << "Error. Filename extension should be specified" << endl;
		return false;
	}
	string ext=name.substr(pos+1,name.length());
	for (int i = 0; i < ext.length(); ++i) ext[i] = tolower(ext[i]);
	const char*file_type=ext.c_str();
	cout<<"obtained file extension:" << file_type << endl;
	
	if (ENDS_WITH(filename,".off") || ENDS_WITH(filename,".OFF"))
		return saveFormat(filename,"off");
	if (ENDS_WITH(filename,".coff") || ENDS_WITH(filename,".COFF"))
		return saveFormat(filename,"coff");
	if (ENDS_WITH(filename,".noff") || ENDS_WITH(filename,".NOFF"))
		return saveFormat(filename,"noff");
	if (ENDS_WITH(filename,".P3D") || ENDS_WITH(filename,".p3d"))
		return saveFormat(filename,"P3D");
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::saveFormat ( const char *filename, const char*file_type )
{
	if ( ( strcmp ( file_type,"off" ) ==0 ) || ( strcmp ( file_type,"OFF" ) ==0 ) )
	{
		int mode=0;
		ofstream os ( filename );
		if ( mode==1 )
		{
			os << "OFF" << std::endl << p.size_of_vertices() << ' ' << p.size_of_facets() << " 0" << std::endl;
			
			CGAL::Unique_hash_map< Vertex_handle, int > V;
			
			int number_of_vertices = 0;
			for ( Vertex_iterator i = p.vertices_begin(); i != p.vertices_end(); ++i )
			{
				i->id = number_of_vertices++; 
				//V[i] = number_of_vertices++;
				os << CGAL::to_double ( i->point().x() ) << " " << CGAL::to_double ( i->point().y() ) << " " << CGAL::to_double ( i->point().z() );
				//				os << " " << i->color[0] << " " << i->color[1] << " " << i->color[2] << " 0.8";
				//os << " " << i->normal();
				os << std::endl;
			}
			
			//    std::copy( P.points_begin(), P.points_end(),
			//               std::ostream_iterator<Point_3>( os, "\n"));
			for ( Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i )
			{
				HF_circulator j = i->facet_begin();
				// Facets in polyhedral surfaces are at least triangles.
				CGAL_assertion ( CGAL::circulator_size ( j ) >= 3 );
				os << CGAL::circulator_size ( j ) << ' ';
				do
				{
					//					os << ' ' << std::distance(P.vertices_begin(), j->vertex());
					os << ' ' << j->vertex()->id;
				}
				while ( ++j != i->facet_begin() );
				os << std::endl;
			}
		}
		else
			os << p;
		cout << "* Saving mesh: " << filename << " - " << p.size_of_facets() << " facets and " << p.size_of_vertices() << " vertices." << endl ;
		return true;
	}
	else if ( ( strcmp ( file_type,"coff" ) ==0 ) || ( strcmp ( file_type,"COFF" ) ==0 ) || ( strcmp ( file_type,"noff" ) ==0 ) || ( strcmp ( file_type,"NOFF" ) ==0 ) )
	{
		ofstream os ( filename );
		
		if (( strcmp ( file_type,"coff" ) ==0 ) || ( strcmp ( file_type,"COFF" ) ==0 ))
			os << "COFF" << std::endl;
		else
			os << "NOFF" << std::endl;
		
		os << p.size_of_vertices() << ' ' << p.size_of_facets() << " 0" << std::endl;
		
		//		CGAL::Unique_hash_map< Vertex_handle, int > V;
		
		int number_of_vertices = 0;
		for ( Vertex_iterator i = p.vertices_begin(); i != p.vertices_end(); ++i ) {
			i->id = number_of_vertices++; 
			//V[i] = number_of_vertices++;
			os << CGAL::to_double ( i->point().x() ) << " " << CGAL::to_double ( i->point().y() ) << " " << CGAL::to_double ( i->point().z() );
			if (( strcmp ( file_type,"coff" ) ==0 ) || ( strcmp ( file_type,"COFF" ) ==0 ))
				os << " " << i->color[0] << " " << i->color[1] << " " << i->color[2] << " 1";
			else
				os << " " << i->normal();
			os << std::endl;
		}
		
		//    std::copy( P.points_begin(), P.points_end(),
		//               std::ostream_iterator<Point_3>( os, "\n"));
		for ( Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i )
		{
			HF_circulator j = i->facet_begin();
			// Facets in polyhedral surfaces are at least triangles.
			CGAL_assertion ( CGAL::circulator_size ( j ) >= 3 );
			os << CGAL::circulator_size ( j ) << ' ';
			do
			{
				//				os << ' ' << V[j->vertex() ];
				os << ' ' << j->vertex()->id;
			}
			while ( ++j != i->facet_begin() );
			os << std::endl;
		}
		cout << "* Saving mesh: [" << file_type << "] " << filename << " - " << p.size_of_facets() << " facets and " << p.size_of_vertices() << " vertices." << endl ;
		return true;
	}
	else if ( ( strcmp ( file_type,"P3D" ) ==0 ) || ( strcmp ( file_type,"p3d" ) ==0 ) ) {
		int mode=0;
		if (computeConnectedComponents(false) > 1000) {
			mode=1;	
			cout << "Assuming that this is a triangle soup when computing vertex normals." << endl;
		}
		if (mode==0) {
			ofstream os ( filename );
			
			//os << p.size_of_vertices() << std::endl;
			
			int number_of_vertices = 0;
			for ( Vertex_iterator i = p.vertices_begin(); i != p.vertices_end(); ++i ) {
				os << CGAL::to_double ( i->point().x() ) << " " << CGAL::to_double ( i->point().y() ) << " " << CGAL::to_double ( i->point().z() );
				os << " " << i->normal();
				os << std::endl;
			}
			cout << "* Saving mesh as Points with Normals: [" << file_type << "] " << filename << " - " << p.size_of_vertices() << " vertices." << endl;
			return true;
		}
		else { 
			//this mode computes the normals - such that it works properly with triangle soups
			// main drawback - it messes the order of the original points - only useful for triangle soups
			std::map <Kernel::Point_3, Kernel::Vector_3> vertex_map;
			for ( Vertex_iterator i = p.vertices_begin(); i != p.vertices_end(); ++i )
				vertex_map[i->point()] = Kernel::Vector_3(0,0,0);
			for ( Edge_iterator e = p.edges_begin(); e != p.edges_end(); ++e )
				vertex_map[e->vertex()->point()] = vertex_map[e->vertex()->point()] + e->facet()->normal();
			ofstream os ( filename );
			for(std::map <Kernel::Point_3, Kernel::Vector_3>::iterator mit=vertex_map.begin();mit!=vertex_map.end();mit++) {
				os << mit->first;
				os << " ";
				os << v_normalized(mit->second);
				os << std::endl;
			}
		}
	}
	return false;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
// A modifier creating a tesselated polyhedron with the incremental builder.
/*
 template <class HDS>
 class PolyhedronTesselator : public CGAL::Modifier_base<HDS>, public PolygonTesselator<Kernel::Point_3>
 {
 public:
 Polyhedron pp;
 CGAL::Polyhedron_incremental_builder_3<HDS> *current_builder;
 vector<int> triangle_indices;
 
 
 PolyhedronTesselator ( Polyhedron p_ ) :PolygonTesselator<Kernel::Point_3> ( p_.size_of_vertices() )
 {
 pp = p_;
 }
 
 inline void emitVertex ( GLdouble* coords )
 {
 current_builder->add_vertex ( Kernel::Point_3 ( coords[0],coords[1],coords[2] ) );
 cout <<"adding some new point" << endl;
 }
 
 //from polygon tesselator
 inline void emitTriangle ( int* idx )
 {
 int i1 = idx[0];
 int i2 = idx[1];
 int i3 = idx[2];
 triangle_indices.push_back ( i1 );
 triangle_indices.push_back ( i2 );
 triangle_indices.push_back ( i3 );
 #if DEBUG_TESSELATE==1
 cout << "adding triangle with vertices " << i1 << " " << i2 << " " << i3 << endl;
 #endif
 }
 
 void operator() ( HDS& hds )
 {
 
 CGAL::Polyhedron_incremental_builder_3<HDS> B ( hds, true );
 current_builder=&B;
 B.begin_surface ( pp.size_of_vertices(), triangle_indices.size() /3 );
 
 for ( Vertex_iterator vi=pp.vertices_begin(); vi!=pp.vertices_end();vi++ )
 {
 //			cout << "adding point [" << pvi->point() << "]" <<endl;
 B.add_vertex ( vi->point() );
 }
 
 // invoke the tesselator and queue the triangle_indices - it might add new vertices via emitVertex
 int vertex_id=0;
 
 for ( Vertex_iterator vi=pp.vertices_begin(); vi!=pp.vertices_end();vi++ )
 vi->id = vertex_id++;
 
 for ( Facet_iterator fi=pp.facets_begin(); fi!=pp.facets_end();fi++ )
 {
 HF_circulator h = fi->facet_begin();
 beginPolygon();
 beginContour();
 #if DEBUG_TESSELATE==1
 cout << "starting contour : [ ";
 #endif
 do
 {
 vertex ( h->vertex()->id,h->vertex()->point() );
 #if DEBUG_TESSELATE==1
 cout << h->vertex()->id << " ";
 #endif
 }
 while ( ++h != fi->facet_begin() );
 #if DEBUG_TESSELATE==1
 cout << "]" << endl;
 #endif
 endContour();
 endPolygon();
 }
 
 
 cout << "no of elements in triangle_indices: "<< triangle_indices.size() << endl;
 for ( int i=0;i<triangle_indices.size() /3;i++ )
 {
 B.begin_facet();
 B.add_vertex_to_facet ( triangle_indices[3*i+0] );
 B.add_vertex_to_facet ( triangle_indices[3*i+1] );
 B.add_vertex_to_facet ( triangle_indices[3*i+2] );
 B.end_facet();
 #if DEBUG_TESSELATE==1
 //			cout << "creating triangle [" << triangle_indices[3*i+0] << " " << triangle_indices[3*i+1] << " " << triangle_indices[3*i+2] << "]"<<endl;
 #endif
 }
 B.end_surface();
 
 if ( B.check_unconnected_vertices() )
 {
 cout << "There are unconnected vertices!" << endl;
 B.remove_unconnected_vertices();
 }
 
 if ( B.error() )
 {
 cout << "error obtained while tesselating. Rolling back." << endl;
 B.rollback();
 }
 }//operator()
 
 };
 */


void Mesh::resetSimplexIndices(bool bResetVertices, bool bResetFacets) {
	int id=0;
	
	
	if (bResetVertices)
	{
		id = 0;
		for ( Vertex_iterator vi= p.vertices_begin(); vi != p.vertices_end();vi++ ) {
			vi->id = id++;
		}	
	}

	if (bResetFacets)
	{
		id = 0;
		for ( Facet_iterator fi= p.facets_begin(); fi != p.facets_end(); fi++ ) {
			fi->id = id++;
		}	
	}
	
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
// A modifier creating a triangle with the incremental builder.
template <class HDS, class K>
class Mesh_Loader : public CGAL::Modifier_base<HDS> {
private:
	
	std::vector< std::vector<float> > vertices;
	std::vector< std::vector<int> > triangle_vertices;
	bool bFoundProblems;
public:
enum EVertexAttribs { EVertexX, EVertexY, EVertexZ, EVertexR, EVertexG, EVertexB, EVertexAlpha, EVertexQuality, EVertexIgnore, EVertexNoAttribs};
	
	////////////////////////////////////////////////////////////////////////////////////
	Mesh_Loader() { bFoundProblems = false;}
	
	bool hasFoundProblems()
	{
		return bFoundProblems;
	}
	
	void addVertex(std::vector<float> & params) // EVertexNoAttribs
	{
		vertices.push_back(params);
	}
	
	void addTriangle(std::vector<int>& params)
	{
		triangle_vertices.push_back(params);
	}
	
	
	void operator() ( HDS& hds ) {
		
		CGAL::Polyhedron_incremental_builder_3<HDS> B ( hds, false );
		B.begin_surface ( vertices.size(), triangle_vertices.size() );
		
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;
		
		// add the vertices		
		for(int i=0;i<vertices.size();i++)
		{
			Vertex_handle new_v = B.add_vertex ( Point(vertices[i][EVertexX],vertices[i][EVertexY],vertices[i][EVertexZ] ));
			new_v->id = i;
			for(int c=0;c<3;c++)
				new_v->color[c] = vertices[i][c+3];
			new_v->quality_imported() = vertices[i][EVertexQuality];
		}
		
		// add the facets
		bool bWarnings=false;
		for(int i=0;i<triangle_vertices.size();i++)
		{
			Halfedge_handle h=NULL;
			std::vector< std::size_t> newFacetIdx;
			for ( int k=0;k<3;k++ )
					newFacetIdx.push_back( triangle_vertices[i][k] );
			
			if (B.test_facet(newFacetIdx.begin(),newFacetIdx.end()))
				h = B.add_facet(newFacetIdx.begin(),newFacetIdx.end());
			else
			{
				if (!bWarnings)
				{
					//cout << "WARNING: following facet(s) violate the manifold constraint (will be ignored):";
					bWarnings=true;
					bFoundProblems=true;
				}
				cout << " " << i;
				cout.flush();
				
			}

			if (h!=NULL)
			{
				h->facet()->id = i;
			}
			
		}
		if (bWarnings) cout << endl;
		
		if (B.check_unconnected_vertices())
		{
			B.remove_unconnected_vertices();
			//cout << "WARNING: unconnected vertices exists. They will be removed" << endl;
		}
		B.end_surface();
	}
}; //Mesh_Loader
typedef Mesh_Loader<HalfedgeDS,Kernel> MeshLoader;


float readTypeFromStream(std::ifstream &is, std::string type, bool bBinary)
{
	if (bBinary==false)
	{
		float i;
		is >> i;
		return i;
	}
	if (type.compare("uchar")==0)
	{
		unsigned char i;
		is.read(reinterpret_cast < char * > (&i), sizeof(i));
		return i;
	}

	if (type.compare("int")==0)
	{
		int i;
		is.read(reinterpret_cast < char * > (&i), sizeof(i));
		return i;
	}

	if (type.compare("float")==0)
	{
		float i;
		is.read(reinterpret_cast < char * > (&i), sizeof(i));
		return i;
	}
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
Mesh::MeshReturnCodes Mesh::loadFormat ( const char *filename, bool rememberFilename )
{
	std::string token;
	float tmpValue;

	bool isCOFF=false;
	bool isOFF=false;	
	bool isPLY=false;	
	if (filename==NULL) return MeshFileNotFound;
	lock();	
	
#define DEBUG_LOAD(X)
	
	clear ( rememberFilename );
	//std::cout<<"CLEAR ENDED"<<std::endl;
	
	ifstream is;
	if ( rememberFilename )
	{
		loaded_filename = new char[strlen ( filename ) +1];
		strcpy ( loaded_filename,filename );
		
	}

	is.open(filename);
	if (!is.is_open()) 
	{
		cout << "could not open file." << endl;
		return 	MeshFileNotFound;
	}
	is >> token;

	// supported formats
	isPLY=(token.compare("PLY")==0) || (token.compare("ply")==0);
	isOFF=(token.compare("OFF")==0) || (token.compare("off")==0);
	isCOFF=(token.compare("COFF")==0) || (token.compare("coff")==0);

	// initialize mesh build structure and misc structures
	MeshLoader builder;
	std::vector<float>			newVertex;
	std::vector<int>			newVertexIdx;	
	std::vector<std::string>	newVertexAttribTypes;
	int noVertexAttribs = MeshLoader::EVertexNoAttribs;
	int iIgnoreIdx=MeshLoader::EVertexIgnore;	
	std::vector<int>			newTriangle;
	std::vector<std::string>	newFaceAttribTypes;	
	int							iNoVertices;
	int							iNoTriangles;
	
	for(int i=0;i<noVertexAttribs;i++) 
	{	
		newVertex.push_back(0.0f);
		newVertexIdx.push_back(i);	
	}
	for(int i=0;i<3;i++) newTriangle.push_back(-1);
	
	if(isOFF || isCOFF) 
	{
		is >> iNoVertices; is >> iNoTriangles; is >> tmpValue;
		
		cout << "* Loading " << token << " mesh: " << filename << " (" << iNoTriangles << " facets, " << iNoVertices << " vertices)";
		if (isCOFF) cout << " (with colors).";
		cout << endl;

		float normFactor=1.0f;
		bool isNormFactorSet=false;

		// process vertices
		for ( int i=0; i < iNoVertices; i++ ) {			
			for(int k=0;k<3;k++)
				is >> newVertex[k]; 
			if (isCOFF)
			{
				for(int k=3;k<6;k++)
					is >> newVertex[k]; 
				is >>tmpValue;

				if (!isNormFactorSet){
					if (tmpValue>1) normFactor=255.0f;
					isNormFactorSet = true;
				}
				for(int k=0;k<3;k++)
					newVertex[3 + k] /= normFactor;
				newVertex[MeshLoader::EVertexQuality] = (newVertex[3] + newVertex[4] + newVertex[5]) / 3;
			}
			builder.addVertex(newVertex);
			DEBUG_LOAD(
			cout << " - vertex " ;
			for(int i=0;i<6;i++) 
				cout << newVertex[i] << " ";
			cout << endl;
			)
				
		}

		// process triangles
		for ( int i=0; i < iNoTriangles; i++ ) {			
			int noVertsPerFacet;
			is >>noVertsPerFacet;			
			
			if (noVertsPerFacet != 3)
			{
				cout << "\nWARNING: only triangular meshes are supported - ignoring facet";
			}
			
			for(int k=0;k<noVertsPerFacet;k++)
			{
				if (k<3)
					is >> newTriangle[k]; 
				else
					is >> tmpValue;
					
			}
			// skip any other text until the next line
			char buff[256];
			is.getline (buff,256);

			DEBUG_LOAD(
			cout << " - triangle " ;
			for(int i=0;i<3;i++) 
				cout << newTriangle[i] << " ";
			cout << endl;
			)

			builder.addTriangle(newTriangle);
			
		}
	} // OFF
	
	if (isPLY)
	{
		std::string line;
		
		// by default mark all the vertex properties as ignored
		for(int i=0;i<noVertexAttribs;i++) newVertexIdx.push_back(iIgnoreIdx);	
		
		bool bSupportedFormat=false;
		bool bBinary=false;		
		
		int iCurrentPropType = 0; // 0 - not defined; 1 - vertices; 2 - facets;
		int iNoVertexAttribs=0; // vertex attrib index
		int iNoFaceAttribs=0; // facet attrib index		
		
		string sVertexAttribs;

		//process header
		while (getline(is,line))
		{
			std::stringstream ss(line);
			ss >> token; // first word
			if (token.compare("end_header")==0) break;

			if (token.compare("format")==0 && (line.find("1.0")!=string::npos)) 
			{
				ss >> token;
				if (token.compare("ascii")==0) 
				{
					bSupportedFormat=true;
					bBinary = false;
					continue;
				}
				else 
				if (token.compare("binary_little_endian")==0) 
				{
					bSupportedFormat=true;
					bBinary=true;
					continue;
				}
				{
					cout << "ERROR: unexpected property field:" << line <<  endl;
					bSupportedFormat = false;
					break;
				}
			}
			
			if (token.compare("element")==0) 
			{
				ss >> token;
				if (token.compare("vertex")==0)
				{
					ss >> iNoVertices;
					iCurrentPropType = 1;
					continue;
				}
				else if (token.compare("face")==0)				
				{
					ss >> iNoTriangles;
					iCurrentPropType = 2;
					continue;
				}
				else
				{
					cout << "ERROR: unexpected token" << token << " after element" << endl;
					break;
				}
			}
			
			if (token.compare("property")==0) 
			{
				if (iCurrentPropType==0) 
				{
					cout << "ERROR: unexpected property field:" << line <<  endl;
					bSupportedFormat = false;
					break; // we do not expect this type of record now
				}

				if (iCurrentPropType==1)  // vertices
				{
					string sPropDataType;
					ss >> sPropDataType; // property data type
					ss >> token;
					
					//save datatype
					newVertexAttribTypes.push_back(sPropDataType);
					
					//set appropriate index
					if (token.compare("x")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexX;
					if (token.compare("y")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexY;
					if (token.compare("z")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexZ;				
					if (token.compare("red")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexR;
					if (token.compare("green")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexG;
					if (token.compare("blue")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexB;
					if (token.compare("alpha")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexAlpha;
					if (token.compare("quality")==0) newVertexIdx[iNoVertexAttribs]=MeshLoader::EVertexQuality;

					sVertexAttribs = sVertexAttribs + " " + token;

					iNoVertexAttribs++;
					
				}
				if (iCurrentPropType==2)  // facets
				{
					ss >> token; 
					if (token.compare("list")!=0)
					{
						cout << "ERROR: unexpected property token " << token << endl;
						bSupportedFormat = false;
						break;
					}
					
					for(int i=0;i<2;i++)
					{
						ss >> token;
						newFaceAttribTypes.push_back(token);
						iNoFaceAttribs++;
					}
				}				

			}
		}
		if (iNoFaceAttribs!=2)
		{
			cout << "ERROR: property list not found for faces" << endl;
			bSupportedFormat = false;
		}
		if (iNoVertexAttribs<3)
		{
			cout << "ERROR: at least 3 properties expected for the vertices" << endl;
			bSupportedFormat = false;
		}
		
		if (bSupportedFormat)
		{
			cout << "* Loading PLY mesh: " << filename << " (" << iNoTriangles << " facets, " << iNoVertices << " vertices=[" << sVertexAttribs << "])" << endl;
			
			float normFactor=1.0f;
			bool isNormFactorSet=false;
			
			// process vertices
			for ( int i=0; i < iNoVertices; i++ ) {			
				for(int k=0;k<iNoVertexAttribs;k++) newVertex[newVertexIdx[k]] = readTypeFromStream(is,newVertexAttribTypes[k],bBinary); 
				builder.addVertex(newVertex);
				DEBUG_LOAD(
						   cout << " - vertex " ;
						   for(int i=0;i<6;i++) 
						   cout << newVertex[i] << " ";
						   cout << endl;
						   )
			}
			
			// process triangles
			for ( int i=0; i < iNoTriangles; i++ ) {			
				int noVertsPerFacet;
				noVertsPerFacet = readTypeFromStream(is,newFaceAttribTypes[0],bBinary);
				
				if (noVertsPerFacet != 3)
				{
					cout << "\nWARNING: only triangular meshes are supported - ignoring facet";
				}
				
				for(int k=0;k<noVertsPerFacet;k++)
				{
					if (k<3)
						newTriangle[k] = readTypeFromStream(is,newFaceAttribTypes[1],bBinary);
					else
						tmpValue = readTypeFromStream(is,newFaceAttribTypes[1],bBinary);
					
				}
				
				DEBUG_LOAD(
						   cout << " - triangle " ;
						   for(int i=0;i<3;i++) 
						   cout << newTriangle[i] << " ";
						   cout << endl;
						   )
				
				builder.addTriangle(newTriangle);
			}					
		}
	}

	p.delegate ( builder );		
	cout << "* Loaded: " << p.size_of_facets() << " facets, " << p.size_of_vertices() << " vertices and " << p.size_of_border_edges() << " border edges." << endl;
	
//	resetSimplexIndices();
	
	unlock();
	updateMeshData(true);
	if (builder.hasFoundProblems())
		return MeshNonManifold;
	else
		return MeshOK;		
}

// assumes that it has the mesh already loaded
//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::loadVectorField ( const char* filePattern, bool rememberFilename ) {
	
	if ((filePattern==NULL) || (strlen ( filePattern ) <2) )
	{
		cout << "No vectorfield defined." << endl;
		return true;
	}
	cout << "* Loading vectorfield: <" << filePattern << ">" ; cout.flush();
	
	std::ifstream file ( filePattern );
	if ( !file )
	{
		std::cerr << "Error: Unable to read '" << filePattern << "'" << std::endl;
		return false;
	}
	
	if ( rememberFilename )
	{
		loaded_vect_filename = new char[strlen ( filePattern ) +1];
		strcpy ( loaded_vect_filename,filePattern );
	}
	lock();	
	int id=0;
	for ( Vertex_iterator vi= p.vertices_begin(); vi != p.vertices_end();vi++ )
	{
		file >> vi->motion_vec;
		vi->id = id++;
		//		cout << "read " << vi->motion_vec << endl;
	}
	unlock();
	cout << " OK" << endl;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::saveVectorField ( const char* filePattern )
{
	std::ofstream file ( filePattern );
	if ( !file )
	{
		std::cerr << "Error: Unable to save '" << filePattern << "'" << std::endl;
		return false;
	}
	cout << "* Saving vectorfield: "; cout.flush();
	for ( Vertex_iterator vi= p.vertices_begin(); vi != p.vertices_end();vi++ )
	{
		file << vi->motion_vec << endl;
		//		cout << "written " << vi->motion_vec << endl;
	}
	cout << filePattern << " OK" << endl;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::saveVertexIndices ( const char* filePattern )
{
	std::ofstream file ( filePattern );
	if ( !file )
	{
		std::cerr << "Error: Unable to save '" << filePattern << "'" << std::endl;
		return false;
	}
	cout << "* Saving vertex indices: "; cout.flush();
	for ( Vertex_iterator vi= p.vertices_begin(); vi != p.vertices_end();vi++ )
	{
		file << vi->id << endl;
	}
	cout << filePattern << " OK" << endl;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::dilate ( double delta ) {
	lock();
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		vi->point() = vi->point() + vi->normal() *delta;//vert->getMeanCurvatureFlow().Norm();
	unlock();
	updateMeshData();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//assumes that the quality has been copied in quality_prev 
void Mesh::computeVertexConvolution(Vertex& v, float stddevratio) { 
	float stddev=edge_avg*stddevratio;
	float weights=0;

#define CONVOLVE_GEODESIC_RINGS 0
	if (CONVOLVE_GEODESIC_RINGS)
	{
		std::vector< std::pair<Vertex*,float> > elems = getGeodesicNeighbourhood(v,2*stddev,true);
		v.quality() = 0;
		for(int i=0;i<elems.size();i++) {
			float value=elems[i].first->quality_prev();
			float w= GAUSSIAN(stddev,elems[i].second);
			v.quality()+=value*w;
			weights+=w;
		}
	}
	else
	{
		HV_circulator vi = v.vertex_begin();
		
		float w=GAUSSIAN(stddev,0);
		v.quality()=w*v.quality_prev();
		weights=w;
		if ( vi!=NULL )
			do
			{
				float dist=v_norm(vi->prev()->vertex()->point() - v.point());
				w=GAUSSIAN(stddev,dist);
				v.quality()+=vi->prev()->vertex()->quality_prev()*w;
				weights+=w;
			}
			while ( ++vi != v.vertex_begin() );	
		//	v.vh_dist=rand()*1.0f/RAND_MAX;

	}
	if (weights!=0) v.quality()/=weights;		
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// std_dev_ratio - ratio of the avg_edge
void Mesh::convolve(double std_dev_ratio, bool add_to_history, bool print_stats) {
	lock();
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		vi->quality_prev()=vi->quality();

	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		computeVertexConvolution(*vi,std_dev_ratio);
		if (add_to_history) {
			float quality_prev=vi->quality_prev();
			if (vi->values.size() > 1) quality_prev= vi->values[vi->values.size()-2];
			vi->values.push_back(vi->quality()); // the quality
			vi->values.push_back((vi->quality()-quality_prev)); // the difference in adjacent qualities
		}
			
	}

	setMeshQuality(Qual_No_Computation);

	if (print_stats) {
		float lower_qual, upper_qual;	
		computeQualityPercentileBoundaries(lower_qual,upper_qual,0.01);
		cout << "values btw:" << lower_qual << " and " << upper_qual << endl;	
	}
	
	unlock();
	updateMeshData(false,false);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::smooth ( double delta, int mode, bool only_visible )
{ 
	// 0 - both components ; 
	// 1 - tangetial; 
	// 2 - normal; 
	// 3 - second order; 
	// 4 - combined; 
	// 5 - tangential only if bigger than normal
	// 6 - both components - laplacian_avg
	Vector displacement;
	assert ( ( mode>=0 ) && ( mode<=6 ) );
	lock();
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		if (vi->getVisibility()==false) continue;
		if ( mode==0 )
			displacement = vi->laplacian() *delta;//vert->getMeanCurvatureFlow().Norm();
		else if ( mode==1 )
			displacement = computeVectorComponent ( vi->normal(),vi->laplacian(),0 ) *delta;
		else
			if ( mode==2 )
				displacement = computeVectorComponent ( vi->normal(),vi->laplacian(),1 ) *delta;
		if ( mode==3 )
			displacement = vi->laplacian_deriv() *delta;
		if ( mode==4 )
			displacement = vi->laplacian() *delta - vi->laplacian_deriv() *delta;
		if ( mode==5 ) {
			Vector d_tan = computeVectorComponent ( vi->normal(),vi->laplacian(),0 ) *delta;
			Vector d_norm = computeVectorComponent ( vi->normal(),vi->laplacian(),1 ) *delta;
			if (v_norm(d_tan)>2*v_norm(d_norm)) 
				displacement = d_tan;
			else
				displacement = Vector(0,0,0);
		}
		if ( mode==6 )
			displacement = vi->laplacian() *delta - vi->normal()*laplacian_avg*delta;
		
		vi->move ( displacement );
	}
	unlock();
	updateMeshData(false,false);
}


void Mesh::generateRandomColors() {
	lock();
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		for(int j=0;j<3;j++)
			vi->color[j] = ( rand() *1.0f/RAND_MAX );
	}
	unlock();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
float Mesh::getVolume(bool computeNormal) {
	
	float vol=0;
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ ) {	
		//		if (computeNormal) fi->computeNormal();
		
		vol+= (fi->center()-Point(0,0,0))*CGAL::cross_product(fi->halfedge()->next()->vertex()->point() - fi->halfedge()->vertex()->point(),fi->halfedge()->prev()->vertex()->point() - fi->halfedge()->vertex()->point());
	}
	return vol/6;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::bilateralSmooth ( double sigma_c, double sigma_s) {
	lock();
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		
		HV_circulator h = vi->vertex_begin();
		Vector diffusion=Vector ( 0,0,0 );
		float sum=0;
		float normalizer = 0;
		do {
			Vector tmpVec = h->prev()->vertex()->point() - vi->point();
			float t = v_norm(tmpVec);
			float h = vi->normal()*tmpVec;
			float w_c = exp(-t*t/(2*sigma_c*sigma_c));
			float w_s = exp(-h*h/(2*sigma_s*sigma_s));
			sum+=(w_c*w_c)*h;
			normalizer+=w_c*w_s;
		} while ( ++h != vi->vertex_begin() );
		vi->tmp_float=sum/normalizer;
	}
	float volPrev = getVolume(false);
	
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		vi->move (vi->tmp_float*vi->normal());
	}
	
	float volNow = getVolume(true);
	float ratio = pow(volPrev/volNow,1.0f/3.0f);
	
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) 
		vi->point() = Point(0,0,0) + ((vi->point() - Point(0,0,0))*ratio);
	unlock();
	
	updateMeshData();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::evolve ( float sign )
{
	double dimming=1;
	lock();
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		//		cout << "d:" << vi->delta << " n:" << vi->normal() << " c:" << vi->laplacian() << endl;
		if ( isnan ( v_norm ( vi->delta ) ) ) cout << "ISNAN delta" << endl;
		else
		{
			//			vi->prev_delta = vi->delta;
			vi->point() =vi->point() + vi->motion_vec * sign * dimming;
		}
	}
	unlock();
	updateMeshData();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// mode - 0 - difussion of the vector field; 1 - difussion of the transported mesh
void Mesh::diffuse ( float amount, int mode )
{
	
	std::vector<Vector> local_deltas ( p.size_of_vertices() );
	int i_delta=0;
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		//		if (vi->getVisibility()==false) continue;		
		HV_circulator h = vi->vertex_begin();
		Vector diffusion=Vector ( 0,0,0 );
		int order=0;
		do
		{
			if ( mode==1 )
				diffusion = diffusion + ( ( h->prev()->vertex()->point() + h->prev()->vertex()->delta ) - ( vi->point() + vi->delta ) );
			else
				diffusion = diffusion + ( h->prev()->vertex()->delta - vi->delta );
			order++;
		}
		while ( ++h != vi->vertex_begin() );
		diffusion = diffusion/order;
		local_deltas[i_delta++] = vi->delta + amount*diffusion;
	}
	
	// EVOLVE THE MESH
	i_delta = 0;
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		vi->delta = local_deltas[i_delta++];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::updateMeshData(bool bComputeCurvature, bool bComputeNormals) {
	lock();
	p.normalize_border();
	
	fRobustNormalMaxDistance = 5;
	
	if (bComputeNormals)
	{
		// compute triangle normals
		for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ )
			fi->computeNormal();
		
		// compute vertex normal
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) 
			computeVertexNormal( *vi );
		
		// compute robust vertex normal		
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) 
			computeVertexRobustNormal( *vi, fRobustNormalMaxDistance );		
	}

	// compute laplacians
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		vi->border()=false;
		computeVertexLaplacian( *vi );
	}
	
	computeMeshStats();	
		
	// compute curvature and laplacian derivative
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		if (bComputeCurvature) computeVertexCurvature( *vi ); 
		computeVertexLaplacianDeriv ( *vi );
	}

	// set border edges	
	for (Halfedge_iterator hi = p.border_halfedges_begin(); hi!= p.halfedges_end(); hi++) {
		hi->vertex()->border()=true;
	}
	
	computeMeshStats();
	
	setMeshQuality(qual_mode);
	
	is_search_init=false;
	Neighbor_search_tree tmpSearch;
	//search_tree = tmpSearch;
	search_tree.clear();
	is_mapping_init=false;
	id_mapping.clear();
	//vertex_mapping.clear();
	unlock();
//	computeMeshStats();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
double Mesh::oppositeAngle ( Vertex::Halfedge_handle h )
{
	CGAL_precondition ( h->facet()->is_triangle() );
	return v_angle ( h->vertex()->point() - h->next()->vertex()->point(), h->prev()->vertex()->point() - h->next()->vertex()->point() );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeMeshStats() {
	lock();
	int tmpCounter;
	
	//area statistics
	double tmpArea;
	area_avg=0;
	area_min=MAXFLOAT;
	area_max=0;
	tmpCounter=0;
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ ) {
		if (fi->halfedge()->vertex()->getVisibility()==false) continue;
		tmpArea = fi->area();
		area_avg+= tmpArea;
		area_min = std::min ( area_min,tmpArea );
		area_max = std::max ( area_max,tmpArea );
		tmpCounter++;
	}
	area_avg/= tmpCounter;
	
	double tmpEdge;
	edge_avg=0;
	edge_min=MAXFLOAT;
	edge_max=0;
	tmpCounter=0;
	for ( Edge_iterator ei= p.edges_begin(); ei!=p.edges_end(); ei++ ) {
		if (ei->vertex()->getVisibility()==false) continue;
		if (ei->prev()->vertex()->getVisibility()==false) continue;
		const Vector tmpLength=ei->vertex()->point() - ei->prev()->vertex()->point();
		tmpEdge = v_norm(tmpLength);
		edge_avg+= tmpEdge;
		edge_min = std::min ( edge_min,tmpEdge );
		edge_max = std::max ( edge_max,tmpEdge );
		tmpCounter++;
	}
	edge_avg/= tmpCounter;
	

	laplacian_avg=0;
	laplacian_min=0;
	laplacian_max=0;
	tmpCounter=0;
	for ( Vertex_iterator vi = p.vertices_begin(); vi !=p.vertices_end(); vi++ )
	{
		double tmpNorm = v_norm(vi->laplacian());
		if (vi->laplacian()*vi->normal()<0.0f) tmpNorm=-tmpNorm;
		
		laplacian_avg+= tmpNorm;
		laplacian_min = std::min ( laplacian_min, tmpNorm );
		laplacian_max = std::max ( laplacian_max, tmpNorm );
		tmpCounter++;
	}
	laplacian_avg/= tmpCounter;
	
	
#if COMPUTE_CURVATURE_STATS==1
	double tmpCurv;
	curvature_avg=0;
	curvature_min=1;
	curvature_max=-1;
	tmpCounter=0;
	for ( Vertex_iterator vi = p.vertices_begin(); vi !=p.vertices_end(); vi++ )
	{
		tmpCurv = vi->mean_curvature();
		curvature_avg+= abs(tmpCurv);
		curvature_min = std::min ( curvature_min,tmpCurv );
		curvature_max = std::max ( curvature_max,tmpCurv );
		tmpCounter++;
	}
	curvature_avg/= tmpCounter;
	
	curvature_std_dev=0;
	for ( Vertex_iterator vi = p.vertices_begin(); vi !=p.vertices_end(); vi++ )
	{
		tmpCurv = abs(vi->mean_curvature()) - curvature_avg;
		curvature_std_dev += tmpCurv*tmpCurv;
	}
	if ( tmpCounter>1 ) curvature_std_dev = std::sqrt ( abs(curvature_std_dev)/ ( tmpCounter-1 ) );
	
#endif
	unlock();	
}

void Mesh::resetVertexWeights()
{
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		vi->weight = 1;
		vi->prev_delta = Vector ( 0,0,0 );
	}
#if MVSTEREO_FACETS==1	
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ )
	{
		fi->weight = 1;
	}
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexWeights()
{
#if MVSTEREO_FACETS==1
	double tmpArea;
	double tmpEdge;
	
	computeMeshStats();
	
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ )
	{
		//		fi->weight = (float) (fi->area()-area_min)/(area_max-area_min);
		fi->weight = ( float ) ( fi->edgeStatistics ( 1 )-edge_min ) / ( edge_max-edge_min );
		fi->weight_priors = fi->weight;
	}
	
	// update vertex weights info
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		vi->weight =0;
		int tmpCount=0;
		HV_circulator h = vi->vertex_begin();
		do
		{
			tmpCount++;
			if ( !h->is_border() ) vi->weight += h->facet()->weight;
		}
		while ( ++h != vi->vertex_begin() );
		vi->weight/=tmpCount;
	}
#endif	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexPriorWeights()
{
	double tmpArea;
	double tmpEdge;
	
	computeMeshStats();
#if MVSTEREO_FACETS==1
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ )
	{
		fi->weight_priors = ( float ) ( fi->edgeStatistics ( 1 )-edge_min ) / ( edge_max-edge_min );
		
	}
#endif	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// mode : 0 - min, 1 - avg, 2- max
float Mesh::computeVertexStatistics ( Vertex& v, int mode )
{
	// CFL depends on the incident edges
	CGAL_precondition ( ( mode>=0 ) && ( mode <=2 ) );
	if (v.vertex_degree()==0) return 0;
	HV_circulator h = v.vertex_begin();
	float edge_stats = edge_size ( h );
	int no_h=1;
	
	do
	{
		if ( mode==0 ) edge_stats = std::min ( ( float ) edge_size ( h ),edge_stats );
		if ( mode==1 ) edge_stats += edge_size ( h );
		if ( mode==2 ) edge_stats = std::max ( ( float ) edge_size ( h ),edge_stats );
		no_h++;
	}
	while ( ++h != v.vertex_begin() );
	if ( mode==1 ) edge_stats= ( edge_stats/no_h );
	return edge_stats;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// mode : 0 - tangetial; 1 - across the normal
Vector Mesh::computeVectorComponent ( Vector n, Vector v, int mode )
{
	CGAL_precondition ( ( mode>=0 ) && ( mode <2 ) );
	Kernel::Plane_3 pl;
	Vector across_normal=n*(n*v);
	
	if (mode==1)
		return across_normal;
	else
		return v - across_normal;
	
	if ( mode==0 ) // across the normal
		pl = Kernel::Plane_3 ( CGAL::ORIGIN,n );
	else
	{// tangetial
		Vector c=CGAL::cross_product ( n,v );
		pl = Kernel::Plane_3 ( CGAL::ORIGIN,CGAL::cross_product ( n,c ) );
	}
	
	Vector result = pl.projection ( CGAL::ORIGIN+v ) - CGAL::ORIGIN;
	if ( isnan ( v_norm ( result ) ) ) result = Vector ( 0,0,0 );
	return result;
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexNormal ( Vertex& v )
{
	
	Vertex::Normal_3 normal = CGAL::NULL_VECTOR;
	HV_circulator c = v.vertex_begin();
	HV_circulator d = c;
	float w=0;
	CGAL_For_all ( c, d )
	{
		if ( ! c->is_border() ) {
			w = v_norm(v.point() - c->facet()->center());
			if (w !=0) w = 1/w;
			//w = c->facet()->area();
			normal = normal + c->facet()->normal()*w;
		}
	}
	if ( v_norm ( normal ) !=0 )
		normal = normal / v_norm ( normal ); 
	
	v.tmp_normal() = normal;
	// only temporarily set here, it will be set in normal() when computing the robust measure
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexRobustNormal ( Vertex& v, float maxGeodesic )
{
#ifdef USE_NORMAL_GEODESICS
	vector<pair<Vertex*,float> > neighs = getGeodesicNeighbourhood(v,maxGeodesic,true);	
	vector<pair<Vertex*,float> >::iterator n_it;
#else
	vector<pair<Vertex*,int> > neighs = getRingNeighbourhood(v,(int)maxGeodesic,true);
	vector<pair<Vertex*,int> >::iterator n_it;	
#endif	

	Vertex::Normal_3 normal = CGAL::NULL_VECTOR;	
	for(n_it=neighs.begin(); n_it!=neighs.end();n_it++) 
	{
		Vertex* neigh_v = n_it->first;
		//float w = n_it->second  / maxGeodesic;
		normal = normal + neigh_v->tmp_normal();//*w;
	}
	if ( v_norm ( normal ) !=0 )
		normal = normal / v_norm ( normal );

	v.normal() = normal;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::setMeshQuality(QualityComputationMethod qual) {
	qual_mode = qual;
/*
 #ifdef WITH_CGAL_BLAS
	if ( (qual_mode==Qual_Gaussian_Curv_Deriv) || (qual_mode==Qual_Mean_Curv_Deriv) || (qual_mode==Qual_Color_Deriv)) {
		setMeshQualityViaLinearSystem(qual);
		return;
	}
#endif
*/ 
	
	if (qual_mode!=Qual_No_Computation) {
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		{
			computeVertexQuality(*vi,qual_mode);
			if (isnan(vi->quality()))
			{
				cout << "\r Error (NAN) while computing vertex quality type=" << qual << " for v=" << vi->id << "of degree=" << vi->vertex_degree() << endl;
				vi->quality() = 0;
			}

		}
			

		// if computing gradients, set the quality from the norm
		if ((qual_mode==Qual_Color_Deriv) || (qual_mode==Qual_Mean_Curv_Deriv) ||
			(qual_mode==Qual_Gaussian_Curv_Deriv) || (qual_mode==Qual_Quality_Deriv))
			for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
			{	
				vi->quality()=v_norm(vi->qual_vect);
				if (isnan(vi->quality()))
				{
					cout << "\r Error (NAN) while computing vertex quality type=" << qual << " for v=" << vi->id << "of degree=" << vi->vertex_degree() << endl;
					vi->quality() = 0;
				}				
			}
	}

	
}

void Mesh::setMeshQualityViaLinearSystem(QualityComputationMethod qual) {
#ifdef WITH_CGAL_BLAS
	if ( (qual_mode==Qual_Gaussian_Curv_Deriv) || (qual_mode==Qual_Mean_Curv_Deriv) || (qual_mode==Qual_Color_Deriv)) {
		int no_elems  = 0;

		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) 
			no_elems += vi->vertex_degree();


		Linear_system_matrix solve_matrix(no_elems,p.size_of_vertices()*3);
		Linear_system_vector solve_vector(no_elems);
		Linear_system_solver solver;
		cout << "setting up matrix of size " << solve_matrix.number_of_rows() << "x" << solve_matrix.number_of_columns() << endl;
			
		int i=-1;
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
			HV_circulator c = vi->vertex_begin();
			HV_circulator d = c;
			
			Vector deriv=Vector(0,0,0); 
			CGAL_For_all ( c, d )
			{
				Vector direction = c->opposite()->vertex()->point() - vi->point();
				float dir_norm = v_norm(direction);
				direction=v_normalized(direction);
				float local_deriv = computeVertexDirectionalGradient(*vi,*(c->opposite()->vertex()),qual);
				
				i++;
				int j=c->opposite()->vertex()->id;
				//cout << "setting elements" << i << "," << j << endl;
				solve_matrix.set(i,3*j+0,direction.x());
				solve_matrix.set(i,3*j+1,direction.y());
				solve_matrix.set(i,3*j+2,direction.z());
				solve_vector.set(i,local_deriv);
				
			} //neighbourhood loop
		} // all vertices loop
		cout << "done setting all the elements" << endl;
		float condition = solver.solve(solve_matrix,solve_vector);
		cout << "done solving the system" << endl;

		i=0;
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++,i++ ) {
			vi->qual_vect=Vector(solve_vector(3*i+0),solve_vector(3*i+1),solve_vector(3*i+2));
			vi->quality()=v_norm(vi->qual_vect);
			
		}
		
		// need to set the function now
	} // if qual mode is deriv
#endif
}

/*this is run within the updateMeshData() */
void Mesh::computeVertexQuality(Vertex& v, QualityComputationMethod qual) {
	switch (qual) {
		case Qual_Imported:
			v.quality()=v.quality_imported();
			break;			
		case Qual_Color:
			v.quality()=(v.color[0] + v.color[1] + v.color[2])/3;
			break;			
		case Qual_Mean_Curv:
			v.quality()=v.mean_curvature();
			break;
		case Qual_Gaussian_Curv:
			v.quality()=v.gaussian_curvature();			
			break;
		case Qual_Color_Deriv:
		case Qual_Mean_Curv_Deriv:
		case Qual_Gaussian_Curv_Deriv:
		case Qual_Quality_Deriv:
			v.qual_vect=computeVertexGradient(v,qual);
			break;
	} 
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
float Mesh::computeVertexDirectionalGradient ( Vertex& v, Vertex& dir_u, QualityComputationMethod qual) {
	
	float local_deriv = 0.0f;
	if (qual==Qual_Color_Deriv) {
		for (int i=0;i<3;i++) local_deriv+=(dir_u.color[i] - v.color[i]);
		local_deriv/=3;			
	}
	else if (qual==Qual_Mean_Curv_Deriv) {
		local_deriv=(dir_u.mean_curvature() - v.mean_curvature());			
	}
	else if (qual==Qual_Gaussian_Curv_Deriv) {
		local_deriv=(dir_u.gaussian_curvature() - v.gaussian_curvature());			
	}
	else if (qual==Qual_Quality_Deriv) {
		local_deriv=(dir_u.quality() - v.quality());			
	}
	else {
		cout << " INVALID qual in computeVertexGradient" << endl;
	}

	return local_deriv;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
Vector Mesh::computeVertexGradient ( Vertex& v, QualityComputationMethod qual)
{
#if WITH_CGAL_BLAS
	return computeVertexGradientViaLinearSystem(v,qual);
#endif	
	
	HV_circulator c = v.vertex_begin();
	HV_circulator d = c;
	
	Vector deriv=Vector(0,0,0); 
	
	CGAL_For_all ( c, d )
	{
		Vector direction = c->prev()->vertex()->point() - v.point();
		float dir_norm = v_norm(direction);
		//		cout << "edge=" << edge_avg << " norm=" << v_norm(direction) << " guassian = " << GAUSSIAN(edge_avg,v_norm(direction)) << endl;
		//direction=v_normalized(direction)*GAUSSIAN(edge_avg,v_norm(direction));
		direction=v_normalized(direction);
		float local_deriv = computeVertexDirectionalGradient(v,*(c->opposite()->vertex()),qual);

		if (dir_norm!=0) 
			deriv = deriv + direction*local_deriv;//dir_norm;
		
	}
	if (v.vertex_degree()>0) deriv=deriv/v.vertex_degree();
	return deriv;
}





//////////////////////////////////////////////////////////////////////////////////////////////////////
Vector Mesh::computeVertexGradientViaLinearSystem ( Vertex& v, QualityComputationMethod qual)
{
#ifdef WITH_CGAL_BLAS	
	int no_elems  = v.vertex_degree()+1;
	
	CGAL::Lapack_matrix solve_matrix(no_elems,3);
	CGAL::Lapack_vector solve_vector(no_elems);
	CGAL::Lapack_svd solver;
	
	HV_circulator c = v.vertex_begin();
	HV_circulator d = c;
	
	Vector deriv=Vector(0,0,0.000000001f); 
	if(no_elems==1) return deriv; // isolated vertex
	int i=-1;
	CGAL_For_all ( c, d )
	{
		i++;
		Vector direction = c->prev()->vertex()->point() - v.point();
		float dir_norm = v_norm(direction);
		direction=v_normalized(direction);
		float local_deriv = computeVertexDirectionalGradient(v,*(c->opposite()->vertex()),qual);//dir_norm;

		solve_matrix.set(i,0,direction.x());
		solve_matrix.set(i,1,direction.y());
		solve_matrix.set(i,2,direction.z());
		solve_vector.set(i,local_deriv);
		
	}
	i++;
	solve_matrix.set(i,0,v.normal().x());
	solve_matrix.set(i,1,v.normal().y());
	solve_matrix.set(i,2,v.normal().z());
	solve_vector.set(i,0);
	
//	deriv=deriv/v.vertex_degree();
	float condition = solver.solve(solve_matrix,solve_vector);
	deriv = Vector(solve_vector(0),solve_vector(1), solve_vector(2));
	//printf("%.1f ",condition);
	//printf("angle btw deriv and normal is %f\n",v_angle(deriv,v.normal()));
	if (condition>50) 
		return Vector(0,0,0.00000001f);
	else 
		return deriv;
#endif
}




//////////////////////////////////////////////////////////////////////////////////////////////////////
// Precondition: Assumes that the gradient is already computed in quality(), dir_1, dir_2 are normalized
float Mesh::computeVertex2ndPartialDeriv( Vertex& v, Vector &dir_1, Vector &dir_2) {
	
	HV_circulator c = v.vertex_begin();
	HV_circulator d = c;
	
	Vector deriv=Vector(0,0,0); 
	
	CGAL_For_all ( c, d )
	{
		Vector direction = c->prev()->vertex()->point() - v.point();
		direction=v_normalized(direction);

		float local_deriv=(c->opposite()->vertex()->qual_vect*dir_1 - v.qual_vect*dir_1);

		deriv = deriv + direction*local_deriv;
	}

	return deriv*dir_2;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Precondition: Assumes that the gradient is already computed in quality()
// Uses the the ratio of the eigenvalues as described in the SIFT descriptor (Lowe IJCV 2004)
bool Mesh::isCorner(Vertex &v, float ratio) {
	Vector dir_1 = v.vertex_begin()->prev()->vertex()->point() - v.point();
	Vector dir_2 = CGAL::cross_product(dir_1,v.normal());
	dir_1 = v_normalized(dir_1);
	dir_2 = v_normalized(dir_2);

	float dxx = computeVertex2ndPartialDeriv(v,dir_1,dir_1);
	float dxy = computeVertex2ndPartialDeriv(v,dir_1,dir_2);
	float dyy = computeVertex2ndPartialDeriv(v,dir_2,dir_2);
	
	float trace = dxx + dxy;
	float det = dxx*dyy - dxy*dxy;
	if (det<0) return false;

	return (trace*trace/det) < (ratio+1.0)*(ratio+1.0)/ratio;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexLaplacianDeriv ( Vertex& v )
{
	std::size_t order = 0;
	HV_circulator vi = v.vertex_begin();
	v.laplacian_deriv() = Vector ( 0,0,0 );
	
	if ( vi!=NULL )
	{
		do
		{
			/*			if (vi->is_border_edge()) {
			 v.laplacian_deriv()=Vector(0,0,0);
			 return;
			 }
			 */			
			++order;
			v.laplacian_deriv() = v.laplacian_deriv() + ( vi->prev()->vertex()->laplacian()-v.laplacian() );
		}
		while ( ++vi != v.vertex_begin() );
		v.laplacian_deriv() = v.laplacian_deriv() / order;
	}
	
	//	cout << "curv=" << v.laplacian() << " and curv_deriv=" << v.laplacian_deriv() << endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
float Mesh::getVertexMeanCurvature ( Vertex& v ) {
	float edge_avg = computeVertexStatistics(v,1)/2;
	float result = v_norm(v.laplacian()) / edge_avg;
	if ( v.laplacian() * v.normal()  < 0) {
		result = (-1)*result;
	}
	return result;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexCurvature( Vertex& v) {
	
	float mean_curv, gaussian_curv;
	// curvature methods 1 - meyer et al.; 2 - dong et al.; 3- Bricolaje Maison

	if (curv_comp_method==Curv_Meyer) { //Mark Meyer
		float A_mixed=0;
		float a_1, a_2, a_3; // numbered based on nexts fo the vertex -1
		Vector e12, e23, e31;
		float n_e12, n_e23, n_e31; //cache the norm of the edges
		
		Vector mean_curv_normal = Vector(0,0,0);
		//mean_curv = 0;
		gaussian_curv = 2*PI;
		HV_circulator vi = v.vertex_begin();
		if ( vi!=NULL ) {
			do {
				e12 = vi->next()->vertex()->point() - vi->vertex()->point();
				e23 = vi->prev()->vertex()->point() - vi->next()->vertex()->point();
				e31 = vi->vertex()->point() - vi->prev()->vertex()->point();
				
				n_e12 = v_norm(e12);
				n_e23 = v_norm(e23);
				n_e31 = v_norm(e31);
				
				a_1 = v_angle(e12,-e31);
				a_2 = v_angle((-e12),e23);
				a_3 = PI - a_1 - a_2;
				
				// A_mixed
				if ((a_1 > PI/2) || (a_2 > PI/2) || (a_3 > PI/2)) { //obtuse
					if (a_1> PI/2)
						A_mixed += (v_norm(CGAL::cross_product((-e12),e23))/2)/2;
					else
						A_mixed += (v_norm(CGAL::cross_product((-e12),e23))/2)/4;
					
				}
				else
					A_mixed+= (SQUARED(n_e12)*(1.0/tan(a_3)) + SQUARED(n_e31)*(1.0/tan(a_2)))/8;
				
				mean_curv_normal = mean_curv_normal + (-e12)*(1.0/tan(a_3)) + e31*(1.0/tan(a_2));
				gaussian_curv-= a_1;
			} while ( ++vi != v.vertex_begin() );
			if (A_mixed < 1e-20)  {
				mean_curv_normal = Vector(0,0,0);
				gaussian_curv = 0;
			}
			else {
				mean_curv_normal = mean_curv_normal / (2*A_mixed);
				gaussian_curv=std::max(0.0f,gaussian_curv) / A_mixed;
			}
			
		} //if not NULL
		mean_curv = v_norm(mean_curv_normal)/2;
		if (mean_curv_normal*v.normal()>0)
			mean_curv = -mean_curv;
	} else if (curv_comp_method==Curv_Dong) { //Dong et al.

		Vertex *vi;
		Vector vi_vect,ti;
		float kn_ti;
		
		float max_kn=-MAXFLOAT;
		float min_kn=MAXFLOAT;		
		Vector axis1; //system of coordinates;

		//	curv_comp_neigh_rings

		float geodesicDist = max(edge_avg*1.2f,(double)getSurfacePercentageRadius(curv_comp_neigh_rings));
//			int normalizedRings = max(2,(int)round(sqrt((1/100.0)*area_avg*p.size_of_facets()/(4*PI))/edge_avg));
		vector<pair<Vertex*,int> > neighs = getRingNeighbourhood(v,curv_comp_neigh_rings,false);
//		vector<pair<Vertex*,float> > neighs = getGeodesicNeighbourhood(v,geodesicDist,false);			
		
//			cout << "v=" << v.id << " neighs=" << neighs.size() << " for g=" << geodesicDist << endl;
		for(int i=0;i<neighs.size();i++) 
        {
			vi=neighs[i].first;
			vi_vect = vi->point() - v.point();
			ti = vi_vect - (vi_vect*v.normal())*v.normal();
			ti = v_normalized(ti);
			kn_ti = - (vi_vect*(vi->normal() - v.normal())) / (vi_vect*vi_vect);
			//caching the results
			vi->delta_tmp = ti;
			vi->delta_normalization= kn_ti;
			//compute max
			if (max_kn<kn_ti) {
				max_kn = kn_ti;
				axis1=ti;
			}
			if (min_kn>kn_ti) {
				min_kn = kn_ti;
			}
			
		}
		if (neighs.size()==0)
		{
			max_kn = 0; min_kn=0;
		}
		
		float a11 = 0; float a12 = 0; float a21 = 0; float a22 = 0; float a13 = 0; float a23 = 0;
		float theta_i;

		float a,b,c;
		a = max_kn;
		for(int i=0;i<neighs.size();i++) 
        {
			vi=neighs[i].first;
			//restoring the results
			ti = vi->delta_tmp;
			kn_ti = vi->delta_normalization;
			//compute
			theta_i = v_angle(axis1,ti);
			float cos_theta = cos(theta_i);
			float sin_theta = sin(theta_i);				
			
			a11 += cos_theta * cos_theta * sin_theta * sin_theta;
			a12 += cos_theta * sin_theta * sin_theta * sin_theta;
			a22 += sin_theta * sin_theta * sin_theta * sin_theta;
			a13 += (kn_ti - a * cos_theta * cos_theta) * cos_theta * sin_theta;
			a23 += (kn_ti - a * cos_theta * cos_theta) * sin_theta * sin_theta;
		}
		a21 = a12;
		
		b = (a13 * a22 - a23 * a12) / (a11 * a22 - a12 * a12);
		c = (a11 * a23 - a12 * a13) / (a11 * a22 - a12 * a12);

		if (isnan(b)) b = 0;
		if (isnan(c)) c = 0;
		
		if (neighs.size()==0)
		{
			a=0; b=0; c=0;
		}
	
		gaussian_curv= a*c - SQUARED(b)/4;
		mean_curv  = (a+c)/2;
		
/*		max_kn = std::min(10.0f,max_kn);
		min_kn = std::max(-10.0f,min_kn);		

		mean_curv  = (max_kn+min_kn)/2;
		gaussian_curv  = max_kn*min_kn;
*/
	} else if (curv_comp_method==Curv_Approx) { //Approx
		
		float edge_avg = computeVertexStatistics(v,2);
		mean_curv= std::abs(v_norm(v.laplacian()) / edge_avg);
		if (v.laplacian()*v.normal()<0) mean_curv=-mean_curv;
		gaussian_curv =2*PI;
		/*	HV_circulator vi = v.vertex_begin();
		 do {
		 gaussian_curv-=v_angle(vi->next()->vertex()->point() - vi->vertex()->point(),vi->prev()->vertex()->point()-vi->vertex()->point());
		 } while ( ++vi != v.vertex_begin() );
		 
		 gaussian_curv=std::max(0.0f,gaussian_curv);
		 */
	} else if(curv_comp_method == Geo_Euclid_Ratio) { // Look in a bigger ring
		// of size "curv_geo_size"
		Vertex_handle vi,vj; 
		vector<Vertex_handle> patchVerts; 
		vector<float> patchGeo; 
		vector<float> patchEuclid; 
		int numNeigh=0, ifind=0;
		float curGeo=0, curEdge=0;  
		vector<Vertex_handle>::iterator vfind; 
		float sumGeo=0, sumEuclid=0, sumRatio=0;
		patchVerts.clear(); patchGeo.clear(); patchEuclid.clear(); 
		patchVerts.push_back(&v); patchGeo.push_back(0); patchEuclid.push_back(0); 
		
		do { 
			vi = patchVerts[numNeigh];
			HV_circulator hv = vi->vertex_begin(); 
			//curGeo += edge_avg; 
			curGeo = patchGeo[numNeigh]+1; 
			numNeigh++;
			do{
				vj = hv->opposite()->vertex();
				curEdge =  v_norm(vi->point() - vj->point());
				
				//curGeo = patchGeo[numNeigh]+curEdge;
				if(curGeo > curv_geo_size) continue; 
				
				vfind = find(patchVerts.begin(), patchVerts.end(), vj); 
				if(vfind != patchVerts.end()) { 
					ifind = std::distance(vfind, patchVerts.begin());
					if(patchGeo[ifind] > curGeo) patchGeo[ifind] = curGeo; 
				} else { 
					patchVerts.push_back(vj); 
					patchGeo.push_back(curGeo); 
					patchEuclid.push_back(v_norm(v.point() - vj->point()));
				}
			} while(++hv != vi->vertex_begin());
		} while( numNeigh < patchVerts.size()); 
		
		for(ifind=1; ifind < numNeigh; ifind++) {
			//sumRatio += patchEuclid[ifind]/(patchGeo[ifind]*edge_avg);
			sumEuclid += patchEuclid[ifind]; 
			sumGeo += patchGeo[ifind]*edge_avg; 
		}
		patchVerts.clear(); patchGeo.clear(); patchEuclid.clear(); 
		//mean_curv = sumRatio/numNeigh; 
		mean_curv = sumEuclid/sumGeo; 
		mean_curv = (mean_curv > 1) ? 1 : mean_curv;
	}
	
	if (isnan(gaussian_curv)) gaussian_curv = 0;
	if (isnan(mean_curv)) mean_curv = 0;			
	
	v.mean_curvature()=mean_curv;
	v.gaussian_curvature()=gaussian_curv;
} // compute curvatures

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeVertexLaplacian ( Vertex& v )
{
	// formula taken from "Mesh Smoothing via Mean and Median Filtering Applied to Face Normals"
	Vector result ( 0,0,0 );
	Vector result_laplacian ( 0,0,0 );
	double area = 0;
	
	Vector maxVertex ( 0,0,0 );
	
	std::size_t order = 0;
#define LAPLACIAN_METHOD 1
	
#if LAPLACIAN_METHOD==1
	HV_circulator vi = v.vertex_begin();
	bool contains_borders = false;
	if ( vi!=NULL )
		do
		{
			++order;
			if (vi->is_border_edge()) {
			 v.laplacian()=Vector(0,0,0);
			 return;
			 }
					
			result_laplacian = result_laplacian + ( vi->prev()->vertex()->point() - CGAL::ORIGIN );
		}
	while ( ++vi != v.vertex_begin() );
	result_laplacian = result_laplacian/order - ( v.point() -CGAL::ORIGIN );
	v.laplacian() = result_laplacian;
	
#elif LAPLACIAN_METHOD==2
	HV_circulator vi = v.vertex_begin();
	float w;
	float w_total = 0;
	Vector e, e_next, e_prev;
	float theta_1, theta_2;
	if ( vi!=NULL )
		do
		{
			e_next = vi->next()->vertex()->point() - vi->vertex()->point();
			e = vi->prev()->vertex()->point() - vi->vertex()->point();
			e_prev = vi->opposite()->next()->vertex()->point() - vi->vertex()->point();
			
			theta_1 = v_angle(e,e_next);
			theta_2 = v_angle(e,e_prev);
			
			w=(tan(theta_1/2)+tan(theta_2/2))/v_norm(e);
			w_total+=w;
			result_laplacian = result_laplacian + w*e;
		}
	while ( ++vi != v.vertex_begin() );
	result_laplacian=result_laplacian/w_total;
	v.laplacian() = result_laplacian;
#endif
}

// modes: H- Mean curvature; K - guassian curvature
//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeQualityPercentileBoundaries(float &lower_bound, float &upper_bound, float percentile, int bins) {
	Histogramf h;
	float quality_min=MAXFLOAT;
	float quality_max=-MAXFLOAT;
	
/*	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		if (mode=='H') vi->quality()=vi->mean_curvature();
		if (mode=='K') vi->quality()=vi->gaussian_curvature();
		if (mode=='D') vi->quality()=v_norm(vi->qual_vect);
	}
*/
	
	
	// get max and min	
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		if (quality_min > vi->quality())
			quality_min = vi->quality();
		if (quality_max < vi->quality())
			quality_max = vi->quality();
	}
	h.SetRange(quality_min,quality_max,bins);
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		h.Add(vi->quality());
	lower_bound = h.Percentile(percentile);
	upper_bound = h.Percentile(1.0f-percentile);
}


// modes: H- Mean curvature; K - guassian curvature
//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeQualityDiffPercentileBoundaries(float &lower_bound, float &upper_bound, float percentile, int bins) {
	Histogramf h;
	float quality_min=MAXFLOAT;
	float quality_max=-MAXFLOAT;
	
	/*	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
	 if (mode=='H') vi->quality()=vi->mean_curvature();
	 if (mode=='K') vi->quality()=vi->gaussian_curvature();
	 if (mode=='D') vi->quality()=v_norm(vi->qual_vect);
	 }
	 */
	
	
	// get max and min	
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
		if (quality_min > vi->quality()-vi->quality_prev())
			quality_min = vi->quality()-vi->quality_prev();
		if (quality_max < vi->quality()-vi->quality_prev())
			quality_max = vi->quality()-vi->quality_prev();
	}
	h.SetRange(quality_min,quality_max,bins);
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		h.Add(vi->quality()-vi->quality_prev());
	lower_bound = h.Percentile(percentile);
	upper_bound = h.Percentile(1.0f-percentile);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::computeDistanceFromVisualHull ( Polyhedron& vh_p )
{
	//init
	cout << " VISUAL HULL TERM " << endl;
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		vi->vh_dist=1;
	}
	
	for ( Facet_iterator vh_fi = vh_p.facets_begin(); vh_fi!=vh_p.facets_end(); vh_fi++ )
	{
		//KernelExact::Plane_3 plane_eq(TO_POINT3_EXACT(f->triangle()[0]),TO_POINT3_EXACT(f->triangle()[1]),TO_POINT3_EXACT(f->triangle()[2]));
		Kernel::Plane_3 plane_eq ( vh_fi->triangle() [0],vh_fi->triangle() [1],vh_fi->triangle() [2] );
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
		{
			
			Kernel::Line_3 line_1 ( vi->point(), vi->point() + vi->normal() );
			
			if ( CGAL::do_intersect ( vh_fi->triangle(),line_1 ) )
			{
				CGAL::Object result = CGAL::intersection ( plane_eq,line_1 );
				if ( const Kernel::Point_3 *_inter_point = CGAL::object_cast<Kernel::Point_3> ( &result ) )
				{
					Kernel::Point_3 inter_point = *_inter_point;
					
					if ( !line_1.has_on ( inter_point ) )
					{
						//cout << "numerical instability : line(" << vi->point() << "," << vi->point()+vi->normal() << ") doesn't have " << *inter_point << endl;
						inter_point = line_1.projection ( inter_point );
					}
					
					Kernel::Line_3 line_2 ( vi->point(), inter_point );
					float new_vh_dist = v_norm ( inter_point - vi->point() );
					
					
					
					if ( line_1==line_2.opposite() )
						new_vh_dist = ( -1 ) *new_vh_dist;
					
					if ( abs ( vi->vh_dist ) > abs ( new_vh_dist ) )
					{
						vi->vh_dist = new_vh_dist;
					}
				}
				else
				{
					cout << "intersection failed in computeDistanceFromVisualHull" << endl;
				}
			} //they do intersect
		}
	}
	
	for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	{
		if ( vi->vh_dist==1 ) vi->vh_dist=0;
	}
	cout << " END OF VISUAL HULL TERM " << endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::isPointInsideTriangle ( Kernel::Point_3 &the_point,Kernel::Triangle_3 &the_triangle )
{
	// Compute vectors
	Kernel::Vector_3 v0 = the_triangle[2] - the_triangle[0];
	Kernel::Vector_3 v1 = the_triangle[1] - the_triangle[0];
	Kernel::Vector_3 v2 = the_point - the_triangle[0];
	
	// Compute dot products
	double dot00 = v0*v0;
	double dot01 = v0*v1;
	double dot02 = v0*v2;
	double dot11 = v1*v1;
	double dot12 = v1*v2;
	
	// Compute barycentric coordinates
	double invDenom = 1 / ( dot00 * dot11 - dot01 * dot01 );
	double u = ( dot11 * dot02 - dot01 * dot12 ) * invDenom;
	double v = ( dot00 * dot12 - dot01 * dot02 ) * invDenom;
	
	// Check if point is in triangle
	return ( u > 0 ) && ( v > 0 ) && ( u + v < 1 );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
Vertex* Mesh::vertexMapping(Kernel::Point_3 &the_point) {
	int count=0;
	if ( !is_mapping_init ) {
		vertex_mapping.clear();
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {			
			vi->id = count++; 
			vertex_mapping[vi->point()] = &(*vi);
		}
		is_mapping_init = true;
	}
	return vertex_mapping[the_point];
	
	
}

Kernel::Point_3 Mesh::closestColorPoint ( Kernel::Point_3 &the_point, float *givclr )
{
	
#define NUM_NEIGH 10
	Vertex *vtx; 
	Kernel::Point_3 pt, avg(0,0,0), minpt(0,0,0);
	float sum, dist, factor, colorDiff, minDiff=9999, minDist=9999; 
	int count=0;
	
	float colorThreshold, distThreshold;
	colorThreshold = 0.0001; distThreshold = 10 * edge_avg;
	double sigma = close_color_sigma * edge_avg;
	
	if(givclr[0] == 1 && givclr[1] == 1 && givclr[2] == 1) 
		return the_point;
	if(givclr[0] == 0 && givclr[1] == 0 && givclr[2] == 0) 
		return the_point;
	
	if ( !is_search_init )
	{
		if(! is_mapping_init) vertex_mapping.clear();
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
			if(! is_mapping_init) { vertex_mapping[vi->point()] = &(*vi); vi->id = count++; }
			search_tree.insert (vi->point());
		}
		is_search_init = true; is_mapping_init = true;
	}
	//	Point_Search ps_query(the_point.x(),the_point.y(),the_point.z(),NULL);	
	//	Neighbor_search search ( search_tree, ps_query, 1 );
	
	Neighbor_search search ( search_tree, the_point, NUM_NEIGH );	// Take the 20 closest neighbors
	
	sum = 0;
	for ( Neighbor_search::iterator it = search.begin(); it != search.end(); ++it )
	{
		pt = it->first;
		vtx = vertex_mapping[pt]; 
		//vtx = vertexMapping(pt);	
		dist = v_norm(pt - the_point);	
		
		colorDiff = 0; 
		for(int j=0; j < 3; j++) 
			colorDiff += (givclr[j]-vtx->color[j])*(givclr[j]-vtx->color[j]);
		colorDiff = sqrt(colorDiff);
		
		if(colorDiff < colorThreshold) { if(dist < minDist) { minDist = dist; minpt = pt; minDiff = colorDiff; } }
		//if(colorDiff < colorThreshold) { if(dist < minDist && colorDiff < minDiff) { minDist = dist; minpt = pt; minDiff = colorDiff; } }
		
		factor = exp(-dist*dist/(2*sigma*sigma)) * (1 - colorDiff);
		//avg = avg + factor * (pt - the_point);
		avg = avg + factor * (pt - CGAL::ORIGIN);
		sum += factor; 
	}
	if(v_norm(minpt - the_point) < distThreshold) return minpt; 
	
	return the_point; // don't move this point ? 
	
	//avg = the_point + 1/sum * (avg - CGAL::ORIGIN);
	
	if(sum < 0.01) // don't move the point if there is not too much information
		avg = the_point;
	else
		avg = CGAL::ORIGIN + 1/sum * (avg - CGAL::ORIGIN);
	
	return avg;
	
	//cerr << "movement ( "<< v_norm(avg-CGAL::ORIGIN) << ")" << endl; 
	//return closestPoint(avg); 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
Kernel::Point_3 Mesh::closestPoint ( Kernel::Point_3 &the_point )
{
	int count=0;
	if ( !is_search_init )
	{
		Neighbor_search_tree tmpTree;
		//search_tree=tmpTree;
		search_tree.clear();
		if(! is_mapping_init) vertex_mapping.clear();
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
			if(! is_mapping_init) {vertex_mapping[vi->point()] = &(*vi); } // vi->id = count++; 
			search_tree.insert (vi->point());
		}
		
		is_search_init = true; is_mapping_init = true;
	}
	//	Point_Search ps_query(the_point.x(),the_point.y(),the_point.z(),NULL);	
	//	Neighbor_search search ( search_tree, ps_query, 1 );
	
	Neighbor_search search ( search_tree, the_point, 1 );	
	
	for ( Neighbor_search::iterator it = search.begin(); it != search.end(); ++it )
	{
		return it->first;	 	
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
Vertex::Vertex_handle Mesh::closestVertex ( Kernel::Point_3 &the_point )
{
	Kernel::Point_3 p = closestPoint(the_point);
	return vertex_mapping[p]->halfedge()->vertex();
	
	/*	Vertex::Vertex_handle result = p.vertices_begin();
	 Kernel::Vector_3 dist_vector=p.vertices_begin()->point()-the_point;
	 Kernel::Vector_3 tmp_vector;
	 double dist_size=dist_vector*dist_vector;
	 double tmp_size;
	 
	 //compute distance from points
	 for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ )
	 {
	 tmp_vector = vi->point()-the_point;
	 tmp_size = tmp_vector*tmp_vector;
	 if ( tmp_size < dist_size )
	 {
	 dist_size = tmp_size;
	 dist_vector = tmp_vector;
	 result=vi;
	 }
	 }
	 return result;
	 */
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
Vertex* Mesh::getVertexById ( int id )
{
	if (id_mapping.size()==0)
		for ( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++ ) {
			id_mapping.push_back(&(*vi));
		}
	if (id > id_mapping.size()-1) 
		return NULL;
	else
		return id_mapping[id];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
float Mesh::getSurfacePercentageRadius(float x)
{
	// returns the radius of a circle covering x percentage from the total surface area
	return sqrt((x/100)*area_avg*p.size_of_facets()/PI/4);	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
float Mesh::distanceTo(Mesh &other, bool reinit_search_structures) {
	if (reinit_search_structures) {
		is_mapping_init=false;
		is_search_init=false;
	}

	float max_distance=0;

	for ( Vertex_iterator vi = other.p.vertices_begin(); vi!=other.p.vertices_end(); vi++ ) {
		Point p = closestPoint(vi->point());
		Vector d = p -vi->point();
		max_distance=std::max(max_distance,(float)v_norm(d));
	}	
	return max_distance;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// 0 - compute real distance
// 1 - compute closest point - using search trees
// 2 - compute closest point - using normal search
Kernel::Vector_3 Mesh::computeDistanceFromPoint ( Kernel::Point_3 &the_point, int mode )
{
	
	Kernel::Vector_3 dist_vector;
	double dist_size;
	Kernel::Vector_3 tmp_vector;
	double tmp_size;
	
	if ( ( mode==1 ) || ( mode==0 ) )
	{
		dist_vector=closestPoint ( the_point )-the_point;
		dist_size=dist_vector*dist_vector;
	}
	
	if ( mode==2 )
	{
		dist_vector=closestVertex ( the_point )->point()-the_point;
		dist_size=dist_vector*dist_vector;
		return dist_vector;
		
	}
	
	if ( mode==0 ) 	//compute distance from facets
		for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++ )
		{
			Kernel::Triangle_3 triang = fi->triangle();
			Kernel::Plane_3 plane_eq;
			//		if (usePrecomputedPlaneEquations)
			//			plane_eq = fi->supporting_plane();
			//		else
			plane_eq = triang.supporting_plane();
			
			Kernel::Point_3	proj_p = plane_eq.projection ( the_point );
			if ( isPointInsideTriangle ( proj_p,triang ) )
			{
				tmp_vector = proj_p-the_point;
				tmp_size = tmp_vector*tmp_vector;
				if ( tmp_size < dist_size )
				{
					dist_size = tmp_size;
					dist_vector = tmp_vector;
				}
				
			}
		}
	
	return dist_vector;
	
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::canSplitEdge ( Vertex::Halfedge_handle h )
{
	if ( h->is_border_edge() ) return false;
	if ( h->facet() == h->opposite()->facet() ) return false;
	if ( ( !h->facet()->is_triangle() ) || ( !h->opposite()->facet()->is_triangle() ) ) return false;
	if ( h->vertex()->point() ==h->opposite()->vertex()->point() ) return false;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::splitEdge ( Vertex::Halfedge_handle h, int mode ) { // 1 - middle ; 2-projection
	Point p1 = h->vertex()->point();
	Point p2 = h->opposite()->vertex()->point();
	Point p3 = h->next()->vertex()->point();
	
	double ratio;
	Point p_midddle;
	
	if ( mode==1 )
	{ //middle
		ratio = 0.5;
		p_midddle = p1 + ( p2-p1 ) *ratio;
	}
	else if ( mode==2 )
	{ // projection of the 3rd vertex
		ratio = v_norm ( p3-p2 ) *cos ( oppositeAngle ( h->next() ) ) / v_norm ( p1-p2 );
		p_midddle = p2 + ( p1-p2 ) *ratio;
	}
	else
	{
		cout << "splitEdge - invalid mode !" << endl;
		return;
	}
	lock();
	Vertex::Halfedge_handle hnew = p.split_edge ( h );
	hnew->vertex()->point() = p_midddle;
	hnew->vertex()->weight = ( h->vertex()->weight + hnew->opposite()->vertex()->weight ) /2;
	hnew->vertex()->id = -1;
	
	p.split_facet ( hnew,h->next() );
	p.split_facet ( h->opposite(),hnew->opposite()->next() );
	unlock();
	
}

#define REPLACE_POINT(V,P1,P2,PMIDDLE) (((V==P1)||(V==P2)) ? (PMIDDLE) : (V))

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::canCollapseCenterVertex ( Vertex::Vertex_handle v )
{
	if ( v->is_trivalent() ==false ) return false;
	if ( v->border()) return false;
	
	if ( v->halfedge()->prev()->opposite()->facet() ==v->halfedge()->opposite()->next()->opposite()->facet() ) return false;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::canCollapseEdge ( Vertex::Halfedge_handle v0v1 )
{
	Vertex::Halfedge_handle v1v0 = v0v1->opposite();
	Vertex::Vertex_handle v0 = v0v1->vertex();
	Vertex::Vertex_handle v1 = v1v0->vertex();

	if ( v0v1->is_border_edge() ) return false;
	if (v0->border() || v1->border()) return false;
	Vertex::Vertex_handle vl, vr, vTmp;
	Vertex::Halfedge_handle h1,h2;
	
	
	if ( v0v1->next()->opposite()->facet() == v1v0->prev()->opposite()->facet() ) return false;
	
	if ( !v0v1->is_border() )
	{
		vl = v0v1->next()->vertex();
		h1 = v0v1->next();
		h2 = v0v1->next()->next();
		if ( h1->is_border() || h2->is_border() )
			return false;
	}
	
	if ( !v1v0->is_border() )
	{
		vr = v1v0->next()->vertex();
		h1 = v1v0->next();
		h2 = v1v0->next()->next();
		if ( h1->is_border() || h2->is_border() )
			return false;
	}
	// if vl and vr are equal or both invalid -> fail
	if ( vl == vr ) return false;
	
	HV_circulator c,d;
	
	// test intersection of the one-rings of v0 and v1
	
	c = v0->vertex_begin(); d = c;
	CGAL_For_all ( c, d )
	c->opposite()->vertex()->flag[0] = false;
	
	c = v1->vertex_begin(); d = c;
	CGAL_For_all ( c, d )
	c->opposite()->vertex()->flag[0] = true;
	
	c = v0->vertex_begin(); d = c;
	CGAL_For_all ( c, d )
	{
		vTmp =c->opposite()->vertex();
		if ( vTmp->flag[0] && ( vTmp!=vl ) && ( vTmp!=vr ) )
			return false;
	}
	
	// test weather when performin the edge collapse we change the signed area of any triangle
	
#if STRONG_EDGE_COLLAPSE_CHECK==1
	Point p0 = v0->point();
	Point p1 = v1->point();
	Point p_middle = p0 + ( p1 - p0 ) / 2;
	Point t1,t2,t3;
	Vector a1,a2;
	
	for ( int x=0;x<2;x++ )
	{
		if ( x==0 )
		{
			c = v0->vertex_begin(); d = c;
		}
		else
		{
			c = v1->vertex_begin(); d = c;
		}
		CGAL_For_all ( c, d )
		{
			t1 = c->vertex()->point();
			t2 = c->next()->vertex()->point();
			t3 = c->next()->next()->vertex()->point();
			a1 = CGAL::cross_product ( t2-t1,t3-t2 );
			t1 = REPLACE_POINT ( t1,p0,p1,p_middle );
			t2 = REPLACE_POINT ( t2,p0,p1,p_middle );
			t3 = REPLACE_POINT ( t3,p0,p1,p_middle );
			a2 = CGAL::cross_product ( t2-t1,t3-t2 );
			
			if ( ( v_norm ( a2 ) != 0 ) && ( v_angle ( a1,a2 ) > PI/2 ) )
				return false;
		}
	}
#endif
	/*
	 // check all the intersections
	 Point p0 = v0->point();
	 Point p1 = v1->point();
	 Point p_middle = p0 + (p1 - p0) / 2;
	 Point t1,t2,t3;
	 Vector a1,a2;
	 
	 for(int x=0;x<2;x++) {
	 if (x==0) {
	 c = v0->vertex_begin(); d = c;
	 }
	 else {
	 c = v1->vertex_begin(); d = c;
	 }
	 CGAL_For_all( c, d) {
	 t1 = c->vertex()->point();
	 t2 = c->next()->vertex()->point(); 
	 t3 = c->next()->next()->vertex()->point();
	 a1 = CGAL::cross_product(t2-t1,t3-t2);
	 t1 = REPLACE_POINT(t1,p0,p1,p_middle);
	 t2 = REPLACE_POINT(t2,p0,p1,p_middle);
	 t3 = REPLACE_POINT(t3,p0,p1,p_middle);	
	 a2 = CGAL::cross_product(t2-t1,t3-t2);
	 
	 if ((v_norm(a2) != 0) && (v_angle(a1,a2) > PI/2))
	 return false; 
	 }
	 }
	 */
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::collapseEdge ( Vertex::Halfedge_handle h )
{
	
	//	h->vertex()->weight=1;
	//	h->opposite()->vertex()->weight=1;
	
	Vertex::Halfedge_handle h1,h2;
	h1 = h->next();
	h2 = h->opposite()->prev();
	Point p1 = h->vertex()->point();
	Point p2 = h->opposite()->vertex()->point();
	Point p3 = p1 + ( p2-p1 ) /2;
	std::size_t degree_p1 = h->vertex()->vertex_degree();
	std::size_t degree_p2 = h->opposite()->vertex()->vertex_degree();

/*	if (h->vertex()->border()==true)
		p3=p1;
	else if (h->opposite()->vertex()->border()==true)
			 p3=p2;
	else */
	if (degree_p1 > degree_p2)
		p3=p1;
	else
		p3=p2;
	
	h->vertex()->point() = p3;
	lock();	
	p.join_facet ( h1->opposite() );
	p.join_facet ( h2->opposite() );
	p.join_vertex ( h );
	unlock();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 void Mesh::vertexSplit(Vertex::Halfedge_handle h, Vertex::Halfedge_handle g) {
 
 Vertex::Halfedge_handle newh = p.split_vertex(h, g);
 
 Point h_hacet_center = h->facet()->center();
 Point g_hacet_center = g->facet()->center();
 
 h->vertex()->point() = h_hacet_center;
 g->vertex()->point() = g_hacet_center;
 
 p.split_facet(h1->opposite());
 p.split_facet(h2->opposite());
 
 
 
 
 Vertex::Halfedge_handle h1,h2;
 h1 = h->next();
 h2 = h->opposite()->prev();
 Point p1 = h->vertex()->point();
 Point p2 = h->opposite()->vertex()->point();
 Point p_3 = p1 + (p2-p1)/2;
 h->vertex()->point() = p_3;
 }
 */

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool Mesh::canFlipEdge ( Vertex::Halfedge_handle h )
{
	//XXX: TODO : verify the quality of the other triangle as well
	if ( h->is_border_edge() ) return false;
	Vertex::Halfedge_handle null_h;
	if ( ( h->next() ==null_h ) | ( h->prev() ==null_h ) || ( h->opposite() ==null_h ) || ( h->opposite()->next() ==null_h ) ) return false;
	Vertex::Vertex_handle v0 = h->next()->vertex();
	Vertex::Vertex_handle v1 = h->opposite()->next()->vertex();
	
	v0->flag[0] = false;
	
	HV_circulator c,d;
	c = v1->vertex_begin(); d = c;
	CGAL_For_all ( c, d )
	c->opposite()->vertex()->flag[0] = true;
	
	if ( v0->flag[0] ) return false;
	
	// check if it increases the quality overall
	double a1 = oppositeAngle ( h );
	double a2 = oppositeAngle ( h->next() );
	double a3 = oppositeAngle ( h->next()->next() );
	double b1 = oppositeAngle ( h->opposite() );
	double b2 = oppositeAngle ( h->opposite()->next() );
	double b3 = oppositeAngle ( h->opposite()->next()->next() );
	
	if ( ( a1*a1 + b1*b1 ) / ( a2*a2 + a3*a3 + b2*b2 + b3*b3 ) < 1.01 ) return false;
	
	Vector v_perp_1 = CGAL::cross_product ( h->vertex()->point() - h->next()->vertex()->point(), h->vertex()->point() - h->prev()->vertex()->point() );
	//	Vector v_perp_2 = CGAL::cross_product(h->opposite()->vertex()->point()-h->opposite()->next()->vertex()->point(),h->opposite()->vertex()->point()-h->opposite()->prev()->vertex()->point());
	Vector v_perp_2 = CGAL::cross_product ( h->opposite()->next()->vertex()->point() - h->opposite()->vertex()->point(),h->vertex()->point()-h->prev()->vertex()->point() );
	if ( v_angle ( v_perp_1,v_perp_2 ) > PI/180*20 ) return false;
	
	return ( h->next()->opposite()->facet() != h->opposite()->prev()->opposite()->facet() ) &&
	( h->prev()->opposite()->facet() != h->opposite()->next()->opposite()->facet() ) &&
	( circulator_size ( h->opposite()->vertex_begin() ) >= 3 ) &&
	( circulator_size ( h->vertex_begin() ) >= 3 );
}

void Mesh::flipEdge ( Vertex::Halfedge_handle h )
{
	/*	Vertex::Halfedge_handle h2 = h->next();
	 p.join_facet(h);
	 p.split_facet(h2,h2->next()->next());
	 */
	//	h->vertex()->weight=1;
	//	h->opposite()->vertex()->weight=1;
	lock();
	p.flip_edge ( h );
	unlock();
	//	h->vertex()->weight=1;
	//	h->opposite()->vertex()->weight=1;
	
}


void Mesh::improveVertexValence ( int valence_mode )
{
	//	int valence_mode =1;
	//	return;
	
	if ( valence_mode==1 )
	{
		//erase all the center triangles!
		for ( Vertex_iterator vi= p.vertices_begin(); vi!=p.vertices_end(); )
		{
			Vertex_handle old_vi = vi;
			vi++;
			std::size_t degree = old_vi->vertex_degree();
			//size - 3
			if ( canCollapseCenterVertex ( old_vi ) ) {
				lock();
				p.erase_center_vertex ( old_vi->halfedge() );
				unlock();
			}
		}
		
		//erase all the center triangles!
		for ( Vertex_iterator vi= p.vertices_begin(); vi!=p.vertices_end(); )
		{
			Vertex_handle old_vi = vi;
			vi++;
			std::size_t degree = old_vi->vertex_degree();
			//		cout << "degree" << degree << endl;
			//size - 3
			//		if (canCollapseCenterVertex(old_vi)) p.erase_center_vertex(old_vi->halfedge());
			//		else
			if ( degree==4 )
			{
				double edge_stats = computeVertexStatistics ( *old_vi,0 ); //min
				//			cout << edge_stats << endl;
				HV_circulator c,d;
				c = old_vi->vertex_begin();
				float current_edge_stats;
				current_edge_stats = edge_size ( c );
				
				while ( edge_stats!=current_edge_stats )
				{
					//				cout << "current edge stats:" << current_edge_stats << endl;
					c++;
					current_edge_stats = edge_size ( c );
				}
				d = c;
				bool collapsed = false;
				CGAL_For_all ( c, d )
				if ( canCollapseEdge ( c->opposite() ) )
				{
					//					if ((c->opposite()->vertex()==vi) && (vi!=p.vertices_end())) vi++;
					collapseEdge ( c->opposite() );
					collapsed = true;
					break;
				}
				//			if (!collapsed) cout << "could not collapse edge!" << endl;
			}
			
		}
	}
	
	if ( valence_mode==2 )
	{
		int l_iters=0;
		int no_ops;
		do
		{
			l_iters++;
			no_ops=0;
			
			for ( Edge_iterator ei= p.edges_begin(); ei!=p.edges_end(); ) {
				if ((ei->vertex()->getVisibility()==false) || ei->is_border_edge()) {
					ei++;
					continue;
				}
				int d1_1 = ei->vertex()->vertex_degree();
				int d1_2 = ei->opposite()->vertex()->vertex_degree();
				int d2_1 = ei->next()->vertex()->vertex_degree();
				int d2_2 = ei->opposite()->next()->vertex()->vertex_degree();
				Halfedge_handle h = ei;
				ei++;
				if ( ( ( d1_1+d1_2 ) - ( d2_1+d2_2 ) > 2 ) && canFlipEdge ( h ) )
				{
					no_ops++;
					flipEdge ( h );
				}
			}
			//		fixDegeneracy(0.2,150);
		}
		while ( ( no_ops>0 ) && ( l_iters<1 ) );
	}

	if ( valence_mode==3 )
	{
		int l_iters=0;
		int no_ops;
		do
		{
			l_iters++;
			no_ops=0;
			
			for ( Edge_iterator ei= p.edges_begin(); ei!=p.edges_end(); ) {
				if ((ei->vertex()->getVisibility()==false) || ei->is_border_edge()) {
					ei++;
					continue;
				}
				Point p1=ei->vertex()->point();
				Point p2=ei->next()->vertex()->point();
				Point p3=ei->prev()->vertex()->point();
				Point p4=ei->opposite()->next()->vertex()->point();
				
				float cost1 = std::min(std::min(std::min(std::min(std::min(p_angle(p3,p1,p2),p_angle(p1,p3,p2)),p_angle(p4,p1,p3)),p_angle(p4,p3,p1)),p_angle(p1,p4,p2)),p_angle(p1,p2,p3));
				float cost2 = std::min(std::min(std::min(std::min(std::min(p_angle(p1,p2,p4),p_angle(p1,p4,p2)),p_angle(p3,p4,p2)),p_angle(p4,p2,p3)),p_angle(p4,p1,p2)),p_angle(p4,p3,p2));
				
				Halfedge_handle h = ei;
				ei++;
				if ( ( cost2 > cost1 ) && canFlipEdge(h) )
				{
					no_ops++;
					flipEdge ( h );
				}
			}
			//		fixDegeneracy(0.2,150);
		}
		while ( ( no_ops>0 ) && ( l_iters<1 ) );
	}	
	//	cout << "edge swaps after minimization=" << l_iters << endl;
	
}


/*
 Description: 
	It iterates through all the mesh vertices and it tries to fix degenerate triangles.
	There are conditions that check for large and small angles.
 Parameters:
	- degenerateAngleDeg 
			- for large angles: if an angle is bigger than degenerateAngleDeg.
			- a good values to use is typically 170
	- collapseRatio 
			- for small angles: given the corresponding edges (a,b,c) in all permutations, if (a/b < collapseRatio) & (a/c < collapseRatio)
			- a good value to use is 0.1 
 */
int Mesh::fixDegeneracy ( double collapseRatio,double degenerateAngleDeg )
{
	int no_ops=0;
#if DEBUG_MESH>1
	cout << "IMPROVING MESH" <<endl;
	cout << " - collapseRatio=" << collapseRatio << " degenerateAngleDeg=" << degenerateAngleDeg << endl;
#endif
	double edge[3];
	int counter=0;
	Vertex::Halfedge_handle edge_h[3];
	
	double extremeCollapseRatio = 1;
	double extremeDegenerateAngleDeg = 0;
	
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); )
	{
		counter++;
		if ( fi->triangle().is_degenerate() )
		{
		//	cout << "DEGENERATE angle is = " << v_angle ( fi->halfedge()->vertex()->point() - fi->halfedge()->next()->vertex()->point(), fi->halfedge()->vertex()->point() - fi->halfedge()->next()->next()->vertex()->point() );// << " for " << fi->triangle();
			double tmp_avg_edge = fi->edgeStatistics ( 1 );
		//	cout << " avg edge = " << tmp_avg_edge << endl;
			fi->halfedge()->vertex()->point() = fi->halfedge()->vertex()->point() + Vector ( 0,0,tmp_avg_edge*0.01 );
			fi->halfedge()->next()->vertex()->point() = fi->halfedge()->next()->vertex()->point() + Vector ( 0,tmp_avg_edge*0.01,0 );
			fi->halfedge()->next()->next()->vertex()->point() = fi->halfedge()->next()->next()->vertex()->point() + Vector ( tmp_avg_edge*0.01,0,0 );
		}
		
		// collect facet statistics
		HF_circulator hf = fi->facet_begin();
		int i=0;
		if ( hf==NULL ) continue;
		do
		{
			/*			if (isnan(v_norm(hf->vertex()->point() - CGAL::ORIGIN))) {
			 cout << "POINTS ARE AREADY NAN!" << endl;
			 }*/
			if ( i==3 ) {
				cout << "fixDegeneracy: not a triangular mesh!!!!" << endl; 
				saveFormat("not_triangular.off");
				if (hf->is_border_edge()) cout << "forgiven - border edge " << endl;
				return -1;
				}
			Vector a = hf->vertex()->point() - hf->prev()->vertex()->point();
			edge_h[i] = hf;
			edge[i++] = v_norm ( a );
		}
		while ( ++hf !=fi->facet_begin() );
		
		fi++;
		//we should have the 3 sizes of the edges by now
		for ( i=0;i<3;i++ )
		{
			extremeCollapseRatio = std::min ( extremeCollapseRatio, std::min ( edge[i]/edge[ ( i+1 ) %3], edge[i]/edge[ ( i+2 ) %3] ) );
			double tmpAngle = TO_DEG ( oppositeAngle ( edge_h[i] ) );
			extremeDegenerateAngleDeg = std::max ( extremeDegenerateAngleDeg,tmpAngle );
			if ( ( edge[i]/edge[ ( i+1 ) %3] < collapseRatio ) && ( edge[i]/edge[ ( i+2 ) %3] < collapseRatio ) )
			{
				if ( canCollapseEdge ( edge_h[i] ) )
				{
					while ( ( fi!=p.facets_end() ) && ( fi==edge_h[i]->opposite()->facet() ) ) fi++;
					collapseEdge ( edge_h[i] );
					no_ops++;
					break;
				}
				/*				else if (canCollapseEdge(edge_h[i]->opposite())) {
				 while ((fi!=p.facets_end()) && (fi==edge_h[i]->facet())) fi++;
				 collapseEdge(edge_h[i]->opposite());
				 break;
				 }
				 */
			}
			else
				if ( ( tmpAngle > degenerateAngleDeg ) && canFlipEdge ( edge_h[i] ) )
				{//
					flipEdge ( edge_h[i] );
					no_ops++;
					break;
				}
			/*			else
			 if ((tmpAngle > degenerateAngleDeg) && canSplitEdge(edge_h[i])) {
			 //				splitEdge(edge_h[i]);
			 splitEdge(edge_h[i],2);
			 if (canCollapseEdge(edge_h[i]->prev())) {
			 while ((fi!=p.facets_end()) && (fi==edge_h[i]->prev()->opposite()->facet())) fi++;
			 collapseEdge(edge_h[i]->prev());
			 }
			 no_ops++;
			 break;
			 }
			 */
		}
		
	}
	return no_ops;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh::removeInvisibleFacets() {
	for ( Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); ) {
		
		HF_circulator hf = fi->facet_begin();
		int i=0;
		if ( hf==NULL ) continue;
		bool erasedFacet=false;
		do {
			if (hf->vertex()->getVisibility()==false) {
				erasedFacet=true;			
				fi++;
				p.erase_facet(hf);
				break;
			}
		} while ( ++hf !=fi->facet_begin() );
		if (erasedFacet==false) fi++;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define INSIDE_INTERVAL(x,A,B) ((A<=x) &&  (x<=B))
void Mesh::restrictToBoundingBox(float x1, float y1, float z1, float x2, float y2, float z2) {
	for( Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++) {
		float x_p=vi->point().x();
		float y_p=vi->point().y();		
		float z_p=vi->point().z();		
		bool inside= INSIDE_INTERVAL(x_p,x1,x2) && INSIDE_INTERVAL(y_p,y1,y2) && INSIDE_INTERVAL(z_p,z1,z2);
		vi->setVisibility(inside);
	}
	
	removeInvisibleFacets();
}

void Mesh::fixDegeneracy ( bool updateMesh )
{
	//	fixDegeneracy(0.01*(tmpFix),0.95);
	//if (tmpFix < 44)
	tmpFix=15;
	fixDegeneracy ( 0.01* ( tmpFix ),180-tmpFix );
	//else fixDegeneracy(0.01*(tmpFix),130);
	
	//	fixDegeneracy(0.01*(tmpFix),181);
	//	if (tmpFix<40) tmpFix+=3;
	if ( updateMesh ) updateMeshData();
}

void Mesh::fixAllDegeneracy ( double collapseRatio,double degenerateAngleDeg )
{
	int size_of_facets;
	int runs=0;
	do
	{
		size_of_facets = p.size_of_facets();
		//		fixDegeneracy(collapseRatio,degenerateAngleDeg);
		improveVertexValence();
		runs++;
	}
	while ( ( p.size_of_facets() !=size_of_facets ) && ( runs<1 ) );
	//	fixDegeneracy(collapseRatio,degenerateAngleDeg);
	updateMeshData(false,false);
}

/**
 Description: 
 - The goal of this method is to ensure that all the edges of the mesh are within the interval [epsilonMin,epsilonMax].
   In order to do so, edge collapses and edge split operations are performed.
 - The method also attempts to fix degeneracies by invoking fixDegeneracy(collapseRatio,degenerate_angle_deg) and performs some local smoothing, based on the operating mode.
 Parameters:
 - [epsilonMin, epsilonMax] - the desired edge interval
 - collapseRatio, degenerate_angle_deg - parameters used to invoke fixDegeneracy (see function for more details)
 - mode : 0 - fixDegeneracy=No  smoothing=Yes;
		  1 - fixDegeneracy=Yes smoothing=Yes; (default)
		 10 - fixDegeneracy=Yes smoothing=No;
 - max_iter (default=30) - maximum number of iterations to be performed; since there is no guarantee that one operations (such as a collapse, for example)
   will not in turn generate new degeneracies, operations are being performed on the mesh in an iterative fashion. 
 */
void Mesh::ensureEdgeSizes ( double epsilonMin, double epsilonMax, double collapseRatio,double degenerate_angle_deg, int mode, int max_iters )
{
	int no_ops=1;
	int iters=0;
	//	int max_iters=30;
	int smallEdges=1;
	
	//mode
	bool invokeFixes=true;
	
	queue<Vertex::Halfedge_handle> bigEdgesQueue;
	queue<Vertex::Halfedge_handle> smallEdgesQueue;
	
	if ( mode>0 )
	{
		fixDegeneracy ( collapseRatio, degenerate_angle_deg );
		//		fixAllDegeneracy(collapseRatio, degenerate_angle_deg);
		//		remeshWithThreshold(epsilonMax);
		
		//		improveVertexValence();
		//fixAllDegeneracy(collapseRatio, degenerate_angle_deg);
		
	}
	
	computeMeshStats();
	cout.precision ( 8 );
	
#if DEBUG_MESH>0
	cout << "ENSURING EDGE_SIZE IN {" << epsilonMin << "," << epsilonMax << "} (where edges in {" << edge_min << "," << edge_max << "} )" << endl;
#endif	
	while ( ( ( ( edge_min<epsilonMin ) ) || ( edge_max>epsilonMax ) ) && ( no_ops>0 ) && ( iters<max_iters ) )
	{
		iters++;
		no_ops=0;
		double currentEpsilonMax = max ( edge_max / 2 ,epsilonMax );
		currentEpsilonMax = epsilonMax;
		//		smooth(0.8,1);
		// fixing flips
		//		if (!invokeFixDegen) {
		if ( invokeFixes )
		{
			//				fixDegeneracy(collapseRatio, degenerate_angle_deg);
		}
		//
		/*
		 Halfedge_iterator end_edge = p.halfedges_end();
		 for(Halfedge_iterator hi=p.edges_begin();hi!=p.edges_end();) {
		 Halfedge_handle old_hi = hi;
		 hi++;
		 if (hi->opposite()==old_hi) hi++;
		 if (canFlipEdge(old_hi) && (edge_size(old_hi) > currentEpsilonMax)) { //&& old_hi->vertex()->flag[1]==false
		 if (v_norm(old_hi->next()->vertex()->point() - old_hi->opposite()->next()->vertex()->point()) < epsilonMin) continue;
		 old_hi->vertex()->flag[1]==true;
		 old_hi->opposite()->vertex()->flag[1]==true;
		 flipEdge(old_hi);
		 no_ops++;
		 }
		 }
		 */
		//	}
		
		//process big edges
		for ( Halfedge_iterator h = p.edges_begin(); h != p.edges_end(); ++h )
		{
			if ( ( edge_size ( h ) > currentEpsilonMax )) { 
				bigEdgesQueue.push ( h );
			}
		}
#if DEBUG_MESH>1
		cout << "added big edges=" << bigEdgesQueue.size() << endl;
#endif
		while ( !bigEdgesQueue.empty() ) {
			Vertex::Halfedge_handle h = bigEdgesQueue.front(); bigEdgesQueue.pop();
			float avg_mean_curv = max(abs(h->vertex()->mean_curvature()),abs(h->prev()->vertex()->mean_curvature()));
			if ( (h->vertex()->getVisibility())  && (h->prev()->vertex()->getVisibility()) && canSplitEdge ( h ) && (avg_mean_curv > 0.1)) {splitEdge ( h ); no_ops++; }
			/*			if ((edge_size(h->prev()) > currentEpsilonMax))
			 bigEdges.push(h->prev());
			 if ((edge_size(h->opposite()->next()) > currentEpsilonMax))
			 bigEdges.push(h->opposite()->next());
			 */
		}
		
		// process small edges
		Facet_iterator f = p.facets_begin();
		Facet_handle old_f;
		while ( f !=p.facets_end() ) {
			old_f = f;
			f++;
			int oldSmallEdges=smallEdges;
			double minEdge = old_f->edgeStatistics ( 0 );
			if ( minEdge < epsilonMin )
			{
				HF_circulator h = old_f->facet_begin();
				do
				{
					if ( h->vertex()->getVisibility() && ( edge_size ( h ) == minEdge ) && canCollapseEdge ( h ) )
					{
						no_ops++;
						while ( ( f!=p.facets_end() ) && ( ( f==h->opposite()->facet() ) || ( f==h->facet() ) ) )
						{
							f++;
						}
						collapseEdge ( h );
						break;
					}
				}
				while ( ++h==old_f->facet_begin() );
			}
		}
		
		/*
		 //process small edges
		 for ( Edge_iterator h = p.edges_begin(); h != p.edges_end(); )
		 {
		 if ( ( edge_size ( h ) < epsilonMin) && canCollapseEdge(h) ) {
		 Vertex::Halfedge_handle col_h = h;
		 h++;
		 while ( (h!=p.edges_end()) && ( (h==col_h->next()) || (h==col_h->next()->opposite()) || (h==col_h->opposite()->prev()) || (h==col_h->opposite()->prev()->opposite()) ) )
		 {
		 h++;
		 }
		 //smallEdgesQueue.push ( h );
		 collapseEdge( col_h ); no_ops++;
		 }
		 else
		 h++;
		 }
		 */
		
#if DEBUG_MESH>1
		cout << "delete small edges=" << smallEdges << endl;
#endif
		
		//		if (invokeFixDegen) fixAllDegeneracy(collapseRatio, degenerate_angle_deg);
		//		improveVertexValence();
		//		improveVertexValence();
		//		fixDegeneracy(collapseRatio, degenerate_angle_deg);
		if ( mode==10 )
		{
			improveVertexValence ( 2 );
			fixDegeneracy ( collapseRatio, degenerate_angle_deg );
		}
		else
			if ( mode <10 ) improveVertexValence ( 2 );
		
		//		computeMeshStats();
		updateMeshData();
		if ( mode <10 ) smooth ( 0.1,1);
		
	}
#if DEBUG_MESH>0
	cout << "no_ops=" << no_ops << " and iters=" << iters << endl;
	cout << "EDGE_SIZE IN {" << edge_min << "," << edge_max << "} before fixAllDegeneracy" << endl;
#endif	
	if ( ( mode > 0 ) && ( iters==max_iters ) )
	{
		
		fixAllDegeneracy ( collapseRatio, degenerate_angle_deg );
#if DEBUG_MESH>0				
		cout << "EDGE_SIZE IN {" << edge_min << "," << edge_max << "}" << endl;
#endif	
		//	computeMeshStats();
		if ( mode <10 ) smooth ( 0.1,1 );
		updateMeshData();
		
	}
	//	smooth(0.1);
//  saveFormat("fitgfdckudoyufoufouf.off");
//	removeConnectedComponents(10,edge_min);
}


/** MODIFIED BY Andrea Romanoni
 Description: 
 - The goal of this method is to ensure that all the edges of the mesh are within the interval [epsilonMin,epsilonMax].
   In order to do so, edge collapses and edge split operations are performed.
 - The method also attempts to fix degeneracies by invoking fixDegeneracy(collapseRatio,degenerate_angle_deg) and performs some local smoothing, based on the operating mode.
 Parameters:
 - [epsilonMin, epsilonMax] - the desired edge interval
 - collapseRatio, degenerate_angle_deg - parameters used to invoke fixDegeneracy (see function for more details)
 - mode : 0 - fixDegeneracy=No  smoothing=Yes;
		  1 - fixDegeneracy=Yes smoothing=Yes; (default)
		 10 - fixDegeneracy=Yes smoothing=No;
 - max_iter (default=30) - maximum number of iterations to be performed; since there is no guarantee that one operations (such as a collapse, for example)
   will not in turn generate new degeneracies, operations are being performed on the mesh in an iterative fashion. 
 */
void Mesh::ensureEdgeSizesWOupdate ( double epsilonMin, double epsilonMax, double collapseRatio,double degenerate_angle_deg, int mode, int max_iters)
{
	int no_ops=1;
	int iters=0;
	//	int max_iters=30;
	int smallEdges=1;
	
	//mode
	bool invokeFixes=true;
	
	queue<Vertex::Halfedge_handle> bigEdgesQueue;
	queue<Vertex::Halfedge_handle> smallEdgesQueue;
	
	if ( mode>0 )
	{
		fixDegeneracy ( collapseRatio, degenerate_angle_deg );
	}
	
	computeMeshStats();
	cout.precision ( 8 );
	
#if DEBUG_MESH>0
	cout << "ENSURING EDGE_SIZE IN {" << epsilonMin << "," << epsilonMax << "} (where edges in {" << edge_min << "," << edge_max << "} )" << endl;
#endif	
	while ( ( ( ( edge_min<epsilonMin ) ) || ( edge_max>epsilonMax ) ) && ( no_ops>0 ) && ( iters<max_iters ) )
	{
		iters++;
		no_ops=0;
		double currentEpsilonMax = max ( edge_max / 2 ,epsilonMax );
		currentEpsilonMax = epsilonMax;
		
		//process big edges
		for ( Halfedge_iterator h = p.edges_begin(); h != p.edges_end(); ++h )
		{
			if ( ( edge_size ( h ) > currentEpsilonMax )) { 
				bigEdgesQueue.push ( h );
			}
		}
#if DEBUG_MESH>1
		cout << "added big edges=" << bigEdgesQueue.size() << endl;
#endif
		while ( !bigEdgesQueue.empty() ) {
			Vertex::Halfedge_handle h = bigEdgesQueue.front(); bigEdgesQueue.pop();
			float avg_mean_curv = max(abs(h->vertex()->mean_curvature()),abs(h->prev()->vertex()->mean_curvature()));
			if ( (h->vertex()->getVisibility())  && (h->prev()->vertex()->getVisibility()) && canSplitEdge ( h ) && (avg_mean_curv > 0.1)) {splitEdge ( h ); no_ops++; }
			
		}
		
		// process small edges
		Facet_iterator f = p.facets_begin();
		Facet_handle old_f;
		while ( f !=p.facets_end() ) {
			old_f = f;
			f++;
			int oldSmallEdges=smallEdges;
			double minEdge = old_f->edgeStatistics ( 0 );
			if ( minEdge < epsilonMin )
			{
				HF_circulator h = old_f->facet_begin();
				do
				{
					if ( h->vertex()->getVisibility() && ( edge_size ( h ) == minEdge ) && canCollapseEdge ( h ) )
					{
						no_ops++;
						while ( ( f!=p.facets_end() ) && ( ( f==h->opposite()->facet() ) || ( f==h->facet() ) ) )
						{
							f++;
						}
						collapseEdge ( h );
						break;
					}
				}
				while ( ++h==old_f->facet_begin() );
			}
		}
		
		
		
#if DEBUG_MESH>1
		cout << "delete small edges=" << smallEdges << endl;
#endif
		
		//		if (invokeFixDegen) fixAllDegeneracy(collapseRatio, degenerate_angle_deg);
		//		improveVertexValence();
		//		improveVertexValence();
		//		fixDegeneracy(collapseRatio, degenerate_angle_deg);
		if ( mode==10 )
		{
			improveVertexValence ( 2 );
			fixDegeneracy ( collapseRatio, degenerate_angle_deg );
		}
		else
			if ( mode <10 ) improveVertexValence ( 2 );
		
		//		computeMeshStats();
		updateMeshData(false,false);
		if ( mode <10 ) smooth ( 0.1,1);
		
	}
#if DEBUG_MESH>0
	cout << "no_ops=" << no_ops << " and iters=" << iters << endl;
	cout << "EDGE_SIZE IN {" << edge_min << "," << edge_max << "} before fixAllDegeneracy" << endl;
#endif	
	if ( ( mode > 0 ) && ( iters==max_iters ) )
	{
		
		fixAllDegeneracy ( collapseRatio, degenerate_angle_deg );
#if DEBUG_MESH>0				
		cout << "EDGE_SIZE IN {" << edge_min << "," << edge_max << "}" << endl;
#endif	
		//	computeMeshStats();
		if ( mode <10 ) smooth ( 0.1,1 );
		updateMeshData(false,false);
		
	}
	//	smooth(0.1);
	saveFormat("fitgfdckudoyufoufouf.off");
	removeConnectedComponents(10,edge_min);
}


void Mesh::ensureEdgeSizes()
{
	ensureEdgeSizes ( 0, edge_max/4, 0.2, 150 );//edge_avg
}




	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::createMeshFromPoints ( const char *filename )
	{
		
		/*
		 std::ifstream is(filename);
		 int n;
		 is >> n;
		 
		 KCH::Point_3 point;
		 vector<KCH::Point_3> points;
		 
		 
		 std::cout << n << " points read" << std::endl;
		 if (n <0) {
		 cout << "no of points is negative =  " << n << endl;
		 return;
		 }
		 for( ; n>0 ; n--) {
		 is >> point;
		 //		dt.insert(point);
		 points.push_back(point);
		 }
		 
		 // define object to hold convex hull
		 CGAL::Object ch_object;
		 
		 // compute convex hull
		 CGAL::convex_hull_3(points.begin(), points.end(), ch_object);
		 
		 // determine what kind of object it is
		 Segment_3 segment;
		 Polyhedron polyhedron;
		 if ( CGAL::assign(segment, ch_object) )
		 std::cout << "convex hull is a segment " << std::endl;
		 else if ( CGAL::assign (polyhedron, ch_object) )
		 std::cout << "convex hull is a polyhedron " << std::endl;
		 else
		 std::cout << "convex hull error!" << std::endl;
		 
		 Polyhedron p_new;
		 Build_New_Mesh<HalfedgeDS,KA> builder;
		 
		 vector<KA::Point_3> tr_points;
		 Alpha_shape_3::Cell_handle c;
		 int i;
		 
		 for(std::list<Alpha_shape_3::Facet>::iterator it = facets.begin(); it !=facets.end(); it++) {
		 tr_points.clear();
		 c = it->first;
		 i = it->second;
		 
		 if (as.classify(*it) == Alpha_shape_3::REGULAR && (as.classify(c) == Alpha_shape_3::INTERIOR)){
		 c = c->neighbor(i);
		 i = c->index(it->first);
		 }
		 
		 int incr =1;
		 if (it->second%2==1) incr=3;
		 for(int x=(i+incr)%4;x!=i;x=(x+incr)%4)
		 tr_points.push_back(c->vertex(x)->point());
		 builder.addTriangle(KA::Triangle_3(tr_points[0],tr_points[1],tr_points[2]));
		 
		 }
		 
		 
		 p_new.delegate( builder);
		 p = p_new;
		 */
		updateMeshData();
	}
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int Mesh::computeConnectedComponents ( bool colorVertices )
	{
		connected_components.clear();
		queue<Facet_handle> facet_queue;
		
		MeshConnectedComponent lastComponent;
		float color[3];
		
		//reset the facet status
		for ( Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i )
		{
			i->removal_status = 'U';
		}
		
		cout << "Connected components of :";
		//traverse the mesh via facets
		for ( Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i )
		{
			if ( i->removal_status=='U' )
			{ //start a new component
				lastComponent.setParams ( i,0 );
				i->removal_status='V'; // it is now visited
				facet_queue.push ( i );
				for ( int x=0;x<3;x++ ) color[x]=rand() *1.0/RAND_MAX;
				//			cout << " colors " << " " << color[0]  << " " << color[1] << " " << color[2] << endl;
				while ( !facet_queue.empty() )
				{ //fill the current component
					Facet_handle f = facet_queue.front(); facet_queue.pop();
					lastComponent.size++;
#if MVSTEREO_FACETS==1
					if ( colorVertices )
						f->set_color ( color[0],color[1],color[2] );
#endif
					HF_circulator h = f->facet_begin();
					do
					{
						if (h->is_border_edge()) continue;
						lastComponent.is_open=true;
						float edge_size=v_norm(h->vertex()->point() - h->prev()->vertex()->point());
						lastComponent.updateStats(edge_size);
						lastComponent.area+=f->area();
						if ( colorVertices ) h->vertex()->set_color(color[0],color[1],color[2]);
						Facet_handle opposite_f = h->opposite()->facet();
						if ( ( opposite_f!=Facet_handle() ) && ( opposite_f->removal_status=='U' ) )
						{
							opposite_f->removal_status='V'; // it is now visited
							facet_queue.push ( opposite_f );
						}
						
					}
					while ( ++h != f->facet_begin() );
				} //done traversing the current component
				lastComponent.edge_avg/=lastComponent.size*3;
				connected_components.push_back ( lastComponent );
				cout << lastComponent.size << " ";
			} // found a new component
		}//done traversing the mesh
		cout << " - total : "<< connected_components.size() << endl;
		return connected_components.size();
	}
	
	// Duplicate the CGAL remove connected component, since it crashes with borders !
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int Mesh::eraseConnectedComponent ( Facet_handle &f)
	{
		connected_components.clear();
		queue<Facet_handle> facet_queue;
		queue<Facet_handle> facet_erase_queue;
		
//		facet_queue.push ( hf->facet() );
//		hf->facet()->removal_status='D'; // marked for deletion
		facet_queue.push ( f );
		f->removal_status='D'; // marked for deletion

		while ( !facet_queue.empty() ) { //traverse the current component
			Facet_handle f = facet_queue.front(); facet_queue.pop();
			facet_erase_queue.push(f);
			HF_circulator h = f->facet_begin();
			do
			{
				if (h->is_border_edge()) continue;
				Facet_handle opposite_f = h->opposite()->facet();
				if ( ( opposite_f!=Facet_handle() ) && ( opposite_f->removal_status!='D' ) )
				{
					opposite_f->removal_status='D'; // marked for deletion
					facet_queue.push ( opposite_f );
				}
				
			}
			while ( ++h != f->facet_begin() );
		} //done traversing the current component

		while ( !facet_erase_queue.empty() ) {
			Facet_handle f = facet_erase_queue.front(); facet_erase_queue.pop();
			p.erase_facet(f->facet_begin());
		}
		return connected_components.size();
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::keepLargestConnectedComponent()
	{
		computeConnectedComponents ( false );
		int max_size=connected_components[0].size;
		int max_index=0;
		
		for ( int i=0;i<connected_components.size();i++ )
			if ( connected_components[i].size > max_size )
			{
				max_size=connected_components[i].size;
				max_index=i;
			}
		
		for ( vector<MeshConnectedComponent>::iterator vi=connected_components.begin(); vi!=connected_components.end(); )
		{
			if ( vi->size==max_size )
				vi++;
			else
			{
				lock();
				if ((vi->start_facet->halfedge()->is_border()==false)) {
					p.erase_connected_component ( vi->start_facet->facet_begin() );
					//eraseConnectedComponent(vi->start_facet );
					vi->start_facet = Facet_handle();
					vi = connected_components.erase ( vi );
					//updateMeshData();return;
				}
				else
					vi++;
	
				unlock();
			}
		}
		updateMeshData();
	}
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::removeConnectedComponents(int size_threshold, float edge_threshold)
	{
		computeConnectedComponents ( false );
		for ( vector<MeshConnectedComponent>::iterator vi=connected_components.begin(); vi!=connected_components.end(); )
		{
			if (( vi->size<=size_threshold) || ( vi->edge_max<=edge_threshold) || (( vi->area <edge_threshold*edge_threshold*PI*2) && (vi->is_open==false))) {
				lock();
				p.erase_connected_component ( vi->start_facet->facet_begin() );
				vi = connected_components.erase ( vi );
				unlock();
			}
			else
				vi++;
		}
		updateMeshData();
	}	
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::invert() {
		p.inside_out();
		updateMeshData();		
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::move(float dx, float dy, float dz) {
		Vector the_shift = Vector(dx,dy,dz);
		for ( Vertex_iterator v=p.vertices_begin();v!=p.vertices_end();v++ )
			v->move(the_shift);
		//v->point() = v->point() + the_shift;
		updateMeshData();
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::scale(float dx, float dy, float dz) {
		Vector the_shift;
		for ( Vertex_iterator v=p.vertices_begin();v!=p.vertices_end();v++ ) {
			v->move(Point(dx*v->point().x(),dy*v->point().y(),dz*v->point().z()) - v->point());
		}
		//v->point() = v->point() + the_shift;
		updateMeshData();
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::rotate(float angle_x_deg, float angle_y_deg, float angle_z_deg, bool meshCentered) {
		Affine_3 rx,ry,rz,r;
		double ax= angle_x_deg/180*PI;
		double ay= angle_y_deg/180*PI;
		double az= angle_z_deg/180*PI;
		
		rx = Affine_3(1,0,0,  0,cos(ax),sin(ax), 0,-sin(ax),cos(ax));
		ry = Affine_3(cos(ay),0,-sin(ay),0,1,0, sin(ay),0,cos(ay));
		rz = Affine_3(cos(az), sin(az),0,-sin(az),cos(az),0,0,0,1);
		
		r = rx*ry*rz;
		
		affine(r,meshCentered);		
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::affine(Affine_3 &rot, bool meshCentered) {
		float mx,my,mz;
		if (meshCentered) {
			float *bounds = getBoundingBox();
			mx = (bounds[3]-bounds[0])/2;
			my = (bounds[4]-bounds[1])/2;
			mz = (bounds[5]-bounds[2])/2;
			move(mx,my,mz);
		}
		
		for ( Vertex_iterator v=p.vertices_begin();v!=p.vertices_end();v++ ) {
			v->move(rot.transform(v->point()) - v->point());
		}
		if (meshCentered) {
			move(-mx,-my,-mz);
		}
	}
	
	// It replaces the current mesh with the provided one in the space constrainted by the current bounding box
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::replaceWith(Polyhedron &other, float relative_scale) {
		float* bbox=getBoundingBox();
		// Apply affine transformation to obtain initial configuration
		Kernel::Vector_3 rel_transl((bbox[0]+bbox[3])/2,(bbox[1]+bbox[4])/2,(bbox[2]+bbox[5])/2);
		float rel_scale = relative_scale*std::max(std::max((-bbox[0]+bbox[3])/2,(-bbox[1]+bbox[4])/2),(-bbox[2]+bbox[5])/2);
		Aff_trans A = Aff_trans(CGAL::Translation(), rel_transl) * Aff_trans(CGAL::Scaling(), rel_scale );
		p = other;
		std::transform( p.points_begin(), p.points_end(), p.points_begin(), A);
		updateMeshData();
	}
	


//////////////////////////////////////////////////////////////////////////////////////////////////////
// A modifier creating a triangle with the incremental builder.
template <class HDS, class K>
class Mesh_Joiner : public CGAL::Modifier_base<HDS> {
private:
	Mesh& other_mesh;	
public:
	////////////////////////////////////////////////////////////////////////////////////
	Mesh_Joiner() {}
	Mesh_Joiner(Mesh& other_mesh_):other_mesh(other_mesh_)
	{
		other_mesh=other_mesh_;
	}
	
	void operator() ( HDS& hds ) {

		CGAL::Polyhedron_incremental_builder_3<HDS> B ( hds, true );
		B.begin_surface ( other_mesh.p.size_of_vertices(), other_mesh.p.size_of_facets() );
		
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;
		int counter=0;

		for ( Vertex_iterator vi=other_mesh.p.vertices_begin(); vi!=other_mesh.p.vertices_end();vi++ ) {
			Vertex_handle new_v = B.add_vertex ( vi->point() );
			new_v->transferData(*vi);
			vi->id=counter++;
		}

		for ( Facet_iterator fi=other_mesh.p.facets_begin(); fi!=other_mesh.p.facets_end();fi++ ) {
			
			B.begin_facet();
	
			HF_circulator c = fi->facet_begin();
			HF_circulator d = c;
			float w=0;
			CGAL_For_all ( c, d ) {
				//B.add_vertex_to_facet ( hds.size_of_vertices()-other_mesh.p.size_of_vertices()+c->vertex()->id -1);
				B.add_vertex_to_facet (c->vertex()->id);
			}
	/*		B.add_vertex_to_facet ( hds.size_of_vertices()-3 );
			B.add_vertex_to_facet ( hds.size_of_vertices()-2 );
			B.add_vertex_to_facet ( hds.size_of_vertices()-1 );
	*/		B.end_facet();
		}
		B.end_surface();
	}
}; //Mesh_Joiner




	//////////////////////////////////////////////////////////////////////////////////////////////////////
	void Mesh::unionWith(Mesh &other) {

		Mesh_Joiner<HalfedgeDS,Kernel> builder(other);
		//Polyhedron p_new;
		p.delegate ( builder );
		updateMeshData();
		return;

	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	


