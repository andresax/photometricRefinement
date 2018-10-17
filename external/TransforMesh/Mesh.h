/*
 *  Mesh.h
 *  src
 *
 *  Created by Andrei Zaharescu on 28/11/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef TRANSFORMESH_H
#define TRANSFORMESH_H

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

//#define USE_MESH_LOCKING

#ifdef USE_MESH_LOCKING
#include <QMutex>
#endif

#ifndef MAXFLOAT
#define MAXFLOAT ((float)3.40282346638528860e+38)    
#endif

#define ZERO_APPROX 1e-12


#include "MeshItems.h"


//#include "CImg_mod.h"
#include <CGAL/assertions.h>
#include <CGAL/Object.h>
#include <CGAL/box_intersection_d.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Polyhedron_3.h>

//#define TO_FLOAT(A) A
#define TO_FLOAT(A) (float)CGAL::to_double(A)
#define SQUARED(A) (A)*(A)

typedef CGAL::Polyhedron_3<Kernel, MeshItems>          Polyhedron;
typedef Polyhedron::HalfedgeDS                         HalfedgeDS;
typedef Polyhedron::Vertex                             Vertex;
typedef Polyhedron::Facet                              Facet;
typedef Polyhedron::Halfedge                           Halfedge;
typedef Polyhedron::Vertex_iterator                    Vertex_iterator;
typedef Polyhedron::Facet_iterator                     Facet_iterator;
typedef Polyhedron::Facet_const_iterator               Facet_const_iterator;

typedef Polyhedron::Point_iterator                     Point_iterator;
typedef Polyhedron::Edge_iterator                      Edge_iterator;
typedef Polyhedron::Halfedge_iterator                  Halfedge_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator   HF_circulator;
typedef Polyhedron::Halfedge_around_vertex_circulator  HV_circulator;

typedef Polyhedron::Facet_const_handle                 Facet_const_handle;
typedef Polyhedron::Halfedge_const_handle              Halfedge_const_handle;
typedef Vertex::Halfedge_handle			               Halfedge_handle;
typedef Vertex::Facet_handle                           Facet_handle;
typedef Vertex::Vertex_handle                          Vertex_handle;

typedef CGAL::Box_intersection_d::Box_with_handle_d<double, 3, Facet_handle>  Box;
typedef Kernel::Point_3 Point_d;
typedef CGAL::Search_traits_3<Kernel> SearchTreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<SearchTreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Neighbor_search_tree;
typedef Kernel::Aff_transformation_3 Affine_3;

using namespace std;

// *********************************************************************************************************
// Helper class used in self-intersection removal
// *********************************************************************************************************
class Triangle_Job {
public:
    Facet_handle facet;
    CDT::Face_handle start_subfacet; //will be set by a _findStartupCDTriangle
    KernelExact::Vector_3 updated_norm;
    int updated_norm_factor;

    KernelExact::Segment_3 entrance_segment;
    KernelExact::Vector_3 entrance_norm;
    KernelExact::Point_3 entrance_opposite_point;
    
//KernelExact::Vector_3 entrance_norm;
//    Vector      updated_norm;
   Triangle_Job(Facet_handle theHandle) {facet = theHandle; updated_norm_factor=1;}
};

// *********************************************************************************************************
// Helper class used to for computation of connected components
// *********************************************************************************************************
class MeshConnectedComponent {
public:
	Facet_handle start_facet;
	int size;
	float area;
	float edge_min, edge_avg, edge_max;
	bool is_open;
	MeshConnectedComponent(){
		start_facet = NULL;
		size=0;
		edge_min=MAXFLOAT;
		edge_max=0;
		edge_avg=0;
		area=0.0f;
		is_open=false;
	}
	void setParams(Facet_handle start_facet_, int size_){
		start_facet = start_facet_;
		size = size_;
	}
	void updateStats(float edge_size) {
		edge_max=std::max(edge_max,edge_size);
		edge_min=std::min(edge_min,edge_size);
		edge_avg+=edge_size;
	}
};

// *********************************************************************************************************
// Main Mesh class - should be eventually broken into smaller sub-classes that are generic
// *********************************************************************************************************
class Mesh {
public:
	enum CurvatureComputationMethod {Curv_Meyer=1,Curv_Dong=2,Curv_Approx=3,Geo_Euclid_Ratio=4};
	enum QualityComputationMethod {Qual_No_Computation=0, Qual_Color=1, Qual_Color_Deriv=2, Qual_Mean_Curv=3, Qual_Mean_Curv_Deriv=4, Qual_Gaussian_Curv=5, Qual_Gaussian_Curv_Deriv=6, Qual_Quality_Deriv=7, Qual_Imported=8};
	enum MeshRemeshingMethod {Remesh_Sqrt3=1,Remesh_Loop=2,Remesh_CatmullClark=3,Remesh_DooSabin=4};	
	enum NoiseMode {Colour=0,Geometry=1};
	enum MeshReturnCodes {MeshFileNotFound, MeshNonManifold, MeshOK};	
	
	Mesh();
	Mesh(const Mesh& mesh_copy);
	~Mesh();
	const Mesh& operator=(const Mesh& x);

	void clear(bool cleanFilenames=true);
	float* getBoundingBox();


	
	MeshReturnCodes loadFormat(const char *filename,bool rememberFilename);
	bool loadAsTriangleSoup(const char *filename);
	bool saveFormat(const char *filename); // file type is inferred from extension
	bool saveFormat(const char *filename, const char*file_type);
	bool loadVectorField(const char* filePattern, bool rememberFilename);
	bool saveVectorField(const char* filePattern);	
	bool saveVertexIndices(const char* filePattern);
	
	void createMeshFromPoints(const char *filename);
	void mergeTriangleSoup();
	void reload(); 

	// Euler operations (topology modifying)
	bool canSplitEdge(Vertex::Halfedge_handle h);
	void makeHole(Vertex *v, int noRings);
	void splitEdge(Vertex::Halfedge_handle h, int mode=1); // 1 - middle, 2 - projection of the 3rd vertex
	bool canCollapseEdge(Vertex::Halfedge_handle h);
	bool canCollapseCenterVertex(Vertex::Vertex_handle v);
	void collapseEdge(Vertex::Halfedge_handle h);
	bool canFlipEdge(Vertex::Halfedge_handle h);
	void flipEdge(Vertex::Halfedge_handle h);
	void removeInvisibleFacets();
	void restrictToBoundingBox(float x1, float y1, float z1, float x2, float y2, float z2); 

	// mesh improvement
	void improveVertexValence(int mode=2);
	void fixDegeneracy(bool updateMesh=true); // removes elongated or wide triangles
	int fixDegeneracy(double collapseRatio,double degenerate_angle_deg);
	void fixAllDegeneracy(double collapseRatio,double degenerate_angle_deg);
	
	//connected components related
	int computeConnectedComponents(bool colorVertices=false);
	int eraseConnectedComponent (Facet_handle &f);
	void keepLargestConnectedComponent();
	void removeConnectedComponents(int size_threshold, float edge_threshold);
	
	//mesh set operations
	void invert();
	void move(float dx, float dy, float dz);
	void scale(float dx, float dy, float dz);
	void rotate(float angle_x_deg, float angle_y_deg, float angle_z_deg, bool meshCentered);
	void affine(Affine_3 &rot, bool meshCentered);
	void replaceWith(Polyhedron &other, float relative_scale=1);
	void unionWith(Mesh& other);
	
	
	bool isPointInsideTriangle(Kernel::Point_3 &the_point,Kernel::Triangle_3 &the_triangle);
	void computeDistanceFromVisualHull(Polyhedron& vh);
	
	Vertex::Vertex_handle closestVertex(Kernel::Point_3 &the_point);
	Kernel::Point_3 closestPoint(Kernel::Point_3 &the_point);	
	Kernel::Point_3 closestColorPoint(Kernel::Point_3 &the_point, float *givclr);	
	Vertex* vertexMapping(Kernel::Point_3 &the_point);
	Vertex* getVertexById(int id);
	
	Kernel::Vector_3 computeDistanceFromPoint(Kernel::Point_3 &the_point, int mode=0);

	float distanceTo(Mesh &other, bool reinit_search_structures=false);
	float getSurfacePercentageRadius(float percentage);
	void removeSelfIntersections();

	static std::vector< std::pair<Vertex*,int> > getRingNeighbourhood(std::vector<Vertex*> vertices,int ring_size, bool include_original);
	static std::vector< std::pair<Vertex*,int> > getRingNeighbourhood(Vertex& v,int ring_size, bool include_original=false);
	static std::vector< std::pair<Vertex*,float> > getGeodesicNeighbourhood(Vertex& v, float geo_size, bool include_original);
	

	
	double oppositeAngle(Vertex::Halfedge_handle h);

	void evolve(float sign=1);
	void diffuse(float amount=0.4,int mode=0);


	float getVolume(bool computeNormal=false);
	
	void dilate(double delta);
	void smooth(double delta, int mode=0, bool only_visible=false);	
	void convolve(double std_dev_ratio=1.25, bool add_to_history=false, bool print_stats=false);
	
	void bilateralSmooth(double sigma_c, double sigma_s);
	void perturb(double maxDelta);
	void remesh(int method=Remesh_Sqrt3, int steps=1);

	void displayInfo();

	void ensureEdgeSizes(double min_edge, double max_edge, double collapseRatio, double degenerate_angle_deg, int mode=1, int max_iters=30);
	void ensureEdgeSizes();
void ensureEdgeSizesWOupdate ( double epsilonMin, double epsilonMax, double collapseRatio,double degenerate_angle_deg, int mode, int max_iters);

	void create_center_vertex(Facet_iterator f);
	Point smooth_old_vertex(Vertex_handle v) const;
	void remeshWithThreshold(float edge_threshold=0);

	void setMeshQuality(QualityComputationMethod qual);
	void setMeshQualityViaLinearSystem(QualityComputationMethod qual);
	
	void resetVertexWeights();
	void computeVertexPriorWeights();
	void computeVertexWeights();
	void computeMeshStats();

	void computeVertexNormal(Vertex& v);
	void computeVertexRobustNormal(Vertex& v, float maxGeodesic);	
	void computeVertexConvolution(Vertex& v, float stddev);	
	void computeVertexCurvature(Vertex& v);
	void computeVertexLaplacian(Vertex& v);
	void computeVertexLaplacianDeriv(Vertex& v);
	void computeVertexQuality(Vertex& v, QualityComputationMethod qual);

	static float  computeVertexDirectionalGradient ( Vertex& v, Vertex& dir_u, QualityComputationMethod qual);
	static Vector computeVertexGradient ( Vertex& v, QualityComputationMethod qual);
	static Vector computeVertexGradientViaLinearSystem ( Vertex& v, QualityComputationMethod qual);

	static float computeVertex2ndPartialDeriv( Vertex& v, Vector &dir_1, Vector &dir_2);
	static bool isCorner(Vertex &v, float ratio_threshold);
	
	float computeVertexStatistics(Vertex& v, int mode);
	float getVertexMeanCurvature(Vertex& v);
	
	void generateRandomColors();

	void updateMeshData(bool bComputeCurvature=true, bool bComputeNormals=true);
	void resetSimplexIndices(bool bResetVertices = true, bool bResetFacets = true);
	
	void computeQualityPercentileBoundaries(float &lower_bound, float &upper_bound, float percentile=0.1, int bins=10000);
	void computeQualityDiffPercentileBoundaries(float &lower_bound, float &upper_bound, float percentile=0.1, int bins=10000);	

	Vector computeVectorComponent(Vector n, Vector v, int mode);

	vector<Facet*> getNeighbours(Facet *t);
	bool triangleIntersectsMesh(Facet *a);		
	bool doTrianglesIntersect(Triangle_3 a, Triangle_3 b);

	bool doFacetsIntersect(const Box *b, const Box *c, bool computeSegments=true);
	int checkIntersections(bool computeSegments=true);

	KernelExact::Segment_3* intersectTriangles(Triangle_3 a, Triangle_3 b);
	KernelExact::Segment_3* intersectTrianglesExact(Triangle_3 a, Triangle_3 b);
	
	void testStuff(float someParam);


	// locking
	inline void lock() {
	#ifdef USE_MESH_LOCKING
		_mutex.lock();
	#endif
	}
	inline void unlock() {
	#ifdef USE_MESH_LOCKING
		_mutex.unlock();
	#endif
	}

	// constrainted Delaunay triangulation
	CDT* getTriangulation(Vertex::Facet_handle f);

	Polyhedron p;
	// self intersection removal
	vector<KernelExact::Segment_3*> inter_segments;
	std::map <Kernel::Point_3, Vertex*>vertex_mapping;
	vector<Vertex*> id_mapping;
	
	float bounding_box[6]; //bounding box
	//seed triangle finding
	void *_AABB_tree;
	double close_color_sigma;

	//curvature
	int curv_comp_method;
	int curv_comp_neigh_rings;
	float curv_geo_size; 
	//stats
	double area_min, area_max, area_avg;
	double edge_min, edge_avg, edge_max;
	double laplacian_min, laplacian_avg, laplacian_max;
	double curvature_min, curvature_avg, curvature_std_dev, curvature_max;
	
	int no_intersections;
	
	char *loaded_filename, *loaded_file_type, *loaded_vect_filename;
	//std::string loaded_filename;

	int interactive_colors_max_facet_no;
	
	QualityComputationMethod qual_mode;
	
private:

	int tmpFix;
	#ifdef USE_MESH_LOCKING	
	QMutex _mutex;
	#endif

	bool is_search_init;
	Neighbor_search_tree search_tree;
	bool is_mapping_init;
	
	vector<MeshConnectedComponent> connected_components;
	
	// helper methods for removeSelfIntersection
	void _updateNormal(Triangle_Job &job);
	Facet_handle _getSeedTriangle();
	bool _findStartupCDTriangle(Triangle_Job &job);
	
	float fRobustNormalMaxDistance;

};

#endif
