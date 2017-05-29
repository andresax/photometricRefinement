/*
 * MeshBuilder.h
 *
 *  Created on: 20 feb 2025
 *      Author: andrea
 */

#ifndef INCLUDE_INCREMENTAL_DENSE_RECONSTRUCTION_PHOTOMETRIC_REFINEMENT_MESHBUILDER_H_
#define INCLUDE_INCREMENTAL_DENSE_RECONSTRUCTION_PHOTOMETRIC_REFINEMENT_MESHBUILDER_H_

#include <CGAL/HalfedgeDS_halfedge_base.h>
#include <CGAL/Modifier_base.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <Mesh.h>
#include <cstddef>
#include <map>
#include <queue>
#include <vector>



#define MESH_BUILDER_HASHING 1

//////////////////////////////////////////////////////////////////////////////////////////////////////
// A modifier creating a triangle with the incremental builder.
template<class HDS, class K>
class MeshBuilder : public CGAL::Modifier_base<HDS> {
private:
  std::vector<typename K::Triangle_3> new_triangles;
  std::vector<Facet_handle> new_triangles_orig_facet;

  std::vector<Vertex_handle> vertex_orig_vertex_map;
  std::map<typename K::Point_3, int> vertex_map;

  //additional structures required in order to be able to perform vertex splits
  std::vector<std::vector<int> > vertex_triangle_map; // vertex triangle iterator
  std::vector<typename K::Point_3> vertices;
  std::vector<bool> vertices_new_flag;
  std::vector<bool> triangle_status;
  std::vector<std::vector<int> > triangle_vertex_map; //triangle vertex iterator

  int idFromNew;

  bool error;
  bool copy_original_facet_data;

  // assumes that the additional data structures have been initialized
  bool _findTriangleWithinVertex(int vertex_id, int & triangle_id, int & other_vertex_id_1, int & other_vertex_id_2) {
    triangle_id = -1;
    for (int t = 0; t < vertex_triangle_map[vertex_id].size(); t++)
      if (triangle_status[vertex_triangle_map[vertex_id][t]] == false) {
        triangle_id = vertex_triangle_map[vertex_id][t];
        triangle_status[triangle_id] = true;
        for (int i = 0; i < 3; i++)
          if (triangle_vertex_map[triangle_id][i] == vertex_id) {
            other_vertex_id_1 = triangle_vertex_map[triangle_id][(i + 1) % 3];
            other_vertex_id_2 = triangle_vertex_map[triangle_id][(i + 2) % 3];
            return true;
          }
      }
    if (triangle_id != -1) {
      cout << "_findTriangleWithinVertex failed !" << endl;
    }
    return false;

  }
  bool _triangleContainsVertex(int triangle_id, int vertex_id) {
    for (int v = 0; v < 3; v++)
      if (triangle_vertex_map[triangle_id][v] == vertex_id)
        return true;
    return false;
  }

  bool _chooseNextTriangle(int vertex_id, int triangle_id, int other_vertex_id, int & new_triangle_id, int & new_other_vertex_id) {
    new_triangle_id = -1;
    for (int t = 0; t < vertex_triangle_map[other_vertex_id].size(); t++)
      if ((triangle_status[vertex_triangle_map[other_vertex_id][t]] == false) && _triangleContainsVertex(vertex_triangle_map[other_vertex_id][t], vertex_id)) {
        new_triangle_id = vertex_triangle_map[other_vertex_id][t];
        triangle_status[new_triangle_id] = true;
        for (int i = 0; i < 3; i++)
          if ((triangle_vertex_map[new_triangle_id][i] != vertex_id) && (triangle_vertex_map[new_triangle_id][i] != other_vertex_id)) {
            new_other_vertex_id = triangle_vertex_map[new_triangle_id][i];
            return true;
          }
      }
    if (new_triangle_id != -1) {
      cout << "_chooseNextTriangle failed !" << endl;
    }
    return false;
  }

public:
  ////////////////////////////////////////////////////////////////////////////////////
  MeshBuilder() {
    error = false;
    copy_original_facet_data = true;
    idFromNew = 0;
  }

  void reset() {
    //std::cout << "new_triangles_orig_facet " << std::endl;
    vector<Facet_handle>().swap(new_triangles_orig_facet);
    //std::cout << "new_triangles " << std::endl;
    //std::cout << "new_triangles.size() "<<new_triangles.size() << std::endl;
    vector<typename K::Triangle_3>().swap(new_triangles);
    //std::cout << "vertex_orig_vertex_map " << std::endl;
    vector<Vertex_handle>().swap(vertex_orig_vertex_map);
    //std::cout << "vertex_triangle_map " << std::endl;
    vector<std::vector<int>>().swap(vertex_triangle_map);
    //std::cout << "vertices " << std::endl;
    vector<typename K::Point_3>().swap(vertices);
   // std::cout << "vertices_new_flag " << std::endl;
    vector<bool>().swap(vertices_new_flag);
    //std::cout << "triangle_status " << std::endl;
    vector<bool>().swap(triangle_status);
    //std::cout << "vertex_map " << std::endl;
    vertex_map.clear();
    //std::cout << "triangle_vertex_map " << std::endl;
    std::vector<std::vector<int> >().swap(triangle_vertex_map);
    //std::cout << "END " << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////////
  void newfromNow() {
    idFromNew = new_triangles.size();
  }
  ////////////////////////////////////////////////////////////////////////////////////
  void addTriangle(typename K::Triangle_3 tmpT) {
    new_triangles.push_back(tmpT);
    copy_original_facet_data = false;
  }

  ////////////////////////////////////////////////////////////////////////////////////
  void addTriangle(typename K::Triangle_3 tmpT, Triangle_Job &job) {

    if (job.updated_norm_factor == 1) {
      new_triangles.push_back(tmpT);

      for (int i = 0; i < 3; i++) {
        if (vertex_map.find(tmpT[i]) == vertex_map.end()) {
          //              vertex_map[tmpT[i]] = vertex_id++;
          //              vertex_orig_vertex_map[tmpT[i]] = job.facet->
        }
      }

    } else {
      //    cout << "Added inverted element" << endl;
      //new_triangles.push_back(tmpT);
      new_triangles.push_back(typename K::Triangle_3(tmpT[2], tmpT[1], tmpT[0]));
      //    cout << "Added triangle " << new_triangles.size()-1 << " with normal_factor=" << job.updated_norm_factor << endl;
      //    new_triangles.push_back(Triangle_3(tmpT[2],tmpT[1],tmpT[0]));

    }
    new_triangles_orig_facet.push_back(job.facet);

    //  cout << "Now " << new_triangles.size() << " ==> " << tmpT << endl;

  }

  void operator()(HDS& hds) {
#if DEBUG_MESH>0
    cout << "STITCHING TOGETHER THE NEW MESH "; flush ( cout );
#endif
#if MESH_BUILDER_HASHING==1

    std::vector<int> tmpVector;

    // build the map with all the vertices
    int vertex_id = 0;

    for (typename std::vector<typename K::Triangle_3>::iterator it = new_triangles.begin(); it != new_triangles.end(); it++) {
      for (int i = 0; i < 3; i++) {
        if (vertex_map.find(it->vertex(i)) == vertex_map.end()) {
          vertex_map[it->vertex(i)] = vertex_id++;
        }
      }
    }

    // Need to create additional data structures in order to
    //   process each vertex and split the doberman ears like structures,
    //   where a vertex connects two separated components.

    triangle_status.reserve(new_triangles.size());
    triangle_vertex_map.reserve(new_triangles.size());
    vertex_triangle_map.reserve(vertex_map.size());
    vertices.reserve(vertex_id);
    vertices_new_flag.reserve(vertex_id);

    //create structures
    for (typename std::vector<typename K::Triangle_3>::iterator it = new_triangles.begin(); it != new_triangles.end(); it++) {
      triangle_vertex_map.push_back(tmpVector);
    }

    for (typename std::map<typename K::Point_3, int>::iterator v_it = vertex_map.begin(); v_it != vertex_map.end(); v_it++) {
      vertices.push_back(new_triangles[0].vertex(0));
      vertices_new_flag.push_back(false);
      vertex_triangle_map.push_back(tmpVector);
    }

    //populate structures
    for (typename std::map<typename K::Point_3, int>::iterator v_it = vertex_map.begin(); v_it != vertex_map.end(); v_it++) {
      vertices[v_it->second] = v_it->first;
    }
    for (int t = 0; t < new_triangles.size(); t++) {
      triangle_status.push_back(false);
      for (int i = 0; i < 3; i++) {
        triangle_vertex_map[t].push_back(vertex_map[new_triangles[t].vertex(i)]);
        vertex_triangle_map[vertex_map[new_triangles[t].vertex(i)]].push_back(t);
      }
    }
    int old_vertices_size = vertices.size();
    int singular_vertex = 0;
    for (int v = 0; v < old_vertices_size; v++) {
      // set all triangles as false
      for (int t = 0; t < vertex_triangle_map[v].size(); t++)
        triangle_status[vertex_triangle_map[v][t]] = false;
      int triangle_count = vertex_triangle_map[v].size();

      int cur_triangle_id, next_triangle_id, next_triangle_id_orig;
      int cur_other_vertex_id, next_other_vertex_id, next_other_vertex_id_1, next_other_vertex_id_2;
      bool first_time = true;
      std::queue<int> trianglesForNewVertex;
      int new_v = -1;
      while (_findTriangleWithinVertex(v, next_triangle_id_orig, next_other_vertex_id_1, next_other_vertex_id_2)) {
        if (!first_time) {
          singular_vertex++;
          vertices.push_back(vertices[v]);
          vertices_new_flag[v] = true;
          vertices_new_flag.push_back(true); // it is a new vertex - move it a bit
          new_v = vertices.size() - 1;
          vertex_triangle_map.push_back(tmpVector);
        }
        //      cout << "seeder vertex: " << v;

        //it makes sense to traverse in both directions for open surfaces!
        // 1st direction
        next_other_vertex_id = next_other_vertex_id_1;
        next_triangle_id = next_triangle_id_orig;
        do {
          cur_triangle_id = next_triangle_id;
          cur_other_vertex_id = next_other_vertex_id;
          if (new_v != -1) {
            trianglesForNewVertex.push(cur_triangle_id);
          }
        } while (_chooseNextTriangle(v, cur_triangle_id, cur_other_vertex_id, next_triangle_id, next_other_vertex_id));
        // 2nd direction
        next_other_vertex_id = next_other_vertex_id_2;
        next_triangle_id = next_triangle_id_orig;
        do {
          cur_triangle_id = next_triangle_id;
          cur_other_vertex_id = next_other_vertex_id;
          if (new_v != -1) {
            trianglesForNewVertex.push(cur_triangle_id);
          }
        } while (_chooseNextTriangle(v, cur_triangle_id, cur_other_vertex_id, next_triangle_id, next_other_vertex_id));

        // we need to move all the triangles to its new papa vertex
        while (!trianglesForNewVertex.empty()) {
          cur_triangle_id = trianglesForNewVertex.front();
          trianglesForNewVertex.pop();
          vertex_triangle_map[new_v].push_back(cur_triangle_id);

          std::vector<int> &localVector = vertex_triangle_map[v];

          localVector.erase(remove_if(localVector.begin(), localVector.end(), bind2nd(equal_to<int>(), cur_triangle_id)), localVector.end());

          for (int ii = 0; ii < 3; ii++)
            if (triangle_vertex_map[cur_triangle_id][ii] == v)
              triangle_vertex_map[cur_triangle_id][ii] = new_v;
        }
        first_time = false;
      }
    }
#if DEBUG_MESH>0
    cout << "(found " << singular_vertex << " singular vertices)." << endl;
#endif
    // Postcondition: `hds' is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, false);
    B.begin_surface(vertex_map.size(), new_triangles.size(), 6);

    for (int i = 0; i < vertices.size(); i++) {
      Vertex_handle v = B.add_vertex(FROM_POINT3_EXACT(vertices[i]));
      // std::cout<<"idFromNew= "<<idFromNew<<" vertex_triangle_map[i][0] = "<<vertex_triangle_map[i][0]<<std::endl;
//      if ( vertices_new_flag[i] ) v->flag[0]=true;
      if (vertex_triangle_map[i][0] >= idFromNew) {
        v->flag[3] = true;
      } else
        v->flag[3] = false;
      //v->flag[0]=true;
    }
    typedef typename HDS::Vertex Vertex;
    typedef typename Vertex::Point Point;
    for (int t = 0; t < triangle_vertex_map.size(); t++) {
      // test if adding this facet violates the manifold constraints
      std::vector<std::size_t> newFacetIdx;
      for (int i = 0; i < 3; i++)
        newFacetIdx.push_back(triangle_vertex_map[t][i]);

      Halfedge_handle h = NULL;
      if (B.test_facet(newFacetIdx.begin(), newFacetIdx.end()))
        h = B.add_facet(newFacetIdx.begin(), newFacetIdx.end());
      else
      // cout << "\r WARNING: facet " << t << " violates the manifold constraint (ignoring it)" << endl;

      /*
       B.begin_facet();
       // copy information from parent mesh
       for ( int i=0;i<3;i++ )
       B.add_vertex_to_facet ( triangle_vertex_map[t][i] );
       Halfedge_handle h = B.end_facet();

       */
      if ((h != NULL) && copy_original_facet_data) {
#if MVSTEREO_FACETS==1
        h->facet()->weight_priors = new_triangles_orig_facet[t]->weight_priors;
        h->facet()->weight = new_triangles_orig_facet[t]->weight;
#endif
        Halfedge_handle orig_h = new_triangles_orig_facet[t]->facet_begin();
        h->facet()->removal_status = new_triangles_orig_facet[t]->removal_status;
        for (int i = 0; i < 3; i++) {
          if (vertices[triangle_vertex_map[t][i]] == TO_POINT3_EXACT(new_triangles_orig_facet[t]->get_point(i))) {
            (h->vertex())->transferData(*(orig_h->vertex()));
          } else {
            h->vertex()->weight = 0.5;
            //              cout << vertices[triangle_vertex_map[t][i]] << "!=" << new_triangles_orig_facet[t]->get_point(i) <<endl;
          }
          h = h->next();
          orig_h = orig_h->next();
        }

      }
    }

    if (B.check_unconnected_vertices()) {
      cout << "There are unconnected vertices!" << endl;
      B.remove_unconnected_vertices();
    }

    if (B.error()) {
      cout << "Error encountered while building the surface with hashin. Undoing operation. Rebuilding without hashing." << endl;
      B.rollback();
      B.begin_surface(new_triangles.size() * 3, new_triangles.size(), 6);
      for (typename std::vector<typename K::Triangle_3>::iterator it = new_triangles.begin(); it != new_triangles.end(); it++) {

        B.add_vertex(FROM_POINT3_EXACT(it->vertex(0)));
        B.add_vertex(FROM_POINT3_EXACT(it->vertex(1)));
        B.add_vertex(FROM_POINT3_EXACT(it->vertex(2)));
        B.begin_facet();
        B.add_vertex_to_facet(hds.size_of_vertices() - 3);
        B.add_vertex_to_facet(hds.size_of_vertices() - 2);
        B.add_vertex_to_facet(hds.size_of_vertices() - 1);
        B.end_facet();
      }
      B.end_surface();
    } else
      B.end_surface();
  }
#else
  cout << "Mesh Builder WITHOUT Hashing invoked" << endl;
  // Postcondition: `hds' is a valid polyhedral surface.
  CGAL::Polyhedron_incremental_builder_3<HDS> B ( hds, true );
  B.begin_surface ( new_triangles.size() *3, new_triangles.size() );

  typedef typename HDS::Vertex Vertex;
  typedef typename Vertex::Point Point;
  for ( std::vector<Triangle_3>::iterator it=new_triangles.begin(); it!=new_triangles.end();it++ )
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
};
//MeshBuilder

#endif /* INCLUDE_INCREMENTAL_DENSE_RECONSTRUCTION_PHOTOMETRIC_REFINEMENT_MESHBUILDER_H_ */
