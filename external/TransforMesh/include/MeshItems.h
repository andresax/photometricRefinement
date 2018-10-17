#ifndef MESH_ITEMS_H
#define MESH_ITEMS_H

#include <CGAL/Polyhedron_3.h>
#include "MeshVertex.h"
#include "MeshFacet.h"

struct MeshItems : public CGAL::Polyhedron_items_3 {
    template <class Refs, class Traits>
    struct Vertex_wrapper {
        typedef typename Traits::Point_3  Point;
        typedef typename Traits::Vector_3 Normal;
        typedef MeshVertex<Refs, CGAL::Tag_true, Point, Normal> Vertex;
    };
    template <class Refs, class Traits>
    struct Face_wrapper {
        typedef typename Traits::Vector_3 Normal;
        typedef MeshFacet<Traits, Refs, CGAL::Tag_true, Normal> Face;
    };
};

#endif