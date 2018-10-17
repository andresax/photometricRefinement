#ifndef CEP_INTERSECTION_SEGMENT_3_TRIANGLE_3_H    // -*- C++ -*-
#define CEP_INTERSECTION_SEGMENT_3_TRIANGLE_3_H

#include <CGAL/Object.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_3.h>


namespace CEP {
namespace intersection {

    using CGAL::Segment_3;
    using CGAL::Triangle_3;


    //! True if the segment and triangle intersect.
    template <class R>
    bool
    coplanar_do_intersect( const Segment_3<R>& segment, 
			   const Triangle_3<R>& triangle );


    //! Returns the point or segment of intersection.
    template <class R>
    CGAL::Object
    coplanar_intersection( const Segment_3<R>& segment, 
			   const Triangle_3<R>& triangle );


    //! True if the segment and triangle intersect.
    template <class R>
    bool
    do_intersect( const Segment_3<R>& segment, 
		  const Triangle_3<R>& triangle );


    //! Returns the point or segment of intersection.
    template <class R>
    CGAL::Object
    intersection( const Segment_3<R>& segment, 
		  const Triangle_3<R>& triangle );


}
}


#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Segment_3_Triangle_3.C>
#endif

#endif
