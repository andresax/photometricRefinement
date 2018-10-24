#ifndef CEP_INTERSECTION_SEGMENT_3_SEGMENT_3_H
#define CEP_INTERSECTION_SEGMENT_3_SEGMENT_3_H

#include <CGAL/Segment_3.h>
#include <CGAL/Object.h>


namespace CEP {
namespace intersection {

    using CGAL::Segment_3;


//! True if the segments intersect.
template <class R>
bool
collinear_do_intersect( const Segment_3<R>& s1, 
			const Segment_3<R>& s2 );


//! Returns the point or segment of intersection.
template <class R>
CGAL::Object
collinear_intersection( const Segment_3<R>& s1, 
			const Segment_3<R>& s2 );


//! True if the segments intersect.
template <class R>
bool
coplanar_do_intersect( const Segment_3<R>& s1,
		       const Segment_3<R>& s2 );


//! True if the segments intersect.
template <class R>
bool
do_intersect( const Segment_3<R>& s1,
	      const Segment_3<R>& s2 );


//! Returns the point or segment of intersection.
template <class R>
CGAL::Object
intersection( const Segment_3<R>& s1,
	      const Segment_3<R>& s2 );


}
}


#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Segment_3_Segment_3.C>
#endif

#endif
