#ifndef CEP_INTERSECTION_SEGMENT_3_POINT_3_H
#define CEP_INTERSECTION_SEGMENT_3_POINT_3_H

#include <CGAL/Object.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/predicates_on_points_3.h>


namespace CEP {
namespace intersection {


    using CGAL::Segment_3;
    using CGAL::Point_3;


//! True if point p lies on the segment s.
/*! \ingroup grp_do_intersect
 */
template <class R>
inline
bool
do_intersect( const Segment_3<R>& s, 
	      const Point_3<R>& p )
{
    return s.has_on(p);
}


//! Compute the point of intersection
/*! \ingroup grp_intersection
 */
template <class R>
inline
CGAL::Object
intersection( const Segment_3<R>& s, 
	      const Point_3<R>& p )
{
    if ( do_intersect(s,p) )
	return CGAL::make_object(p);
    return CGAL::Object();
}


//! True if the point is collinear with the segment.
template <class R>
inline 
bool
collinear( const Segment_3<R>& s, 
	   const Point_3<R>& p )
{
    return CGAL::collinear( s.source(), s.target(), p );
}



}
}

#endif
