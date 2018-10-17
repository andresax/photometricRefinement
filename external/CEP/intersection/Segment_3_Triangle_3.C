#include <CEP/intersection/Segment_3_Segment_3.h>
#include <CEP/intersection/Segment_3_Triangle_3.h>
#include <CEP/intersection/Triangle_3_Point_3.h>
#include <CEP/intersection/Halfplane_3.h>

#include <CGAL/Plane_3.h>
#include <CGAL/predicates_on_points_3.h>
#include <CGAL/intersection_3.h>


namespace CEP {  
namespace intersection {

    using CGAL::Segment_3;
    using CGAL::Triangle_3;
    using CGAL::Plane_3;

    using CGAL::assign;
    using CGAL::orientation;


    /*! \ingroup grp_coplanar_do_intersect
     *
     * \pre triangle is not degenerate
     * \pre Segment is coplanar with triangle
     */
    template <class R>
    bool
    coplanar_do_intersect( const Segment_3<R>& segment, 
			   const Triangle_3<R>& triangle )
    {
	CGAL_exactness_precondition( !triangle.is_degenerate() );
	CGAL_exactness_precondition( CGAL::coplanar( triangle[0],
						     triangle[1],
						     triangle[2],
						     segment.source() ) );
	CGAL_exactness_precondition( CGAL::coplanar( triangle[0],
						     triangle[1],
						     triangle[2],
						     segment.target() ) );

	// Check the segment against each triangle edge.

	return 
	    coplanar_do_intersect( segment, Segment_3<R>(triangle[0],triangle[1]) )
	 || coplanar_do_intersect( segment, Segment_3<R>(triangle[1],triangle[2]) )
	 || coplanar_do_intersect( segment, Segment_3<R>(triangle[2],triangle[0]) )
	 || triangle.has_on( segment.source() );
    }


    /*! \ingroup grp_coplanar_intersection
     *
     * \pre T is not degenerate
     * \pre segment is coplanar with T
     */
    template <class R>
    CGAL::Object
    coplanar_intersection( const Segment_3<R>& segment, 
			   const Triangle_3<R>& T )
    {
	CGAL_exactness_precondition( !T.is_degenerate() );
	CGAL_exactness_precondition( CGAL::coplanar( T[0], T[1], T[2],
						     segment.source() ) );
	CGAL_exactness_precondition( CGAL::coplanar( T[0], T[1], T[2],
						     segment.target() ) );
	// Clip the segment with a halfplane through
	// each triangle edge.
	//using CEP::intersection::Halfplane_3_Segment_3::coplanar_intersection;

	CGAL::Object obj = CEP::intersection::Halfplane_3_Segment_3::coplanar_intersection( T[0], T[1], T[2], segment );
	if ( obj.is_empty() )    return obj;
	Segment_3<R> seg;
	bool intersection_is_segment = assign( seg, obj );
	CGAL_assertion( intersection_is_segment );

	obj = CEP::intersection::Halfplane_3_Segment_3::coplanar_intersection( T[1], T[2], T[0], seg );
	if ( obj.is_empty() )    return obj;
	intersection_is_segment = assign( seg, obj );
	CGAL_assertion( intersection_is_segment );

	return CEP::intersection::Halfplane_3_Segment_3::coplanar_intersection( T[2], T[0], T[1], seg );
    }


    //! True if the segment and triangle intersect.
    template <class R>
    bool
    do_intersect( const Segment_3<R>& segment, 
		  const Triangle_3<R>& triangle )
    {
	// If the triangle is degenerate then we treat it as a
	// segment/segment intersection.  In a degenerate triangle any
	// edge will be contained in the union of the other two edges,
	// so it suffices to check only two edges of the triangle.
	if ( triangle.is_degenerate() )
	{
	    return 
		do_intersect( segment, Segment_3<R>(triangle[0],triangle[1]) )
	     || do_intersect( segment, Segment_3<R>(triangle[1],triangle[2]) );
	}

	CGAL::Orientation or_source = orientation( triangle[0],
						   triangle[1],
						   triangle[2],
						   segment.source() );

	CGAL::Orientation or_target = orientation( triangle[0],
						   triangle[1],
						   triangle[2],
						   segment.target() );

	// For the case that the segment endpoints lie on opposite
	// sides of the triangle's supporting plane, we need to know
	// which is on the positive side and which on the negative side.
	Point_3<R> pos;
	Point_3<R> neg;

	switch( or_source ) {
	case CGAL::NEGATIVE:
	    switch( or_target ) {
	    case CGAL::NEGATIVE:
		return false;
	    case CGAL::COPLANAR:
		return do_intersect( triangle, segment.target() );
	    case CGAL::POSITIVE:
		pos = segment.target();
		neg = segment.source();
		break;
	    }
	    break;
	case CGAL::COPLANAR:
	    switch( or_target ) {
	    case CGAL::NEGATIVE:
		return do_intersect( triangle, segment.source() );
	    case CGAL::COPLANAR:
		return coplanar_do_intersect( segment, triangle );
	    case CGAL::POSITIVE:
		return do_intersect( triangle, segment.source() );
	    }
	case CGAL::POSITIVE:
	    switch( or_target ) {
	    case CGAL::NEGATIVE:
		pos = segment.source();
		neg = segment.target();
		break;
	    case CGAL::COPLANAR:
		return do_intersect( triangle, segment.target() );
	    case CGAL::POSITIVE:
		return false;
	    }
	    break;
	}

	// We are left with the case that point neg is strictly on
	// the negative side of the (plane supporting the) triangle,
	// while point pos is strictly on the positive side.
	// Consider the three-sided cone with apex at neg, and
	// passing through the three edges of the triangle.  The
	// intersection of the segment (neg,pos) with the triangle's
	// supporting plane is on the triangle if, and only if, point
	// pos is inside the cone (including the bounding planes).

	return 
	    orientation( neg, triangle[0], triangle[1], pos ) != CGAL::NEGATIVE
	 && orientation( neg, triangle[1], triangle[2], pos ) != CGAL::NEGATIVE
	 && orientation( neg, triangle[2], triangle[0], pos ) != CGAL::NEGATIVE;
    }	


    //! Returns the point or segment of intersection.
    /*! \pre T is not degenerate
     */
    template <class R>
    CGAL::Object
    intersection( const Segment_3<R>& segment, 
		  const Triangle_3<R>& T )
    {
	CGAL_exactness_precondition( !T.is_degenerate() );

	Plane_3<R> plane = T.supporting_plane();
	CGAL::Object obj = CGAL::intersection( plane, segment );

	if ( obj.is_empty() )    return obj;

	Segment_3<R> s;
	if ( assign(s, obj) ) {
	    CGAL_assertion( s == segment );
	    return coplanar_intersection( segment, T );
	}

	Point_3<R> p;
	bool intersection_is_point = assign( p, obj );
	CGAL_assertion( intersection_is_point );
	if ( T.has_on(p) )
	    return CGAL::make_object(p);
	return CGAL::Object();
    }

}
}

