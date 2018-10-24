#include <CGAL/Segment_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/predicates_on_points_3.h>
#include <CGAL/Object.h>

#include <CEP/intersection/Segment_3_Point_3.h>
#include <CEP/intersection/Line_3_Line_3.h>


namespace CEP {  
namespace intersection {

    using CGAL::Segment_3;

    using CGAL::make_object;
    using CGAL::collinear;
    using CGAL::collinear_are_ordered_along_line;
    using CGAL::coplanar;
    using CGAL::coplanar_orientation;


    /*! \ingroup grp_collinear_do_intersect
     *
     * \pre Segments are collinear
     */
    template <class R>
    inline
    bool
    collinear_do_intersect( const Segment_3<R>& s1, 
			    const Segment_3<R>& s2 )
    {
	CGAL_exactness_precondition( collinear(s1,s2.source()) );
	CGAL_exactness_precondition( collinear(s1,s2.target()) );

	// segment 1     s-------------t
	// segment 2  *----*
	// cases      *--------------------*
	//                  *------*
	//                     *-----------*

	return collinear_are_ordered_along_line
	       ( s1.source(), s2.source(), s1.target() )
	    || collinear_are_ordered_along_line
	       ( s1.source(), s2.target(), s1.target() )
	    || collinear_are_ordered_along_line
	       ( s2.source(), s1.source(), s2.target() );
    }


    /*! \ingroup grp_coplanar_do_intersect
     *
     * \pre segments are coplanar
     */
    template <class R>
    bool
    coplanar_do_intersect( const Segment_3<R>& s1,
			   const Segment_3<R>& s2 )
    {
	if ( s1.is_degenerate() )
	    return s2.has_on(s1.source());
	if ( s2.is_degenerate() )
	    return s1.has_on(s2.source());

	CGAL_exactness_precondition ( coplanar(s1.source(),s1.target(),
					       s2.source(),s2.target()) );

	// If two non-degenerate segments intersect, the intersection
	// either contains a segment endpoint (four possibilities to
	// check) or it is a point on the relative interior of both
	// segments.  We perform the first check now.

	if ( s1.has_on(s2.source()) ||
	     s1.has_on(s2.target()) ||
	     s2.has_on(s1.source()) ||
	     s2.has_on(s1.target()) )
	    return true;

	// At this point, the segments are guaranteed to be
	// non-degenerate, coplanar, and no endpoint lies on the other
	// segment.  The coplanar_orientation function requires that the
	// first three arguments (that define the plane and its orientation)
	// are NOT collinear.  In the case that the three points are
	// collinear, we can rule out an intersection, because such
	// an intersection would contain at least one segment endpoint,
	// and so would have been caught by the above tests.

	if ( collinear( s1.source(), s1.target(), s2.source() ) ||
	     collinear( s2.source(), s2.target(), s1.source() ) )
	    return false;

	return coplanar_orientation( s1.source(), s1.target(),
				     s2.source(), s2.target() ) 
	       == CGAL::NEGATIVE
	    && coplanar_orientation( s2.source(), s2.target(),
				     s1.source(), s1.target() ) 
	       == CGAL::NEGATIVE;
    }


    /*! \ingroup grp_do_intersect
     */
    template <class R>
    bool
    do_intersect( const Segment_3<R>& s1,
		  const Segment_3<R>& s2 )
    {
	if ( s1.is_degenerate() )
	    return s2.has_on(s1.source());
	if ( s2.is_degenerate() )
	    return s1.has_on(s2.source());

	// If two non-degenerate segments intersect, the intersection
	// either contains a segment endpoint (four possibilities to
	// check) or it is a point on the relative interior of both
	// segments.  The latter case is determined by checking (a) that
	// all four endpoints are coplanar, and (b) that the line through
	// a segment separates the two endpoints of the other segment (in
	// the common plane).

	if ( s1.has_on(s2.source()) ||
	     s1.has_on(s2.target()) ||
	     s2.has_on(s1.source()) ||
	     s2.has_on(s1.target()) )
	    return true;

	if ( !coplanar(s1.source(),s1.target(),
		       s2.source(),s2.target()) )
	    return false;

	// At this point, the segments are guaranteed to be
	// non-degenerate, coplanar, and no endpoint lies on the other
	// segment.  The coplanar_orientation function requires that the
	// first three arguments (that define the plane and its orientation)
	// are NOT collinear.  In the case that the three points are
	// collinear, we can rule out an intersection, because such
	// an intersection would contain at least two segment endpoints,
	// and so would have been caught by the above tests.

	if ( collinear( s1.source(), s1.target(), s2.source() ) ||
	     collinear( s2.source(), s2.target(), s1.source() ) )
	    return false;

	return coplanar_orientation( s1.source(), s1.target(),
				     s2.source(), s2.target() ) 
	       == CGAL::NEGATIVE
	    && coplanar_orientation( s2.source(), s2.target(),
				     s1.source(), s1.target() ) 
	       == CGAL::NEGATIVE;
    }


    namespace Segment_3_Segment_3 {
	
	/*! \brief Classify point with respect to segment. 
	 * \internal
	 * 
	 * \param S a line segment
	 * \param p a point
	 *
	 * \pre p is on supporting line of S.
	 *
	 * Orient S so that the source is on the left, and the target
	 * point on the right.  Classify p according to where it falls
	 * on the line:
	 *
	 * = 1 if strictly left of S.source
	 * = 2 if equal to S.source
	 * = 3 if strictly on the interior of S
	 * = 4 if equal to S.target
	 * = 5 if strictly right of S.target
	 */
	template <class R>
	int classify_point( const Segment_3<R>& S,
			    const Point_3<R>& p )
	{
	    CGAL_exactness_precondition( collinear(S,p) );

	    const Point_3<R>& s = S.source();
	    const Point_3<R>& t = S.target();

	    if ( p == s )  return 2;
	    if ( p == t )  return 4;
	    
	    if ( collinear_are_ordered_along_line( p, s, t ) )  return 1;
	    if ( collinear_are_ordered_along_line( s, p, t ) )  return 3;
	    CGAL_exactness_assertion( collinear_are_ordered_along_line( s,t,p ) );
	    return 5;
	}
    }


    /*! \ingroup grp_collinear_intersection
     *
     * \pre Segments are collinear
     */
    template <class R>
    CGAL::Object
    collinear_intersection( const Segment_3<R>& seg1,
			    const Segment_3<R>& seg2 )
    {
	const Point_3<R>& s1 = seg1.source();
	const Point_3<R>& t1 = seg1.target();
	const Point_3<R>& s2 = seg2.source();
	const Point_3<R>& t2 = seg2.target();

	CGAL_exactness_precondition( collinear(seg1,s2) );
	CGAL_exactness_precondition( collinear(seg1,t2) );

	int s1_class = Segment_3_Segment_3::classify_point( seg2, s1 );
	int t1_class = Segment_3_Segment_3::classify_point( seg2, t1 );

	switch( s1_class ) {
	case 1: 
	    switch( t1_class ) {
	    case 1: return Object();
	    case 2: return make_object( s2 );
	    case 3: return make_object( Segment_3<R>(s2,t1) );
	    case 4: // fall through
	    case 5: return make_object( seg2 );
	    }
	case 2: 
	    switch( t1_class ) {
	    case 1: // fall through
	    case 2: return make_object( s2 );
	    case 3: // fall through
	    case 4: return make_object( seg1 );
	    case 5: return make_object( seg2 );
	    }
	case 3: 
	    switch( t1_class ) {
	    case 1: // fall through
	    case 2: return make_object( Segment_3<R>(s2,s1) );
	    case 3: // fall through
	    case 4: return make_object( seg1 );
	    case 5: return make_object( Segment_3<R>(s1,t2) );
	    }
	case 4: 
	    switch( t1_class ) {
	    case 1: // fall through
	    case 2: return make_object( seg2 );
	    case 3: return make_object( seg1 );
	    case 4: // fall through
	    case 5: return make_object( t2 );
	    }
	case 5: 
	    switch( t1_class ) {
	    case 1: // fall through
	    case 2: return make_object( seg2 );
	    case 3: return make_object( Segment_3<R>(t1,t2) );
	    case 4: return make_object( t2 );
	    case 5: return Object();
	    }
	}
	CGAL_assertion_msg(false, "internal error");
	return CGAL::Object();
    }



    /*! \ingroup grp_intersection
     */
    template <class R>
    CGAL::Object
    intersection( const Segment_3<R>& s1,
		  const Segment_3<R>& s2 )
    {
	if ( s1.is_degenerate() ) 
	    return intersection( s2, s1.source() );
	if ( s2.is_degenerate() )
	    return intersection( s1, s2.source() );

	// Check all cases of collinear endpoints

	if ( collinear( s1, s2.source() )) {
	    if ( collinear( s1, s2.target() )) {
		return collinear_intersection( s1, s2 );
	    } else {
		return intersection( s1, s2.source() );
	    }
	} else if ( collinear( s1, s2.target() )) {
	    return intersection( s1, s2.target() );
	} else if ( collinear( s2, s1.source() )) {
	    return intersection( s2, s1.source() );
	} else if ( collinear( s2, s1.target() )) {
	    return intersection( s2, s1.target() );
	}

	// If no three points are collinear, the intersection can only
	// be a point in the relative interior of each segment, which
	// implies that the segments must be coplanar, and that the
	// line through a segment separates the endpoints of the
	// other, in the plane.

	if ( !coplanar(s1.source(),s1.target(),
		       s2.source(),s2.target()) )
	    return CGAL::Object();

	if (    coplanar_orientation( s1.source(), s1.target(),
				      s2.source(), s2.target() ) 
		!= CGAL::NEGATIVE
	     || coplanar_orientation( s2.source(), s2.target(),
				      s1.source(), s1.target() ) 
		!= CGAL::NEGATIVE )
	    return CGAL::Object();

	return 
	   CEP::intersection::Line_3_Line_3::coplanar_nonparallel_intersection
	    ( s1.source(), s1.target(), s2.source(), s2.target() );
    }


}
}
