// Copyright 2001 Steven M. Robbins

#include <CGAL/intersection_3.h>


namespace CEP {  namespace intersection {


    using CGAL::Point_3;
    using CGAL::Segment_3;
    using CGAL::Line_3;
    using CGAL::Plane_3;
    using CGAL::Object;

    using CGAL::assign;
    using CGAL::make_object;


    namespace Plane_3_Triangle_3 {

	// Intersection is segment (u,v) *in* the plane.
	template <class R>
	inline Object
	_seg_in_plane( const Point_3<R>& u,
		       const Point_3<R>& v )
	{
	    Segment_3<R> seg(u,v);
	    return make_object(seg);
	}
    

	// Let x = intersection(plane,line(u,v)).
	// Return segment(p,x).
	template <class R>
	inline Object
	_point_and_seg( const Plane_3<R>& plane, 
			const Point_3<R>& p,
			const Point_3<R>& u,
			const Point_3<R>& v )
	{
	    Line_3<R> uv(u,v);
	    Object i = CGAL::intersection(plane,uv);
	    Point_3<R> x;
	    bool intersection_is_point = assign(x,i);
	    CGAL_assertion( intersection_is_point );
		
	    Segment_3<R> seg(p,x);
	    return make_object(seg);
	}


	// Let x = intersection(plane,line(p,u))
	// Let y = intersection(plane,line(p,v))
	// Return segment(x,y)
	template <class R>
	inline Object
	_2seg_through_plane( const Plane_3<R>& plane, 
			     const Point_3<R>& p,
			     const Point_3<R>& u,
			     const Point_3<R>& v )
	{
	    Line_3<R> pu(p,u);
	    Object i = CGAL::intersection(plane,pu);
	    Point_3<R> x;
	    bool intersection_is_point = assign(x,i);
	    CGAL_assertion( intersection_is_point );
		
	    Line_3<R> pv(p,v);
	    i = CGAL::intersection(plane,pv);
	    Point_3<R> y;
	    intersection_is_point = assign(y,i);
	    CGAL_assertion( intersection_is_point );
		
	    Segment_3<R> seg(x,y);
	    return make_object(seg);
	}


	// Used to be \ingroup grp_intersection
	/*! 
	 * Compute the intersection of the plane with the triangle
	 * given by points (p,q,r).  The result can be: empty, a
	 * point, a segment, or a triangle.
	 *
	 * \pre The triangle is not degenerate.
	 * \return Intersection of plane with triangle (p,q,r).
	 */
	template <class R>
	CGAL::Object
	intersection(const Plane_3<R>& plane, 
		     const Point_3<R>& p,
		     const Point_3<R>& q,
		     const Point_3<R>& r )
	{
	    CGAL_exactness_precondition( !collinear(p,q,r) );

	    CGAL::Oriented_side os0 = plane.oriented_side(p);
	    CGAL::Oriented_side os1 = plane.oriented_side(q);
	    CGAL::Oriented_side os2 = plane.oriented_side(r);

	    switch( os0 ) {
	    case CGAL::ON_NEGATIVE_SIDE:
		switch( os1 ) {
		case CGAL::ON_NEGATIVE_SIDE:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (-,-,-)
			return Object();
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (-,-,0)
			return make_object(r);
		    case CGAL::ON_POSITIVE_SIDE:                // (-,-,+)
			return _2seg_through_plane( plane, r,p,q );
		    }
		case CGAL::ON_ORIENTED_BOUNDARY:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (-,0,-)
			return make_object(q);
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (-,0,0)
			return _seg_in_plane( q, r );
		    case CGAL::ON_POSITIVE_SIDE:                // (-,0,+)
			return _point_and_seg( plane, q, p, r );
		    }
		case CGAL::ON_POSITIVE_SIDE:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (-,+,-)
			return _2seg_through_plane( plane, q,p,r );
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (-,+,0)
			return _point_and_seg( plane, r, p, q );
		    case CGAL::ON_POSITIVE_SIDE:                // (-,+,+)
			return _2seg_through_plane( plane, p,q,r );
		    }
		}
	    case CGAL::ON_ORIENTED_BOUNDARY:
		switch( os1 ) {
		case CGAL::ON_NEGATIVE_SIDE:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (0,-,-)
			return make_object(p);
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (0,-,0)
			return _seg_in_plane( p, r );
		    case CGAL::ON_POSITIVE_SIDE:                // (0,-,+)
			return _point_and_seg( plane, p, q, r );
		    }
		case CGAL::ON_ORIENTED_BOUNDARY:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (0,0,-)
			return _seg_in_plane( p, q );
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (0,0,0)
			{
			    Triangle_3<R> T(p,q,r);
			    return make_object(T);
			}
		    case CGAL::ON_POSITIVE_SIDE:                // (0,0,+)
			return _seg_in_plane( p, q );
		    }
		case CGAL::ON_POSITIVE_SIDE:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (0,+,-)
			return _point_and_seg( plane, p, q, r );
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (0,+,0)
			return _seg_in_plane( p, r );
		    case CGAL::ON_POSITIVE_SIDE:                // (0,+,+)
			return make_object(p);
		    }
		}
	    case CGAL::ON_POSITIVE_SIDE:
		switch( os1 ) {
		case CGAL::ON_NEGATIVE_SIDE:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (+,-,-)
			return _2seg_through_plane( plane, p,q,r );
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (+,-,0)
			return _point_and_seg( plane, r, p, q );
		    case CGAL::ON_POSITIVE_SIDE:                // (+,-,+)
			return _2seg_through_plane( plane, q,p,r );
		    }
		case CGAL::ON_ORIENTED_BOUNDARY:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (+,0,-)
			return _point_and_seg( plane, q, p, r );
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (+,0,0)
			return _seg_in_plane( q, r );
		    case CGAL::ON_POSITIVE_SIDE:                // (+,0,+)
			return make_object(q);
		    }
		case CGAL::ON_POSITIVE_SIDE:
		    switch( os2 ) {
		    case CGAL::ON_NEGATIVE_SIDE:                // (+,+,-)
			return _2seg_through_plane( plane, r,p,q );
		    case CGAL::ON_ORIENTED_BOUNDARY:            // (+,+,0)
			return make_object(r);
		    case CGAL::ON_POSITIVE_SIDE:                // (+,+,+)
			return Object();
		    }
		}
	    }

	    CGAL_assertion_msg(0,"cannot reach this point");
	    return Object();
	}


	//! Decide if a triangle intersects a plane.
	/*!
	 * The input given is the Oriented_side value for each of the
	 * three triangle vertices, relative to the plane.
	 */
	static 
	inline bool
	do_intersect_helper( CGAL::Oriented_side os0, 
			     CGAL::Oriented_side os1, 
			     CGAL::Oriented_side os2 )
	{
	    //return ! ( os0 == os1 && os0 == os2 
	    //           && os0 != CGAL::ON_ORIENTED_BOUNDARY );
	    return os0 != os1 || os0 != os2 
                   || os0 == CGAL::ON_ORIENTED_BOUNDARY;
	}


	// used to be \ingroup grp_do_intersect
	/*!
	 * \return True if the plane intersects triangle (p,q,r).
	 */
	template<class R>
	bool
	do_intersect( const Plane_3<R>& plane, 
		      const Point_3<R>& p,
		      const Point_3<R>& q,
		      const Point_3<R>& r )
	{
	    return do_intersect_helper( plane.oriented_side(p),
					plane.oriented_side(q),
					plane.oriented_side(r) );
	}


    }
}
}
