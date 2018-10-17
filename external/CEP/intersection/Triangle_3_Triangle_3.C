// Copyright 2001 Steven M. Robbins

#include <vector>
#include <iterator>

#include <CGAL/Triangle_2.h>
#include <CGAL/Triangle_2_Triangle_2_intersection.h>
#include <CGAL/circulator.h>
#include <CGAL/predicates_on_points_3.h>

#include <CEP/intersection/Plane_3_Triangle_3.h>
#include <CEP/intersection/Segment_3_Segment_3.h>
#include <CEP/intersection/Segment_3_Point_3.h>
#include <CEP/intersection/Line_3_Line_3.h>


namespace CEP {    namespace intersection {


    using CGAL::Plane_3;
    using CGAL::Triangle_3;
    using CGAL::Triangle_2;

    using CGAL::assign;
    using CGAL::make_object;
    using CGAL::collinear_are_ordered_along_line;


    /*! \brief Specialized triangle/triangle intersection functions.
     * 
     */
    namespace Triangle_3_Triangle_3 {

	//! Project a 3D triangle to 2D.
	/*! \internal
	 * Returns the parallel projection of the input triangle onto a
	 * plane perpendicular to the projection direction.  The projection
	 * direction must be parallel to one of the coordinate axes.
	 *
	 * \param projection_dir 0=x-axis, 1=y-axis, 2=z-axis
	 */
	template <class R>
	Triangle_2<R>
	_project_triangle( const Triangle_3<R>& T, int projection_dir )
	{
	    int i1 = (projection_dir + 1) %3;
	    int i2 = (projection_dir + 2) %3;
	    
	    CGAL::Point_2<R> p( T[0].homogeneous(i1), 
				T[0].homogeneous(i2), 
				T[0].hw() );
	    CGAL::Point_2<R> q( T[1].homogeneous(i1), 
				T[1].homogeneous(i2), 
				T[1].hw() );
	    CGAL::Point_2<R> r( T[2].homogeneous(i1), 
				T[2].homogeneous(i2),
				T[2].hw() );
	    
	    return CGAL::Triangle_2<R>(p,q,r);
	}


	// \ingroup grp_coplanar_do_intersect
	/*! \brief True if triangles T1 and T2 intersect.
	 * \internal
	 *
	 * \pre Both T1 and T2 must lie on plane P.
	 */
	template <class R>
	bool
	coplanar_do_intersect( const Plane_3<R>& P,
			       const Triangle_3<R>& T1, 
			       const Triangle_3<R>& T2 )
	{
	    CGAL_exactness_precondition( P.has_on(T1[0]) );
	    CGAL_exactness_precondition( P.has_on(T1[1]) );
	    CGAL_exactness_precondition( P.has_on(T1[2]) );
	    
	    CGAL_exactness_precondition( P.has_on(T2[0]) );
	    CGAL_exactness_precondition( P.has_on(T2[1]) );
	    CGAL_exactness_precondition( P.has_on(T2[2]) );
	    
	    // Project triangles to some axis-aligned plane, then use
	    // 2D routines.  Any projection direction is fine, as long
	    // as the triangles do not project to a line segment.
	    // Equivalently, any direction for which the triangles'
	    // plane normal does not vanish is acceptable.  Choose
	    // projection direction that maximizes the plane normal
	    // projection.
	    CGAL::Direction_3<R> normal = P.orthogonal_direction();
	    int proj_dir;

	    double dx = fabs(CGAL::to_double(normal.dx()));
	    double dy = fabs(CGAL::to_double(normal.dy()));
	    double dz = fabs(CGAL::to_double(normal.dz()));

	    if ( dx > dy ) {
		if ( dx > dz )
		    proj_dir = 0;
		else
		    proj_dir = 2;
	    } else {
		if ( dy > dz )
		    proj_dir = 1;
		else
		    proj_dir = 2;
	    }
	    
	    Triangle_2<R> T1p = _project_triangle(T1,proj_dir);
	    Triangle_2<R> T2p = _project_triangle(T2,proj_dir);
	    
	    return CGAL::do_intersect( T1p,T2p );
	}



	// \ingroup grp_do_intersect
	/*! \internal
	 *
	 * \pre triangles are not degenerate
	 * \pre triangle T1 lies in plane P1
	 * \pre triangle T2 lies in plane P2
	 */
	template <class R>
	bool
	do_intersect( const Triangle_3<R>& T1, const Plane_3<R>& P1,
		      const Triangle_3<R>& T2, const Plane_3<R>& P2 )
	{
	    CGAL_exactness_precondition( !T1.is_degenerate() );
	    CGAL_exactness_precondition( !T2.is_degenerate() );
	    
	    CGAL_exactness_precondition( P1.has_on(T1[0]) );
	    CGAL_exactness_precondition( P1.has_on(T1[1]) );
	    CGAL_exactness_precondition( P1.has_on(T1[2]) );
	    
	    CGAL_exactness_precondition( P2.has_on(T2[0]) );
	    CGAL_exactness_precondition( P2.has_on(T2[1]) );
	    CGAL_exactness_precondition( P2.has_on(T2[2]) );


	    CGAL::Object P1_int_T2 = CEP::intersection::intersection( P1, T2 );
	    if ( P1_int_T2.is_empty() )
		return false;

	    CGAL::Object P2_int_T1 = CEP::intersection::intersection( P2, T1 );
	    if ( P2_int_T1.is_empty() )
		return false;


	    // Either both intersections are a triangle, or neither.
	    Triangle_3<R> t1, t2;
	    if ( assign(t1,P2_int_T1) ) {
		bool intersection_is_triangle = assign( t2, P1_int_T2 );
		CGAL_assertion( intersection_is_triangle );
		return coplanar_do_intersect(P1,T1,T2);
	    }
	    CGAL_assertion( !assign(t2,P1_int_T2) );


	    // Only point and segment cases left.
	    CGAL::Point_3<R>    p1, p2;  
	    CGAL::Segment_3<R>  s1, s2;  
	    
	    if ( assign(p1,P2_int_T1) ) {
		if ( assign(p2,P1_int_T2) ) {
		    return p1 == p2;
		} 
		if ( assign(s2,P1_int_T2) ) {
		    return collinear_are_ordered_along_line( s2.source(), 
							     p1, 
							     s2.target() );
		}
	    } else if ( assign(s1,P2_int_T1) ) {
		if ( assign(p2,P1_int_T2) ) {
		    return collinear_are_ordered_along_line( s1.source(), 
							     p2, 
							     s1.target() );
		} 
		if ( assign(s2,P1_int_T2) ) {
		    return collinear_do_intersect( s1, s2 );
		}
	    }

	    // Cannot get here.
	    CGAL_assertion_msg(0,"should not reach here");
	    return true;
	}


	/*
	 * Input is a polygon P given as a sequence [points_begin,points_end)
	 * of Point_3<R>, and a half-plane H specified using three points s,
	 * t, u.  H is half the plane incident to (s,t,u), with s and t on the
	 * boundary, and u on the interior of H.  H is relatively closed;
	 * i.e. it includes the line through (s,t).
	 *
	 * All points must lie in a common plane (not checked).
	 *
	 * The function inserts points of the intersection of P and H into the
	 * container associated with output.
	 * 
	 */
	template <class R, class InPointIterator, class OutPointIterator>
	void _slice_in_plane( InPointIterator points_begin, 
			      InPointIterator points_end,
			      const CGAL::Point_3<R>& s,
			      const CGAL::Point_3<R>& t,
			      const CGAL::Point_3<R>& u,
			      OutPointIterator output )
	{
	    if ( points_begin == points_end )
		return;
	    
	    typedef CGAL::Circulator_from_iterator<InPointIterator> Circulator;
	    typedef typename 
		std::iterator_traits<InPointIterator>::value_type Point_;
	    
	    Circulator p_begin( points_begin, points_end );
	    Circulator p = p_begin;
	    
	    do {
		CGAL::Orientation side 
		    = CGAL::coplanar_orientation( s,t,u, *p );
		
		if ( side != CGAL::NEGATIVE )
		    *output++ = *p;
		
		if ( side != CGAL::COLLINEAR ) {
		    Circulator q(p);  
		    ++q;
		    CGAL::Orientation q_side 
			= CGAL::coplanar_orientation( s,t,u, *q );
		    
		    if ( q_side != side && q_side != CGAL::COLLINEAR ) {
			CGAL::Object o_int 
			    = CEP::intersection::Line_3_Line_3::coplanar_intersection( s,t, *p,*q );
			Point_ p_int;
			bool intersection_is_point = assign(p_int,o_int);
			CGAL_assertion( intersection_is_point );
			*output++ = p_int;
		    }
		}
	    } while( ++p != p_begin );
	}
	

	/*! \ingroup grp_coplanar_intersection
	 *
	 * The intersection result can be either empty, a point, a
	 * line segment, a triangle, or a list of four or more points
	 * representing a polygon on the plane.  In the latter case,
	 * the return value is of type std::vector<Point_3>.
	 * 
	 * \pre Triangle T1 lies in plane P.
	 * \pre Triangle T2 lies in plane P.
	 */
	template <class R>
	CGAL::Object
	coplanar_intersection( const Plane_3<R>& P, 
			       const Triangle_3<R>& T1, 
			       const Triangle_3<R>& T2 )
	{
	    typedef CGAL::Point_3<R> Point;
	    
	    CGAL_exactness_precondition( P.has_on(T1[0]) );
	    CGAL_exactness_precondition( P.has_on(T1[1]) );
	    CGAL_exactness_precondition( P.has_on(T1[2]) );
	    
	    CGAL_exactness_precondition( P.has_on(T2[0]) );
	    CGAL_exactness_precondition( P.has_on(T2[1]) );
	    CGAL_exactness_precondition( P.has_on(T2[2]) );

	    std::vector<Point> poly1(6);
	    std::vector<Point> poly2(6);

	    // Let poly1 = T2
	    // Let poly2 = poly1 intersect halfplane bounded by T1[0],T1[1]
	    poly1.clear();
	    poly1.push_back( T2[0] );
	    poly1.push_back( T2[1] );
	    poly1.push_back( T2[2] );

	    poly2.clear();
	    _slice_in_plane( poly1.begin(), poly1.end(),
			     T1[0], T1[1], T1[2],
			     std::back_inserter(poly2) );
	    
	    // Let poly1 = poly2 intersect halfplane bounded by T1[1],T1[2]
	    poly1.clear();
	    _slice_in_plane( poly2.begin(), poly2.end(),
			     T1[1], T1[2], T1[0],
			     std::back_inserter(poly1) );

	    // Let poly2 = poly1 intersect halfplane bounded by T1[2],T1[0]
	    poly2.clear();
	    _slice_in_plane( poly1.begin(), poly1.end(),
			     T1[2], T1[0], T1[1],
			     std::back_inserter(poly2) );

	    switch( poly2.size() ) {
	    case 0 : return CGAL::Object();
	    case 1 : return make_object( poly2[0] );
	    case 2 : return make_object( Segment_3<R>(poly2[0],poly2[1]) );
	    case 3 : return make_object( Triangle_3<R>(poly2[0],
						       poly2[1],
						       poly2[2]) );
	    }
	    
	    CGAL_assertion( poly2.size() <= 6 );
	    std::vector<Point>* ret = new std::vector<Point>( poly2 );
	    return make_object( *ret );
	}



	// \ingroup grp_intersection
	/*!
	 *
	 * \pre Triangle T1 lies in plane P1.
	 * \pre Triangle T2 lies in plane P2.
	 */
	template <class R>
	CGAL::Object
	intersection( const Triangle_3<R>& T1, const Plane_3<R>& P1,
		      const Triangle_3<R>& T2, const Plane_3<R>& P2 )
	{
	    CGAL_exactness_precondition( P1.has_on(T1[0]) );
	    CGAL_exactness_precondition( P1.has_on(T1[1]) );
	    CGAL_exactness_precondition( P1.has_on(T1[2]) );
	    
	    CGAL_exactness_precondition( P2.has_on(T2[0]) );
	    CGAL_exactness_precondition( P2.has_on(T2[1]) );
	    CGAL_exactness_precondition( P2.has_on(T2[2]) );

	    CGAL::Object P1_int_T2 = CEP::intersection::intersection( P1, T2 );
	    if ( P1_int_T2.is_empty() )
		return CGAL::Object();

	    CGAL::Object P2_int_T1 = CEP::intersection::intersection( P2, T1 );
	    if ( P2_int_T1.is_empty() )
		return CGAL::Object();

	    // Either both intersections are a triangle, or neither.
	    Triangle_3<R> t1, t2;
	    if ( assign(t1,P2_int_T1) ) {
		bool intersection_is_triangle = assign( t2, P1_int_T2);
		CGAL_assertion( intersection_is_triangle );
		return coplanar_intersection(P1,T1,T2);
	    }
	    CGAL_assertion( !assign(t2,P1_int_T2) );

	    // Only point and segment cases left.
	    CGAL::Point_3<R>    p1, p2;  
	    CGAL::Segment_3<R>  s1, s2;  
	    
	    if ( assign(p1,P2_int_T1) ) {
		if ( assign(p2,P1_int_T2) ) {
		    return p1 == p2 ? make_object(p1) : CGAL::Object();
		} 
		bool intersection_is_segment = assign(s2,P1_int_T2);
		CGAL_assertion( intersection_is_segment );
		return CEP::intersection::intersection( s2,p1 );
	    }
	    
	    bool intersection_is_segment = assign(s1,P2_int_T1);
	    CGAL_assertion( intersection_is_segment );
	    if ( assign(p2,P1_int_T2) )
		return CEP::intersection::intersection( s1,p2 );

	    intersection_is_segment = assign(s2,P1_int_T2);
	    CGAL_assertion( intersection_is_segment );
	    return CEP::intersection::intersection(s1,s2);
	}
	
    }
    
}
}
