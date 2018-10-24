#ifndef CEP_INTERSECTION_TRIANGLE_3_TRIANGLE_3_H  // -*- C++ -*-
#define CEP_INTERSECTION_TRIANGLE_3_TRIANGLE_3_H

#include <CGAL/Plane_3.h>
#include <CGAL/Triangle_3.h>
#include <CEP/intersection/Object.h>
#include <CEP/intersection/Segment_3_Segment_3.h>
#include <CEP/intersection/Segment_3_Triangle_3.h>


namespace CEP {    
namespace intersection {

    using CGAL::Plane_3;
    using CGAL::Segment_3;
    using CGAL::Triangle_3;

    using CGAL::assign;
    using CGAL::make_object;


    namespace Triangle_3_Triangle_3 {
 
	//! True if triangles T1 and T2 intersect.
	template <class R>
	bool
	do_intersect( const Triangle_3<R>& T1, const Plane_3<R>& P1,
		      const Triangle_3<R>& T2, const Plane_3<R>& P2 );

	//! Compute intersection of triangles T1 and T2.
	template <class R>
	CGAL::Object	
	intersection( const Triangle_3<R>& T1, const Plane_3<R>& P1,
		      const Triangle_3<R>& T2, const Plane_3<R>& P2 );

	//! Compute intersection of triangles T1 and T2.
	template <class R>
	CGAL::Object
	coplanar_intersection( const Plane_3<R>& P, 
			       const Triangle_3<R>& T1, 
			       const Triangle_3<R>& T2 );
    }


/*! \brief True if triangles T1 and T2 intersect.
 * \ingroup grp_do_intersect
 */
template <class R>
inline 
bool
do_intersect( const Triangle_3<R>& T1, const Triangle_3<R>& T2 )
{
    // Check for intersection of an edge of T1 with triangle T2.
    // The two triangles intersect if, and only if, there are
    // two such intersecting edges.  Therefore we need only check 
    // five of the six edges.

#ifdef TRIANGLE_3_TRIANGLE_3_TRACE

    extern int TRIANGLE_3_TRIANGLE_3_TRACE[5];

    TRIANGLE_3_TRIANGLE_3_TRACE[0] = 
	do_intersect( Segment_3<R>(T1[0],T1[1]), T2 );
    TRIANGLE_3_TRIANGLE_3_TRACE[1] = 
	do_intersect( Segment_3<R>(T1[1],T1[2]), T2 );
    TRIANGLE_3_TRIANGLE_3_TRACE[2] = 
	do_intersect( Segment_3<R>(T1[2],T1[0]), T2 );
    TRIANGLE_3_TRIANGLE_3_TRACE[3] = 
	do_intersect( Segment_3<R>(T2[0],T2[1]), T1 );
    TRIANGLE_3_TRIANGLE_3_TRACE[4] = 
	do_intersect( Segment_3<R>(T2[1],T2[2]), T1 );
#endif

    return do_intersect( Segment_3<R>(T1[0],T1[1]), T2 )
	|| do_intersect( Segment_3<R>(T1[1],T1[2]), T2 )
	|| do_intersect( Segment_3<R>(T1[2],T1[0]), T2 )
	|| do_intersect( Segment_3<R>(T2[0],T2[1]), T1 )
	|| do_intersect( Segment_3<R>(T2[1],T2[2]), T1 );
}


/*! \brief Compute intersection of triangles T1 and T2.
 * \ingroup grp_intersection
 */
template <class R>
inline 
CGAL::Object
intersection( const Triangle_3<R>& T1, const Triangle_3<R>& T2 )
{
    if ( !T1.is_degenerate() && !T2.is_degenerate() ) {
	Plane_3<R> plane1(T1[0],T1[1],T1[2]);
	Plane_3<R> plane2(T2[0],T2[1],T2[2]);
	return Triangle_3_Triangle_3::intersection(T1,plane1, T2,plane2);
    }
    
    CGAL::Object O1 = make_object_nondegenerate(T1);
    CGAL::Object O2 = make_object_nondegenerate(T2);

    Triangle_3<R> t1,t2;
    Segment_3<R> s1,s2;
    Point_3<R> p1,p2;

    if ( assign(t1,O1) ) {
	if ( assign(s2,O2) )
	    return CEP::intersection::intersection(s2,t1);
	bool intersection_is_point = assign(p2,O2);
	CGAL_assertion( intersection_is_point );
	if ( t1.has_on(p2) )
	    return make_object(p2);
	return CGAL::Object();
    }

    if ( assign(t2,O2) ) {
	if ( assign(s1,O1) )
	    return  CEP::intersection::intersection(s1,t2);	    
	bool intersection_is_point = assign(p1,O1);
	CGAL_assertion( intersection_is_point );
	if ( t2.has_on(p1) )
	    return make_object(p1);
	return CGAL::Object();
    }

    if ( assign(s1,O1) ) {
	if ( assign(s2,O2) )
	    return CEP::intersection::intersection(s1,s2);
	bool intersection_is_point =  assign(p2,O2);
	CGAL_assertion( intersection_is_point );
	if ( s1.has_on(p2) )
	    return make_object(p2);
	return CGAL::Object();
    }

    bool intersection_is_point = assign(p1,O1);
    CGAL_assertion( intersection_is_point );

    if ( assign(s2,O2) ) {
	if ( s2.has_on(p1) )
	    return make_object(p1);
	return CGAL::Object();
    }

    intersection_is_point = assign(p2,O2);
    CGAL_assertion( intersection_is_point );

    if ( p1 == p2 )
	return make_object(p1);
    return CGAL::Object();
}


}
}

#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Triangle_3_Triangle_3.C>
#endif

#endif
