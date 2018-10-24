#ifndef CEP_INTERSECTION_OBJECT_H    // -*- C++ -*-
#define CEP_INTERSECTION_OBJECT_H

#include <CGAL/Point_3.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_3.h>

#include <CGAL/predicates_on_points_3.h>
#include <CGAL/Object.h>



namespace CEP {  namespace intersection {

    using CGAL::Object;
    using CGAL::Point_3;
    using CGAL::Segment_3;
    using CGAL::Triangle_3;

    using CGAL::assign;
    using CGAL::make_object;


    template <class R>
    Object make_object_nondegenerate( const Segment_3<R>& s )
    {
	if ( s.is_degenerate() )
	    return make_object( s.source() );
	return make_object(s);
    }


    template <class R>
    Object make_object_nondegenerate( const Triangle_3<R>& T )
    {
	if ( ! T.is_degenerate() )
	    return make_object(T);

	if ( CGAL::collinear_are_ordered_along_line( T[0], T[1], T[2] ) )
	    return make_object_nondegenerate( Segment_3<R>(T[0],T[2]) );
	if ( CGAL::collinear_are_ordered_along_line( T[1], T[2], T[0] ) )
	    return make_object_nondegenerate( Segment_3<R>(T[1],T[0]) );
	CGAL_assertion( CGAL::collinear_are_ordered_along_line( T[2], T[0], T[1] ));
	return make_object_nondegenerate( Segment_3<R>(T[2],T[1]) );
    }


    template <class R>
    Object make_object_nondegenerate( const Object& obj )
    {
	Triangle_3<R> T;
	Segment_3<R> s;
	Point_3<R> p;
       
	if ( assign(T,obj) )
	    return make_object_nondegenerate( T );
	else if ( assign(s,obj) )
	    return make_object_nondegenerate( s );

	bool intersection_is_point = assign(p,obj);
	CGAL_assertion( intersection_is_point );
	return obj;
    }

}
}

#endif
