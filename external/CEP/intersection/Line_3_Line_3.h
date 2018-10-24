#ifndef CEP_INTERSECTION_LINE_3_LINE_3_H    // -*- C++ -*-
#define CEP_INTERSECTION_LINE_3_LINE_3_H

#include <CGAL/Object.h>
#include <CGAL/Point_3.h>


namespace CEP {  
namespace intersection {

    using CGAL::Point_3;

    namespace Line_3_Line_3 {
	
	//! Compute intersection of line through (p,q) and line through (r,s).
	template <class R>
	CGAL::Object
	coplanar_intersection( const Point_3<R>& p,
			       const Point_3<R>& q,
			       const Point_3<R>& r,
			       const Point_3<R>& s );


	//! Compute intersection of line through (p,q) and line through (r,s).
	template <class R>
	CGAL::Object
	coplanar_nonparallel_intersection( const Point_3<R>& p,
					   const Point_3<R>& q,
					   const Point_3<R>& r,
					   const Point_3<R>& s );
	
    }
}
}


#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Line_3_Line_3.C>
#endif

#endif
