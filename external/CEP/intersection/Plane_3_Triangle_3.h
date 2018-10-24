#ifndef CEP_INTERSECTION_PLANE_3_TRIANGLE_3_H    // -*- C++ -*-
#define CEP_INTERSECTION_PLANE_3_TRIANGLE_3_H

#include <CGAL/Plane_3.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Object.h>


namespace CEP {  
namespace intersection {

    using CGAL::Point_3;
    using CGAL::Plane_3;
    using CGAL::Triangle_3;


    namespace Plane_3_Triangle_3 {
	
	//! True if the plane and triangle (p,q,r) intersect.
	template <class R>
	bool
	do_intersect( const Plane_3<R>& plane, 
		      const Point_3<R>& p,
		      const Point_3<R>& q,
		      const Point_3<R>& r );


	//! Compute intersection of plane and triangle (p,q,r).
	template <class R>
	CGAL::Object
	intersection( const Plane_3<R>& plane, 
		      const Point_3<R>& p,
		      const Point_3<R>& q,
		      const Point_3<R>& r );
    }


    //! Compute intersection of plane and triangle.
    /*! \ingroup grp_intersection
     *
     * \pre The triangle is not degenerate.
     * \return Intersection of plane with triangle.
     */
    template <class R>
    inline CGAL::Object
    intersection(const Plane_3<R>& plane, const Triangle_3<R>& triangle)
    {
	return Plane_3_Triangle_3::intersection( plane, 
						 triangle[0], 
						 triangle[1], 
						 triangle[2] );
    }


    //! True if the plane and triangle intersect.
    /*! \ingroup grp_do_intersect
     *
     * \return True if the plane intersects triangle.
     */
    template <class R>
    inline bool
    do_intersect( const Plane_3<R>& plane, const Triangle_3<R>& triangle )
    {
	return Plane_3_Triangle_3::do_intersect( plane, 
						 triangle[0],
						 triangle[1],
						 triangle[2] );
    }


}
}


#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Plane_3_Triangle_3.C>
#endif

#endif
