#ifndef CEP_INTERSECTION_TRIANGLE_3_PLANE_3_H
#define CEP_INTERSECTION_TRIANGLE_3_PLANE_3_H


#include <CEP/intersection/Plane_3_Triangle_3.h>


namespace CEP {  
namespace intersection {

    //! Compute intersection of triangle and plane.
    /*! \ingroup grp_intersection
     *
     * \pre The triangle is not degenerate.
     * \return Intersection of plane with triangle.
     */
    template <class R>
    inline 
    CGAL::Object
    intersection( const CGAL::Triangle_3<R>& triangle, 
		  const CGAL::Plane_3<R>& plane )
    {
	return intersection( plane, triangle );
    }


    //! True if the plane and triangle intersect.
    /*! \ingroup grp_do_intersect
     *
     * \return True if the plane intersects triangle.
     */
    template <class R>
    inline 
    bool
    do_intersect( const CGAL::Triangle_3<R>& triangle, 
		  const CGAL::Plane_3<R>& plane )
    {
	return do_intersect( plane, triangle );
    }


}
}


#endif

