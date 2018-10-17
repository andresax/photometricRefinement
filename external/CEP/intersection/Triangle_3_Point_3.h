#ifndef CEP_INTERSECTION_TRIANGLE_3_POINT_3_H
#define CEP_INTERSECTION_TRIANGLE_3_POINT_3_H


#include <CGAL/Triangle_3.h>
#include <CGAL/Point_3.h>


namespace CEP {  namespace intersection {

    //! True if point lies on triangle.
    /*! \ingroup grp_coplanar_do_intersect
     *
     * \pre The triangle is not degenerate.
     * \pre Point lies on supporting plane of triangle.
     */
    template <class R>
    inline bool
    coplanar_do_intersect( const CGAL::Triangle_3<R>& triangle, 
			   const CGAL::Point_3<R>& point )
    {
	CGAL_exactness_precondition( !triangle.is_degenerate() );
	CGAL_exactness_precondition
	    ( CGAL::coplanar( triangle[0], triangle[1], triangle[2], point ) );

	return 
	    CGAL::coplanar_orientation( triangle[0],triangle[1],triangle[2], 
					point ) != CGAL::NEGATIVE
	 && CGAL::coplanar_orientation( triangle[1],triangle[2],triangle[0], 
					point ) != CGAL::NEGATIVE
	 && CGAL::coplanar_orientation( triangle[2],triangle[0],triangle[1], 
					point ) != CGAL::NEGATIVE;
    }


    //! True if point lies on triangle.
    /*! \ingroup grp_do_intersect
     *
     * \pre The triangle is not degenerate.
     */
    template <class R>
    inline bool
    do_intersect( const CGAL::Triangle_3<R>& triangle, 
		  const CGAL::Point_3<R>& point )
    {
	CGAL_exactness_precondition( !triangle.is_degenerate() );
	return CGAL::coplanar( triangle[0], triangle[1], triangle[2], point )
	    && coplanar_do_intersect( triangle, point );
    }

}
}


#endif

