#ifndef CEP_INTERSECTION_ISO_CUBOID_3_ISO_CUBOID_3_H
#define CEP_INTERSECTION_ISO_CUBOID_3_ISO_CUBOID_3_H

#include <CGAL/Iso_cuboid_3.h>

namespace CEP {
namespace intersection {


    //! Specialized Iso_cuboid/Iso_cuboid intersection functions.
    namespace Iso_cuboid_3_Iso_cuboid_3 {

	//! True if the cuboids intersect on the d'th axis.
	template <class R>
	inline
	bool do_intersect( const CGAL::Iso_cuboid_3<R>& c1, 
			   const CGAL::Iso_cuboid_3<R>& c2, int d )
	{
	    switch(d) {
	    case 0: return c1.xmin() <= c2.xmax()
			&& c2.xmin() <= c1.xmax();
	    case 1: return c1.ymin() <= c2.ymax()
			&& c2.ymin() <= c1.ymax();
	    case 2: return c1.zmin() <= c2.zmax()
			&& c2.zmin() <= c1.zmax();
	    }
	    CGAL_assertion_msg(0,"should not reach here");
	    return false;
	}
    }


//! True if the cuboids intersect.
/*! \ingroup grp_do_intersect
 */
template<class R>
inline bool do_intersect( const CGAL::Iso_cuboid_3<R> & c1, 
			  const CGAL::Iso_cuboid_3<R> & c2 )
{
    return Iso_cuboid_3_Iso_cuboid_3::do_intersect( c1, c2, 0 )
        && Iso_cuboid_3_Iso_cuboid_3::do_intersect( c1, c2, 1 )
        && Iso_cuboid_3_Iso_cuboid_3::do_intersect( c1, c2, 2 );
}


}
}

#endif

