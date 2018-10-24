#ifndef CEP_INTERSECTION_HALFPLANE_3_H    // -*- C++ -*-
#define CEP_INTERSECTION_HALFPLANE_3_H

#include <CGAL/Object.h>
#include <CGAL/Point_3.h>
#include <CGAL/Segment_3.h>
#include <CGAL/predicates_on_points_3.h>

#include <CEP/intersection/Line_3_Line_3.h>


namespace CEP {  namespace intersection {

    using CGAL::Point_3;
    using CGAL::Segment_3;

    using CGAL::assign;
    using CGAL::make_object;


    namespace Halfplane_3_Segment_3 {

	//! Compute intersection of halfplane with coplanar segment
	/*!
	 * The halfplane H is specified by three noncollinear points p, q, r.
	 * H is half the plane determined by (p,q,r).  Points p and q are
	 * on the boundary of H, and r is in the interior.
	 *
	 * \pre p,q,r not collinear
	 * \pre segment is coplanar with p,q,r
	 */
	template <class R_>
	CGAL::Object coplanar_intersection( const Point_3<R_>& p,
					    const Point_3<R_>& q,
					    const Point_3<R_>& r,
					    const Segment_3<R_>& segment )
        {
	    CGAL_exactness_precondition( ! CGAL::collinear(p,q,r) );
	    CGAL_exactness_precondition( CGAL::coplanar(p,q,r, segment.source()) );
	    CGAL_exactness_precondition( CGAL::coplanar(p,q,r, segment.target()) );

	    bool source_on_halfplane 
		= CGAL::coplanar_orientation(p,q,r, segment.source() ) != CGAL::NEGATIVE;
	    bool target_on_halfplane 
		= CGAL::coplanar_orientation(p,q,r, segment.target() ) != CGAL::NEGATIVE;

	    if ( source_on_halfplane ) {
		if ( target_on_halfplane ) {
		    return make_object(segment);
		} else {
		    CGAL::Object i = CEP::intersection::Line_3_Line_3::coplanar_nonparallel_intersection
			( p,q, segment.source(),segment.target() );
		    Point_3<R_> pi;
		    bool intersection_is_point = assign( pi, i );
		    CGAL_assertion( intersection_is_point );
		    return make_object(Segment_3<R_>(pi,segment.source()));
		}
	    } else if ( target_on_halfplane ) {
		CGAL::Object i = CEP::intersection::Line_3_Line_3::coplanar_nonparallel_intersection
		    ( p,q, segment.source(),segment.target() );
		Point_3<R_> pi;
		bool intersection_is_point = assign( pi, i );
		CGAL_assertion( intersection_is_point );
		return make_object(Segment_3<R_>(pi,segment.target()));
	    }

	    return CGAL::Object();
	}

    }
}
}

	    
    


#endif
