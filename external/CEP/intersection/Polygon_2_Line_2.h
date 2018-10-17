#ifndef CEP_INTERSECTIN_POLYGON_2_LINE_2_H
#define CEP_INTERSECTIN_POLYGON_2_LINE_2_H

#include <CGAL/Line_2.h>
#include <CGAL/Point_2.h>


namespace CEP {
    namespace intersection {


//! Slice a convex polygon into two pieces with a line.
template <class R, class InputIterator, class OutputIterator>
void slice( InputIterator points_begin, 
	    InputIterator points_end,
	    const CGAL::Line_2<R>& l,
	    OutputIterator left_chain,
	    OutputIterator right_chain );

}
}

#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Polygon_2_Line_2.C>
#endif


#endif

