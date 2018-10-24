#include <CGAL/circulator.h>
#include <CGAL/Line_2_Line_2_intersection.h>


namespace CEP {
    namespace intersection {


/*!
 * \param InputIterator an input iterator from a container with 
 *                      elements of Point_2.
 * \param OutputIterator an output iterator from a container with
 *                       elements of Point_2.
 *
 * The input polygon consists of the sequence [points_begin,points_end).
 * The input polygon should be convex; this is not currently checked for.
 *
 * \pre Input polygon is not empty.
 */
template <class R, class InputIterator, class OutputIterator>
void slice( InputIterator points_begin, InputIterator points_end,
	    const CGAL::Line_2<R>& l,
	    OutputIterator left_chain,
	    OutputIterator right_chain )
{
    using CGAL::assign;

    CGAL_assertion_msg( points_begin != points_end, 
			"polygon must be non-empty" );

    typedef CGAL::Circulator_from_iterator<InputIterator> Circulator;
    typedef typename std::iterator_traits<InputIterator>::value_type Point;

    Circulator p_begin( points_begin, points_end );
    Circulator p = p_begin;

    do {
	CGAL::Oriented_side side = l.oriented_side(*p);

	if ( side == CGAL::ON_POSITIVE_SIDE )
	    *left_chain++ = *p;
	else if ( side == CGAL::ON_NEGATIVE_SIDE )
	    *right_chain++ = *p;
	else {
	    *left_chain++ = *p;
	    *right_chain++ = *p;
	}

	if ( side != CGAL::ON_ORIENTED_BOUNDARY ) {
	    Circulator q(p);  
	    ++q;
	    CGAL::Oriented_side q_side = l.oriented_side(*q);

	    if ( q_side != side && q_side != CGAL::ON_ORIENTED_BOUNDARY ) {
		CGAL::Object o_int 
		    = CGAL::intersection( l, CGAL::Line_2<R>(*p,*q) );
		Point p_int;
		bool intersection_is_point = assign(p_int,o_int);
		CGAL_assertion( intersection_is_point );
		*left_chain++ = p_int;
		*right_chain++ = p_int;
	    }
	}
    } while( ++p != p_begin );
}


}
}

