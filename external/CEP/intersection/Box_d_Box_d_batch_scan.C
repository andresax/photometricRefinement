#include <functional>

#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
#  include <CEP/intersection/Indentation.h>
#endif



namespace CEP {
    namespace intersection {


// Precondition: segments and points are both sorted along axis d
// Do linear scan for overlaps on axis d.
template <class BoxInIter, class BoxPairOutIter>
void scan_inner( BoxInIter segments_begin,
		 BoxInIter segments_end,
		 BoxInIter points_begin,
		 BoxInIter points_end,
		 int d,
		 BoxPairOutIter out )
{
#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
    cerr << ++indent << "scan_inner( d = " << d << " )" << endl;
    cerr << indent << "*** Segment list ***" << endl;
    for( BoxInIter i = segments_begin; i != segments_end; ++i )
	cerr << indent << *i << endl;
    cerr << indent << "*** Point list ***" << endl;
    for( BoxInIter i = points_begin; i != points_end; ++i )
	cerr << indent << *i << endl;
#endif

    typedef typename std::iterator_traits<BoxInIter>::value_type  Box;
    typedef std::pair<Box,Box>                                    Box_pair;

    BoxInIter p_start = points_begin;

    for( BoxInIter s = segments_begin; s != segments_end; ++s ) {

	BoxInIter p = p_start;

	// Find first point not left of segment s
	while( p != points_end ) {
	    if ( p->min(d) < s->min(d) )
		++p;
	    else
		break;
	}

	// restart point scan from here, for next segment
	p_start = p;

	// Output for all points lying inside (half-open) segment
	while( p != points_end ) {
	    if ( p->min(d) > s->max(d) )
		break;
            if (do_intersect( *s, *p )) {
		// yes, this does one more compare than necessary ...
                *out++ = Box_pair(*s,*p);
	    }
	    ++p;
	}
    }

#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
    --indent;
#endif
}


/* Compare lower endpoint of boxes along d'th axis */
template <class Box>
struct less_than_pred : public std::binary_function<Box,Box,bool>
{
    int dim;
    less_than_pred( int d ) : dim(d) {};

    bool operator() ( const Box& a, const Box& b )
    {
        return a.min(dim) < b.min(dim);
    }
};



template <class BoxInIter, class BoxPairOutIter>
void scan_oneway( BoxInIter segments_begin,
		  BoxInIter segments_end,
		  BoxInIter points_begin,
		  BoxInIter points_end,
		  int d,
		  BoxPairOutIter out )
{
    typedef typename std::iterator_traits<BoxInIter>::value_type  Box;

    std::sort( segments_begin, segments_end, less_than_pred<Box>(d) );
    std::sort( points_begin, points_end, less_than_pred<Box>(d) );

    scan_inner( segments_begin, segments_end, 
		points_begin, points_end,
		d, out );
}



template <class BoxInIter, class BoxPairOutIter>
void scan_twoway( BoxInIter begin1,
		  BoxInIter end1,
		  BoxInIter begin2,
		  BoxInIter end2,
		  int d,
		  BoxPairOutIter out )
{
    typedef typename std::iterator_traits<BoxInIter>::value_type  Box;

    std::sort( begin1, end1, less_than_pred<Box>(d) );
    std::sort( begin2, end2, less_than_pred<Box>(d) );

    scan_inner( begin1, end1, begin2, end2, d, out );
    scan_inner( begin2, end2, begin1, end1, d, out );
}



template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch_scan( BoxInIter begin1,
			     BoxInIter end1,
			     BoxInIter begin2,
			     BoxInIter end2,
			     BoxPairOutIter out )
{
    scan_twoway( begin1, end1, begin2, end2, 0, out );
}



}
}
