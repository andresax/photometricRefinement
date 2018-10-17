#include <algorithm>
#include <CGAL/Random.h>

#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
#  include <CEP/intersection/Indentation.h>
#  define RECURSE_ENTER  ++indent;
#  define RECURSE_LEAVE  --indent;
#else
#  define RECURSE_ENTER  
#  define RECURSE_LEAVE  
#endif



namespace CEP {
    namespace intersection {


// When the point list is less than this amount, the base
// case brute-force checking is invoked.  This must be >= 1.
int base_case_threshold = 5000;


template<class Box>
inline
bool does_contain( const Box& b, 
		   typename Box::FT low, 
		   typename Box::FT high, 
		   int dim )
{
    return b.min(dim) <= low && high <= b.max(dim);
}


template <class Box>
struct does_contain_pred : public std::unary_function<Box,bool>
{
    typedef typename Box::FT FT;

    FT low, high;
    int dim;

    does_contain_pred( FT l, FT h, int d ) : low(l),high(h),dim(d) {};

    bool operator() ( const Box& b )
    {
        return does_contain( b, low, high, dim );
    }
};



/* True if (closed) d'th interval on box intersects closed interval
   [low,high] */
template <class Box>
inline 
bool do_intersect( const Box& b,
		   typename Box::FT low, 
		   typename Box::FT high, 
		   int d )
{
    return b.min(d) <= high && low <= b.max(d);
}


template <class Box>
struct do_intersect_pred : public std::unary_function<Box,bool>
{
    typedef typename Box::FT FT;

    FT low, high;
    int dim;

    do_intersect_pred( FT l, FT h, int d ) : low(l),high(h),dim(d) {};

    bool operator() ( const Box& b )
    {
        return do_intersect( b, low, high, dim );
    }
};



/* Compare cuboids' lower (leftmost) endpoint on d'th axis */
template <class Box>
struct less_than_fixed_value : public std::unary_function<Box,bool>
{
    typedef typename Box::FT FT;

    FT bound;
    int dim;

    less_than_fixed_value( FT b, int d ) : bound(b),dim(d) {};

    bool operator() ( const Box& b )
    {
        return b.min(dim) < bound;
    }
};

template <class Box>
struct equal_fixed_value : public std::unary_function<Box,bool>
{
    typedef typename Box::FT FT;

    FT bound;
    int dim;

    equal_fixed_value( FT b, int d ) : bound(b),dim(d) {};

    bool operator() ( const Box& b )
    {
        return b.min(dim) == bound;
    }
};




// Computer interval that contains list of points
template <class BoxInIter>
void 
get_interval( BoxInIter begin, BoxInIter end, int d,
	      typename std::iterator_traits<BoxInIter>::value_type::FT& low,
	      typename std::iterator_traits<BoxInIter>::value_type::FT& high )
{
    typedef typename std::iterator_traits<BoxInIter>::value_type::FT FT;

    CGAL_assertion( begin != end );

    low = begin->min(d);
    high = begin->min(d);

    for( BoxInIter p = begin; p != end; ++p ) {
	if ( p->min(d) < low ) 
	    low = p->min(d);
	else if ( p->min(d) > high )
	    high = p->min(d);
    }
}



// Forward declaration of inner recursive procedure
template< class BoxInIter, class BoxPairOutIter >
void stream_inner( BoxInIter segments_begin,
		   BoxInIter segments_end,
		   typename std::iterator_traits<BoxInIter>::value_type::FT low,
		   typename std::iterator_traits<BoxInIter>::value_type::FT high,
		   BoxInIter points_begin,
		   BoxInIter points_end,
		   int d,
		   BoxPairOutIter out );


// Outer recursive procedure
template< class BoxInIter, class BoxPairOutIter >
void stream_outer( BoxInIter begin1,
		   BoxInIter end1,
		   BoxInIter begin2,
		   BoxInIter end2,
		   int d,
		   BoxPairOutIter out )
{
#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
    RECURSE_ENTER
    cerr << indent << "stream_outer( d = " << d << " )" << endl;
    cerr << indent << "*** Box list 1 ***" << endl;
    for( BoxInIter i = begin1; i != end1; ++i )
	cerr << indent << *i << endl;
    cerr << indent << "*** Box list 2 ***" << endl;
    for( BoxInIter i = begin2; i != end2; ++i )
	cerr << indent << *i << endl;
#endif

    typedef typename std::iterator_traits<BoxInIter>::value_type  Box_;
    typedef std::pair<Box_,Box_>                                  BoxPair;
    typedef typename Box_::FT                                     FT;

    if ( begin1 == end1 || begin2 == end2 ) {
	RECURSE_LEAVE
	return;
    }

    // On the last dimension, do a "sort and scan"
    if ( d == 0 ) {
	Box_d_Box_d_batch_scan( begin1, end1, begin2, end2, out );
	RECURSE_LEAVE
	return;
    }

    // Construct segment tree with the first set of boxes and query
    // with the second set, then vice-versa.

    FT low, high;
    get_interval( begin2, end2, d, low, high );
    stream_inner( begin1, end1,
		  low, high,
		  begin2, end2, d, out );

    get_interval( begin1, end1, d, low, high );
    stream_inner( begin2, end2,
		  low, high,
		  begin1, end1, d, out );

    RECURSE_LEAVE
}




template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch_stream( BoxInIter begin1,
			       BoxInIter end1,
			       BoxInIter begin2,
			       BoxInIter end2,
			       BoxPairOutIter out )
{
    typedef typename std::iterator_traits<BoxInIter>::value_type  Box_;

    stream_outer( begin1, end1, begin2, end2, 
		  Box_::dimension - 1, out );
}


// Median of three random values
template<class BoxInIter>
inline
typename std::iterator_traits<BoxInIter>::value_type::FT
approximate_median( BoxInIter begin,
		    int size, 
		    int d )
{
    typedef typename std::iterator_traits<BoxInIter>::value_type  Box_;
    typedef typename Box_::FT        FT;

    CGAL_assertion( size > 0 );

    FT val0, val1, val2;

    if ( size > 3 ) {
	val0 = (begin + CGAL::default_random.get_int(0,size))->min(d);
	val1 = (begin + CGAL::default_random.get_int(0,size))->min(d);
	val2 = (begin + CGAL::default_random.get_int(0,size))->min(d);
    } else {
	if ( size == 3 ) {
	    val0 = (begin+0)->min(d);
	    val1 = (begin+1)->min(d);
	    val2 = (begin+2)->min(d);
	} else {
	    if ( size == 2 ) {
		val0 = (begin+0)->min(d);
		val1 = (begin+1)->min(d);
		return std::max(val0,val1);
	    } else {
		return begin->min(d);
	    }
	}
    }

    if ( val0 <= val1 ) {
	if ( val1 <= val2 )       // val0 <= val1 <= val2
	    return val1;
	if ( val0 <= val2 )       // val0 <= val2 < val1
	    return val2;
	return val0;              // val2 < val0 <= val1
    } else {
	if ( val0 <= val2 )       // val1 < val0 <= val2
	    return val0;
	if ( val1 <= val2 )       // val1 <= val2 < val0
	    return val2;
	return val1;              // val2 < val1 < val0
    }
    CGAL_assertion_msg(0,"should not reach here");
    return 0;
}



/* Preconditions: d > 0
 * All points lie in closed interval [low,high]
 *
 * Orderings of the input lists are not preserved.
 *
 * The algorithm is described in 
 *   Fast Software for Box Intersections
 *   ACM Symposium on Computational Geometry, 2000
 *   Afra Zomorodian and Herbert Edelsbrunner
 *
 * The first list of boxes is used as the segments (the d'th interval
 * of each box) that make up the tree, and the second list is treated
 * as the set of query points (the low point on the d'th interval).
 */
template< class BoxInIter, class BoxPairOutIter >
void 
stream_inner( BoxInIter segments_begin,
	      BoxInIter segments_end,
	      typename std::iterator_traits<BoxInIter>::value_type::FT low,
	      typename std::iterator_traits<BoxInIter>::value_type::FT high,
	      BoxInIter points_begin,
	      BoxInIter points_end,
	      int d,
	      BoxPairOutIter out )
{
#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
    RECURSE_ENTER
    cerr << indent << "stream_inner(" 
	 << " d = " << d
	 << " low = " << low
	 << " high = " << high
	 << " )" << endl;
    cerr << indent << "*** Segment list ***" << endl;
    for( BoxInIter i = segments_begin; i != segments_end; ++i )
	cerr << indent << *i << endl;
    cerr << indent << "*** Point list ***" << endl;
    for( BoxInIter i = points_begin; i != points_end; ++i )
	cerr << indent << *i << endl;
#endif

    typedef typename std::iterator_traits<BoxInIter>::value_type  Box_;
    typedef typename Box_::FT        FT;

    int n_segments = segments_end - segments_begin;
    int n_points = points_end - points_begin;

    CGAL_assertion( low <= high );

    if ( n_segments == 0 || n_points == 0 ) {
	RECURSE_LEAVE
	return;
    }

    if ( n_points <= base_case_threshold ||
	 n_segments <= base_case_threshold ) 
	{
	    scan_oneway( segments_begin, segments_end,
			 points_begin, points_end,
			 d, out );
	    RECURSE_LEAVE
	    return;
	}


    // Compute the canonical set of segments for this "node"
    // of the segment tree.  All such segments are stabbed by
    // the point set, so recurse one dimension lower.

    BoxInIter cs_end = std::partition( segments_begin, segments_end,
				       does_contain_pred<Box_>(low,high,d) );
    if ( cs_end != segments_begin ) {
	stream_outer( segments_begin, cs_end,
		      points_begin, points_end,
		      d-1, out );
    }

    // Consider the children of this node.  Intervals that appear
    // in the canonical set for this node will not appear in the set
    // of any successor node.  From now on, the set of intervals under
    // consideration is the range [cs_end,segments_end).  

    // Split the point set into 
    // [points_begin,points_mid) and 
    // [points_mid,points_end)

    FT mid = approximate_median( points_begin, n_points, d );
    BoxInIter points_mid = std::partition( points_begin, points_end,
					   less_than_fixed_value<Box_>(mid,d) );

#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
    cerr << indent << "midpoint value = " << mid << endl;
#endif

    // The partition [points_begin,points_mid) contains elements
    // strictly < median value.  It can happen --- if there are
    // many points of median value --- that this partition is empty.
    // To correct this, we re-partition with up to n/2 points that
    // precisely equal the median value.
    if ( points_mid == points_begin ) {
	points_mid = std::partition( points_begin, points_end,
				     equal_fixed_value<Box_>(mid,d) );
	CGAL_assertion( points_mid - points_begin > 0 );
	if ( points_mid - points_begin > n_points/2 )
	    points_mid = points_begin + n_points/2;
    }

    BoxInIter split = std::partition( cs_end, segments_end,
				      do_intersect_pred<Box_>(low,mid,d) );
    stream_inner( cs_end, split, low, mid,
		  points_begin, points_mid,
		  d, out );

    split = std::partition( cs_end, segments_end,
			    do_intersect_pred<Box_>(mid,high,d) );
    stream_inner( cs_end, split, mid, high,
		  points_mid, points_end,
		  d, out );

    RECURSE_LEAVE
}


}
}

