#include <utility>
#include <iterator>


namespace CEP {
    namespace intersection {


// Naive O(n^2) brute force algorithm
template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch_allpair( BoxInIter begin1,
				BoxInIter end1,
				BoxInIter begin2,
				BoxInIter end2,
				BoxPairOutIter out )
{
    typedef typename std::iterator_traits<BoxInIter>::value_type  Box_;
    typedef std::pair<Box_,Box_>                                  Box_pair;

    for( BoxInIter i1 = begin1; i1 != end1; ++i1 ) {
        for( BoxInIter i2 = begin2; i2 != end2; ++i2 ) {
            if (do_intersect( *i1, *i2 )) {
                *out++ = Box_pair(*i1,*i2);
	    }
        }
    }
}



}
}
