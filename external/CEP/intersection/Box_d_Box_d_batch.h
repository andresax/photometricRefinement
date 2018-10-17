#ifndef CEP_INTERSECTION_BOX_D_BOX_D_BATCH_H
#define CEP_INTERSECTION_BOX_D_BOX_D_BATCH_H

#include <CEP/intersection/Box_d_Box_d.h>


#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
#include <CEP/intersection/Indentation.h>
   Indentation indent;
#endif



namespace CEP {
    namespace intersection {


/*! \defgroup grp_box_box_batch Box_d_Box_d_batch
 *  \ingroup grp_batch
 *  \brief Compute set of pairwise intersections in batch mode.
 *
 * The batch intersection problem is the following.
 * \pre BoxInIter is input iterator with value type Box_d
 * \pre BoxPairOutIter is output iterator 
 *                     with value type std::pair<Box_d,Box_d>
 * \pre [begin1, end1) is a valid sequence of Box_d
 * \pre [begin2, end2) is a valid sequence of Box_d
 *
 * \post All intersecting pairs (b1,b2) where b1 is a box of
 *       set boxes1, and b2 belongs to the set boxes2 have been
 *       inserted into output container.
 *
 * The algorithms assume the two sets of boxes are distinct.
 * If they are the same set, you must filter the output to discard
 * the self-intersections.
 *
 * The current implementations may report box pairs multiple times,
 * so you need to filter the output to remove duplicates.
 *
 * \warning The ordering of the input sequences may not be preserved.
 *
 * @{
 */


//! Test all pairs for intersection.
/*!
 * This is the brute force approach, with O(N^2) time complexity.
 */
template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch_allpair( BoxInIter begin1,
				BoxInIter end1,
				BoxInIter begin2,
				BoxInIter end2,
				BoxPairOutIter out );
 


//! Sort and scan along one axis.
/*!
 * This useful for one-dimensional boxes, and for small numbers
 * of boxes.
 */
template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch_scan( BoxInIter begin1,
			     BoxInIter end1,
			     BoxInIter begin2,
			     BoxInIter end2,
			     BoxPairOutIter out );




//! Use the streaming approach of Edelsbrunner and Overmars
/*!
 * This is the method to use for large numbers of boxes.
 */
template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch_stream( BoxInIter begin1,
			       BoxInIter end1,
			       BoxInIter begin2,
			       BoxInIter end2,
			       BoxPairOutIter out );



//! Use best intersection approach.
template <class BoxInIter, class BoxPairOutIter>
void Box_d_Box_d_batch( BoxInIter begin1,
			BoxInIter end1,
			BoxInIter begin2,
			BoxInIter end2,
			BoxPairOutIter out )
{
    Box_d_Box_d_batch_stream( begin1, end1, begin2, end2, out );
}


/* @} */

 int base_case_threshold;

}
}


#ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
#  include <CEP/intersection/Box_d_Box_d_batch.C>
#endif


#endif
