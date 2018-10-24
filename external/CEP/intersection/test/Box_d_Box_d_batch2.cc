#include <utility>
#include <vector>
#include <list>
#include <algorithm>
#include <math.h>

#include <CGAL/Random.h>
#include <CGAL/IO/Verbose_ostream.h>

#include "tester.h"
#include "boxdef.h"
#include <CEP/intersection/Box_d_Box_d_batch.h>


using namespace CGAL;
using namespace CEP::intersection;

// In addition to the requirements for the Box listed in Box_d_Box_d.h,
// the box must be of dimension 5, and possess a constructor with the
// following signature:
//     Box::Box( min0, max0, min1, max1, ..., min4, max4 )


// Generate boxes using the "balanced distribution" of Zomo+Edels
template <class Box>
struct generate_box_balanced 
{
    typedef typename Box::FT FT;

    int leftend_range, length_range;

    generate_box_balanced( int n_box )
    {
	length_range = (int) pow( n_box, 4.0/5.0 );
	leftend_range = n_box - length_range;
    }

    Box operator()() const
    {
	FT min0 = CGAL::default_random.get_int(1,leftend_range);
	FT max0 = min0 + CGAL::default_random.get_int(1,length_range);
	FT min1 = CGAL::default_random.get_int(1,leftend_range);
	FT max1 = min1 + CGAL::default_random.get_int(1,length_range);
	FT min2 = CGAL::default_random.get_int(1,leftend_range);
	FT max2 = min2 + CGAL::default_random.get_int(1,length_range);
	FT min3 = CGAL::default_random.get_int(1,leftend_range);
	FT max3 = min3 + CGAL::default_random.get_int(1,length_range);
	FT min4 = CGAL::default_random.get_int(1,leftend_range);
	FT max4 = min4 + CGAL::default_random.get_int(1,length_range);

	return Box( min0,max0, min1,max1, min2,max2, min3,max3, min4,max4 );
    }
};


template <class Box>
class Box_d_Box_d_batch_test2 : public CppUnit::TestCase
{
public:
    typedef std::vector<Box>       Box_list;
    typedef std::pair<Box,Box>     Box_pair;
    typedef std::list<Box_pair>    BP_list;


    Verbose_ostream verr;

    Box_d_Box_d_batch_test2() {
	verr = Verbose_ostream(true);
    }

    struct my_canonical_order
    {
	Box_pair operator() ( const Box_pair& bp )
	{
	    return canonical_order(bp);
	}
    };

    void canonicalize_list( std::list<Box_pair>& l )
    {
	my_canonical_order co;
	std::transform( l.begin(), l.end(), l.begin(), co );
    }


    void compare_lists( BP_list& l1, BP_list& l2 )
    {
	typedef typename BP_list::size_type size_type;

	size_type l1_size = l1.size();
	size_type l2_size = l2.size();

	canonicalize_list( l1 );
	canonicalize_list( l2 );

	l1.sort();
	l2.sort();

	CPPUNIT_ASSERT_EQUAL( l1_size, l1.size() );
	CPPUNIT_ASSERT_EQUAL( l2_size, l2.size() );

	l1.unique();
	l2.unique();

        if ( l1_size != l1.size() )
	    verr << "List 1 changed size from " << l1_size
		 << " to " << l1.size() << std::endl;

        if ( l2_size != l2.size() )
	    verr << "List 2 changed size from " << l2_size
		 << " to " << l2.size() << std::endl;

	while( !l1.empty() && !l2.empty() ) {
	    Box_pair bp1 = l1.front();
	    Box_pair bp2 = l2.front();

	    //verr << "Pair1: " << bp1 << std::endl;
	    //verr << "Pair2: " << bp2 << std::endl;

	    CPPUNIT_ASSERT( do_intersect(bp1.first, bp1.second) );
	    CPPUNIT_ASSERT( do_intersect(bp2.first, bp2.second) );
	    CPPUNIT_ASSERT_EQUAL( bp1, bp2 );

	    // Remove top element and all duplicates
	    while( bp1 == l1.front() )
		l1.pop_front();

	    while( bp2 == l2.front() )
		l2.pop_front();
	}

	CPPUNIT_ASSERT( l1.empty() );
	CPPUNIT_ASSERT( l2.empty() );
    }


    // Test a large, random data set; ensure _stream and _scan
    // give same results.
    void stream_vs_scan() 
    {
	// Mysteriously, this stopped working with GCC 3.0 ...
	//CPPUNIT_ASSERT_EQUAL( Box::dimension, 5 );
	CPPUNIT_ASSERT( Box::dimension == 5 );

	// Lower the base case threshold to make sure the recursion
	// is well tested.
	CEP::intersection::base_case_threshold = 3;
	const typename Box_list::size_type n1 = 1000;
	const typename Box_list::size_type n2 = 1000;

	Box_list boxes1(n1);
	Box_list boxes2(n2);

	std::generate( boxes1.begin(), boxes1.end(), 
			generate_box_balanced<Box>(n1+n2) );
	std::generate( boxes2.begin(), boxes2.end(), 
			generate_box_balanced<Box>(n1+n2) );

	CPPUNIT_ASSERT_EQUAL( n1, boxes1.size() );
	CPPUNIT_ASSERT_EQUAL( n2, boxes2.size() );

	BP_list res_scan;
	std::cerr << "Start Box_d_Box_d_batch_scan" << std::endl;
	Box_d_Box_d_batch_scan( boxes1.begin(), boxes1.end(),
				boxes2.begin(), boxes2.end(),
				std::back_inserter(res_scan) );

	CPPUNIT_ASSERT_EQUAL( n1, boxes1.size() );
	CPPUNIT_ASSERT_EQUAL( n2, boxes2.size() );

#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
	CPPUNIT_ASSERT_EQUAL( 0, indent.num_spaces() );
#endif

	BP_list res_stream;
	std::cerr << "Start Box_d_Box_d_batch_stream" << std::endl;
	Box_d_Box_d_batch_stream( boxes1.begin(), boxes1.end(),
				  boxes2.begin(), boxes2.end(),
				  std::back_inserter(res_stream) );

	CPPUNIT_ASSERT_EQUAL( n1, boxes1.size() );
	CPPUNIT_ASSERT_EQUAL( n2, boxes2.size() );

#if BOX_D_BOX_D_BATCH_TRACE_EXECUTION
	CPPUNIT_ASSERT_EQUAL( 0, indent.num_spaces() );
#endif

	compare_lists( res_scan, res_stream );
    }


    CPPUNIT_TEST_SUITE( Box_d_Box_d_batch_test2 );
    CPPUNIT_TEST( stream_vs_scan );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Box_d_Box_d_batch_test2<Box_5f> );
