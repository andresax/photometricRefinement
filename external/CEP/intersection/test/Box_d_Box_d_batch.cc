#include "tester.h"
#include "boxdef.h"

#include <utility>
#include <vector>

#include <CGAL/IO/Verbose_ostream.h>
#include <CEP/intersection/Box_d_Box_d_batch.h>


using namespace CGAL;
using namespace CEP::intersection;

// In addition to the requirements for the Box listed in Box_d_Box_d.h,
// the box must be of dimension 2, and possess a constructor with the
// following signature:
//     Box::Box( min0, max0, min1, max1 )



template <class Box>
class Box_d_Box_d_batch_test : public CppUnit::TestCase
{
public:
    typedef std::vector<Box>       Box_list;
    typedef std::pair<Box,Box>     Box_pair;
    typedef std::vector<Box_pair>  BP_list;


    Box box0,box1,box2,box3,box4;
    Box_list input;
    Verbose_ostream verr;

    void setUp() 
    {
	box0 = Box( 0,10, 0,10 );
	box1 = Box( 0,10, 11,12 );
	box2 = Box( -5,-1, 0,10 );
	box3 = Box( 4,6, 4,6 );
	box4 = Box( 10,12, 10,12 );

	input.push_back(box0);
	input.push_back(box1);
	input.push_back(box2);
	input.push_back(box3);
	input.push_back(box4);
    }

    void tearDown()
    {
	input.clear();
    }

    void basic() 
    {
	CPPUNIT_ASSERT( Box::dimension == 2 );

	CPPUNIT_ASSERT( do_intersect(box0,box3) );
	CPPUNIT_ASSERT( do_intersect(box0,box4) );
	CPPUNIT_ASSERT( do_intersect(box1,box4) );
    }

    void check_batch_result( const BP_list& res ) 
    {
	int pair_count = 0;

	verr << "List size = " << res.size() << std::endl;

	for( typename BP_list::const_iterator i = res.begin(); i != res.end(); ++i ) {
	    // Don't count self-intersections
	    if ( i->first != i->second ) {
		CPPUNIT_ASSERT( do_intersect(i->first,i->second) );
		++pair_count;
	    }
	}
	verr << "Count: " << pair_count << std::endl;
	// Each pair is counted twice: (a,b) and (b,a) are both on list
	CPPUNIT_ASSERT_EQUAL( 2*3, pair_count );
    }

    void allpair() 
    {
	BP_list res;

	Box_d_Box_d_batch_allpair( input.begin(), input.end(),
				   input.begin(), input.end(),
				   std::back_inserter(res) );
	check_batch_result( res );
    }

    void scan() 
    {
	BP_list res;
	Box_d_Box_d_batch_scan( input.begin(), input.end(),
				input.begin(), input.end(),
				std::back_inserter(res) );
	check_batch_result( res );
    }

    void stream() 
    {
	Box_list input2(input);
	BP_list res;
	Box_d_Box_d_batch_stream( input.begin(), input.end(),
				  input2.begin(), input2.end(),
				  std::back_inserter(res) );
	check_batch_result( res );
    }

    // Testing with a small set will not exercise the general 
    // stream code, since the base case cutoff is quite large.
    // Here we enlarge the test by adding in several boxes
    // that do not intersect.
    void stream_large() 
    {
	verr << "Input size: " << input.size() << std::endl;
	
	for( int i = -300; i < 300; i += 2 )
	    for( int j = -300; j < 300; j += 2 )
		// Omit the middle square where box0 -- box4 lie
		if ( (i < -20 || i > 20) &&
		     (j < -20 || j > 20) ) {
		    input.push_back( Box( i,i+1, j,j+1 ) );
		}

	verr << "Input size: " << input.size() << std::endl;
	Box_list input2(input);
	BP_list res;
	Box_d_Box_d_batch_stream( input.begin(), input.end(),
				  input2.begin(), input2.end(),
				  std::back_inserter(res) );
	verr << "Input size: " << input.size() << std::endl;
	check_batch_result( res );
    }


    // To divide and conquer, first you must divide.
    // If the code does not detect cases when the median is at
    // one end of the range, you can get an infinite recursion
    void stream_divide() 
    {
	CPPUNIT_ASSERT( Box::dimension == 2 );

	Box_list setA;
	setA.push_back( Box(1,6, 2,7) );
	setA.push_back( Box(2,7, 3,6) );
	setA.push_back( Box(3,8, 3,7) );
	setA.push_back( Box(2,5, 3,8) );
	setA.push_back( Box(3,7, 1,2) );

	Box_list setB;
	setB.push_back( Box(1,5, 2,7) );
	setB.push_back( Box(1,4, 2,6) );
	setB.push_back( Box(2,7, 3,5) );
	setB.push_back( Box(1,2, 2,7) );
	setB.push_back( Box(1,4, 2,7) );

	// At the first invocation of stream_inner, the setB
	// is treated as the point set {2,2,2,2,3}, which has a 
	// median value 2.  To generate the sub-problems (to be
	// solved recursively), the point set is split using "less than",
	// which means the set is NOT split at all: one sub-problem has
	// size 0 and the other size 5.  Note that splitting with
	// a "less than or equals" operator will not solve the problem,
	// since it will have a bad split on the set {1,2,2,2,2}.

	// To exhibit this problem, we need to have the "base case size"
	// lowered.
	int bct_save = CEP::intersection::base_case_threshold;
	CEP::intersection::base_case_threshold = 3;

	BP_list results;
	Box_d_Box_d_batch_stream( setA.begin(), setA.end(),
				  setB.begin(), setB.end(),
				  std::back_inserter(results) );

	CEP::intersection::base_case_threshold = bct_save;
    }


    CPPUNIT_TEST_SUITE( Box_d_Box_d_batch_test );
    CPPUNIT_TEST( basic );
    CPPUNIT_TEST( allpair );
    CPPUNIT_TEST( scan );
    CPPUNIT_TEST( stream );

#if !BOX_D_BOX_D_BATCH_TRACE_EXECUTION
    CPPUNIT_TEST( stream_large );
#endif
    CPPUNIT_TEST( stream_divide );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Box_d_Box_d_batch_test<Box_2f> );

