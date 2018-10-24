#include "tester.h"
#include "boxdef.h"
#include <CEP/intersection/Box_d_Box_d.h>


using namespace CGAL;
using namespace CEP::intersection;

// In addition to the requirements for the Box listed in Box_d_Box_d.h,
// the box must be of dimension 2, and possess a constructor with the
// following signature:
//     Box::Box( min0, max0, min1, max1 )


template <class Box>
class Box_d_Box_d_test : public CppUnit::TestCase
{
public:
    typedef typename Box::FT FT;
    typedef std::pair<Box,Box>  Box_pair;

    void testit() {
	CPPUNIT_ASSERT( Box::dimension == 2 );

	Box box0( 0,10, 0,10 );
	Box box1( 0,10, 11,12 );
	Box box2( -5,-1, 0,10 );

	Box_pair bp1(box0,box2);
	Box_pair bp2(box0,box2);

	CPPUNIT_ASSERT_EQUAL( box0, box0 );
	CPPUNIT_ASSERT_EQUAL( bp1, bp2 );

	CPPUNIT_ASSERT(  Box_d_Box_d::do_intersect(box0,box1,0) );
	CPPUNIT_ASSERT( !Box_d_Box_d::do_intersect(box0,box1,1) );
	CPPUNIT_ASSERT( !do_intersect(box0,box1)   );

	CPPUNIT_ASSERT( !Box_d_Box_d::do_intersect(box0,box2,0) );
	CPPUNIT_ASSERT(  Box_d_Box_d::do_intersect(box0,box2,1) );
	CPPUNIT_ASSERT( !do_intersect(box0,box2)   );

	CPPUNIT_ASSERT( do_intersect(box0,Box(4,6,4,6)) );
	CPPUNIT_ASSERT( do_intersect(box0,Box(-1,11,-1,11)) );

	CPPUNIT_ASSERT( do_intersect(box0,Box(10,12,10,12)) );
	CPPUNIT_ASSERT( do_intersect(box0,Box(-1,0,10,12)) );
    }


    CPPUNIT_TEST_SUITE( Box_d_Box_d_test );
    CPPUNIT_TEST( testit );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Box_d_Box_d_test<Box_2f> );
