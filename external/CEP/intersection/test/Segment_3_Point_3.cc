// -*- C++ -*-

#include "tester.h"
#include <CEP/intersection/Segment_3_Point_3.h>


using namespace CGAL;
using namespace CEP::intersection;


template <class R>
class Segment_3_Point_3_test : public CppUnit::TestCase
{
    typedef CGAL::Point_3<R>    Point;
    typedef CGAL::Segment_3<R>  Segment;

public:

    void tests()
    {
	// Points on line (x,-8x+3,3x+20)
	Point a(-1,11,17);
	Point b(5,-37,35);
	Point c(6,-45,38);

	Segment ac(a,c);
	Segment ca(c,a);

	CPPUNIT_ASSERT( do_intersect(ac,b) );
	CPPUNIT_ASSERT( do_intersect(ca,b) );

	ASSERT_EQUAL_OBJECT( b, intersection(ac,b) );
	ASSERT_EQUAL_OBJECT( b, intersection(ca,b) );

	CPPUNIT_ASSERT( collinear(ac,b) );
	CPPUNIT_ASSERT( collinear(ca,b) );
    }

    CPPUNIT_TEST_SUITE( Segment_3_Point_3_test );
    CPPUNIT_TEST( tests );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Segment_3_Point_3_test<R_test> );
