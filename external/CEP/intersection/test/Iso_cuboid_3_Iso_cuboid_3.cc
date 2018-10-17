// -*- C++ -*-

#include "tester.h"
#include <CEP/intersection/Iso_cuboid_3_Iso_cuboid_3.h>


using namespace CGAL;
using namespace CEP::intersection;


template <class R>
class Iso_cuboid_3_Iso_cuboid_3_test : public CppUnit::TestCase
{
    typedef CGAL::Iso_cuboid_3<R>   Cuboid;
    typedef CGAL::Point_3<R>        Point;

public:
    void test_do_intersect() {
	Cuboid c0( Point(0,0,0), Point(10,10,10) );

	CPPUNIT_ASSERT( !do_intersect( c0, Cuboid(Point(0,0,11),Point(10,10,21)) ));
	CPPUNIT_ASSERT(  do_intersect( c0, Cuboid(Point(0,0,10),Point(10,10,20)) ));
	CPPUNIT_ASSERT(  do_intersect( c0, Cuboid(Point(3,3,3),Point(5,5,5)) ));
    }



    CPPUNIT_TEST_SUITE( Iso_cuboid_3_Iso_cuboid_3_test );
    CPPUNIT_TEST( test_do_intersect );
    CPPUNIT_TEST_SUITE_END();

};


CPPUNIT_TEST_SUITE_REGISTRATION( Iso_cuboid_3_Iso_cuboid_3_test<R_test> );
