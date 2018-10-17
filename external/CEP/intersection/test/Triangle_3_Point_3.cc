#include "tester.h"
#include <CEP/intersection/Triangle_3_Point_3.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Triangle_3_Point_3_test : public CppUnit::TestCase
{
    typedef CGAL::Point_3<R>        Point;
    typedef CGAL::Triangle_3<R>     Triangle;

public:

    // Test triangle.  Lies on plane (x,y,-3x+8y+14).
    Point a,b,c;


    int planez(int x, int y) { return -3*x+8*y+14; }


    void setUp()
    {
	a = Point(6,-2,planez(6,-2));
	b = Point(9,2,planez(9,2));
	c = Point(-77,22,planez(-77,22));
    }


    // Test point p against all six possible triangles on the three points
    // (a,b,c).
    void test( const Point& p, bool expect_intersect )
    {
	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      do_intersect(Triangle(a,b,c),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      do_intersect(Triangle(a,c,b),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      do_intersect(Triangle(b,a,c),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      do_intersect(Triangle(b,c,a),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      do_intersect(Triangle(c,a,b),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      do_intersect(Triangle(c,b,a),p) );

    }

    // Test point p against all six possible triangles on the three points
    // (a,b,c).  Use coplanar_do_intersect.
    void coplanar_test( const Point& p, bool expect_intersect )
    {
	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      coplanar_do_intersect(Triangle(a,b,c),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      coplanar_do_intersect(Triangle(a,c,b),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      coplanar_do_intersect(Triangle(b,a,c),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      coplanar_do_intersect(Triangle(b,c,a),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      coplanar_do_intersect(Triangle(c,a,b),p) );

	CPPUNIT_ASSERT_EQUAL( expect_intersect,
			      coplanar_do_intersect(Triangle(c,b,a),p) );

	test( p,expect_intersect );
    }


    void noncoplanar()
    {
	test( Point(5,0,planez(5,0)+2), false );
    }

    void coplanar_inside()
    {
	coplanar_test( Point(5,0,planez(5,0)), true );
    }

    void coplanar_on()
    {
	coplanar_test( a, true );
	coplanar_test( midpoint(a,b), true );
    }

    void coplanar_outside()
    {
	coplanar_test( Point(10,-4,planez(10,-4)), false );
    }



    CPPUNIT_TEST_SUITE( Triangle_3_Point_3_test );
    CPPUNIT_TEST( noncoplanar );
    CPPUNIT_TEST( coplanar_inside );
    CPPUNIT_TEST( coplanar_on );
    CPPUNIT_TEST( coplanar_outside );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Triangle_3_Point_3_test<R_test> );

