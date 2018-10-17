#include "tester.h"
#include <CEP/intersection/Triangle_3_Triangle_3.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Triangle_3_Triangle_3_test : public CppUnit::TestCase
{
    typedef CGAL::Object            Object;
    typedef CGAL::Point_3<R>        Point;
    typedef CGAL::Direction_3<R>    Direction;
    typedef CGAL::Segment_3<R>      Segment;
    typedef CGAL::Triangle_3<R>     Triangle;
    typedef CGAL::Plane_3<R>        Plane;

public:

    // No intersection
    void test( const Point& u0, const Point& u1, const Point& u2,
	       const Point& v0, const Point& v1, const Point& v2 )
    {
	Triangle T1 = Triangle(u0,u1,u2);
	Triangle T2 = Triangle(v0,v1,v2);
	
	CPPUNIT_ASSERT( ! CEP::intersection::do_intersect(T1,T2) );
	ASSERT_NULL_OBJECT( CEP::intersection::intersection(T1,T2) );
    }

    template <class T>
    void test( const Point& u0, const Point& u1, const Point& u2,
	       const Point& v0, const Point& v1, const Point& v2,
	       const T& result )
    {
	Triangle T1 = Triangle(u0,u1,u2);
	Triangle T2 = Triangle(v0,v1,v2);

	CPPUNIT_ASSERT( CEP::intersection::do_intersect(T1,T2) );
	CGAL::Object obj_i = CEP::intersection::intersection(T1,T2);
	ASSERT_EQUAL_OBJECT( result, obj_i );

	Point p;
	Segment s;
	if ( assign(p,obj_i) ) {
	    CPPUNIT_ASSERT( T1.has_on(p) );
	    CPPUNIT_ASSERT( T2.has_on(p) );
	} else if ( assign(s,obj_i) ) {
	    CPPUNIT_ASSERT( T1.has_on(s.source()) );
	    CPPUNIT_ASSERT( T1.has_on(s.target()) );
	    CPPUNIT_ASSERT( T2.has_on(s.source()) );
	    CPPUNIT_ASSERT( T2.has_on(s.target()) );
	}
    }

    // Runs all six permutations of the first triangle points
    void run_tests( const Point& u0, const Point& u1, const Point& u2,
		    const Point& v0, const Point& v1, const Point& v2 )
    {
	test( u0, u1, u2, v0, v1, v2 );
	test( u0, u2, u1, v0, v1, v2 );
	test( u1, u0, u2, v0, v1, v2 );
	test( u1, u2, u0, v0, v1, v2 );
	test( u2, u0, u1, v0, v1, v2 );
	test( u2, u1, u0, v0, v1, v2 );
    }

    // Runs all six permutations of the first triangle points
    template <class T>
    void run_tests( const Point& u0, const Point& u1, const Point& u2,
		    const Point& v0, const Point& v1, const Point& v2,
		    const T& result )
    {
	test( u0, u1, u2, v0, v1, v2, result );
	test( u0, u2, u1, v0, v1, v2, result );
	test( u1, u0, u2, v0, v1, v2, result );
	test( u1, u2, u0, v0, v1, v2, result );
	test( u2, u0, u1, v0, v1, v2, result );
	test( u2, u1, u0, v0, v1, v2, result );
    }


    // parallel and well-separated
    void t_01()
    {
	run_tests( Point(0,0,0), Point(55,22,0), Point(-123,23,0),
		   Point(0,0,10), Point(55,22,10), Point(-123,23,10) );
    }

    // intersect at common vertex
    void t_02()
    {
	run_tests( Point(0,0,0), Point(55,22,0), Point(-123,23,0),
		   Point(0,0,10), Point(55,22,10), Point(-123,23,0),
		   Point(-123,23,0) );
    }

    // intersect vertex and relative interior of edge
    void t_03()
    {
	run_tests( Point(0,0,0), Point(55,22,0), Point(-123,23,0),
		   Point(0,30,10), Point(5,2,0), Point(-123,23,2210),
		   Point(5,2,0) );
    }

    // vertex and interior of triangle
    void t_04()
    {
	run_tests( Point(0,0,0), Point(55,22,0), Point(-123,23,0),
		   Point(0,10,0), Point(55,22,10), Point(-123,23,10),
		   Point(0,10,0) );
    }

    // faces properly intersect
    void t_05()
    {
	// TODO: rotate this example into yz and xz planes
	run_tests( Point(-2,-2,0), Point(2,-2,0), Point(0,3,0),
		   Point(0,0,10), Point(0,0,-10), Point(0,-1323,34),
		   Segment(Point(0,0,0),Point(0,-2,0)) );
    }

    // coplanar: no intersection
    void t_06()
    {
	run_tests( Point(-2,-2,0), Point(2,-2,0), Point(0,3,0),
		   Point(20,20,0), Point(20,22,0), Point(22,20,0) );

    }

    // coplanar: intersect vertex and edge interior
    void t_07()
    {
	run_tests( Point(-2,-2,0), Point(2,-2,0), Point(0,3,0),
		   Point(0,-2,0), Point(20,-22,0), Point(-22,-20,0),
		   Point(0,-2,0) );
    }

    // coplanar: one includes the other
    void t_08()
    {
	run_tests( Point(-2,-2,0), Point(2,-2,0), Point(0,3,0),
		   Point(-3,-3,0), Point(3,-3,0), Point(0,4,0),
		   Triangle(Point(-2,-2,0), Point(2,-2,0), Point(0,3,0)) );
    }

    // Example from Martin Bernreuther, 2001-07-05
    void t_09()
    {
	run_tests( Point(1,0,0), Point(0,1,0), Point(0,0,1),
		   Point(1,-1,1,2), Point(-1,1,1,2), Point(1,1,1,2),
		   Segment(Point(1,0,1,2),Point(0,1,1,2)) );
    }


    CPPUNIT_TEST_SUITE( Triangle_3_Triangle_3_test );
    CPPUNIT_TEST( t_01 );
    CPPUNIT_TEST( t_02 );
    CPPUNIT_TEST( t_03 );
    CPPUNIT_TEST( t_04 );
    CPPUNIT_TEST( t_05 );
    CPPUNIT_TEST( t_06 );
    CPPUNIT_TEST( t_07 );
    CPPUNIT_TEST( t_08 );
    CPPUNIT_TEST( t_09 );
    CPPUNIT_TEST_SUITE_END();

};


CPPUNIT_TEST_SUITE_REGISTRATION( Triangle_3_Triangle_3_test<R_test> );

