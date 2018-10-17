#include "tester.h"
#include <CEP/intersection/Triangle_3_Plane_3.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Triangle_3_Plane_3_test : public CppUnit::TestCase
{
    typedef CGAL::Object            Object;
    typedef CGAL::Point_3<R>        Point;
    typedef CGAL::Direction_3<R>    Direction;
    typedef CGAL::Segment_3<R>      Segment;
    typedef CGAL::Triangle_3<R>     Triangle;
    typedef CGAL::Plane_3<R>        Plane;

public:

    Plane P;
    Point a,b,c;

    void setUp()
    {
	P = Plane( Point(1,2,5), Direction(0,0,1) );
    }


    // Test plane against all six possible triangles on the three points
    // This version CPPUNIT_ASSERTs no intersection.
    void test_all_triangle( const Point& a, const Point& b, const Point& c )
    {
	CGAL::Object i;

	i = CEP::intersection::intersection(Triangle(a,b,c),P);
	CPPUNIT_ASSERT( i.is_empty() );
	i = CEP::intersection::intersection(Triangle(a,c,b),P);
	CPPUNIT_ASSERT( i.is_empty() );
	i = CEP::intersection::intersection(Triangle(b,a,c),P);
	CPPUNIT_ASSERT( i.is_empty() );
	i = CEP::intersection::intersection(Triangle(b,c,a),P);
	CPPUNIT_ASSERT( i.is_empty() );
	i = CEP::intersection::intersection(Triangle(c,a,b),P);
	CPPUNIT_ASSERT( i.is_empty() );
	i = CEP::intersection::intersection(Triangle(c,b,a),P);
	CPPUNIT_ASSERT( i.is_empty() );
    }


    // Test plane against all six possible triangles on the three points
    // This version CPPUNIT_ASSERTs nonempty intersection.
    template <class T>
    void test_all_triangle( const Point& a, const Point& b, const Point& c,
			    const T& expected )
    {
	CGAL::Object i;

	i = CEP::intersection::intersection(Triangle(a,b,c),P);
	ASSERT_EQUAL_OBJECT( expected, i );
	i = CEP::intersection::intersection(Triangle(a,c,b),P);
	ASSERT_EQUAL_OBJECT( expected, i );
	i = CEP::intersection::intersection(Triangle(b,a,c),P);
	ASSERT_EQUAL_OBJECT( expected, i );
	i = CEP::intersection::intersection(Triangle(b,c,a),P);
	ASSERT_EQUAL_OBJECT( expected, i );
	i = CEP::intersection::intersection(Triangle(c,a,b),P);
	ASSERT_EQUAL_OBJECT( expected, i );
	i = CEP::intersection::intersection(Triangle(c,b,a),P);
	ASSERT_EQUAL_OBJECT( expected, i );
    }


    void empty_intersection()
    {
	a = Point(-5,7,-5); b = Point(10,30,4); c = Point(-5,7,-53);
	test_all_triangle( a,b,c );
    }

    void point_intersection()
    {
	a = Point(-5,7,5); b = Point(10,30,4); c = Point(-5,7,-5);
	test_all_triangle( a,b,c, a );
    }

    void segment_intersection()
    {
	a = Point(8,7,5); b = Point(-4,-66,5); c = Point(500,23,18);
	test_all_triangle( a,b,c, Segment(a,b) );

	a = Point(15,-7,6); b = Point(25,-7,7); c = Point(19,-7,4);
	test_all_triangle( a,b,c, Segment(Point(17,-7,5),Point(21,-7,5)) );
    }


    CPPUNIT_TEST_SUITE( Triangle_3_Plane_3_test );
    CPPUNIT_TEST( empty_intersection );
    CPPUNIT_TEST( point_intersection );
    CPPUNIT_TEST( segment_intersection );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Triangle_3_Plane_3_test<R_test> );

