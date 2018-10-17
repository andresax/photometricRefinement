#include "tester.h"
#include <CEP/intersection/Segment_3_Triangle_3.h>
#include <CGAL/Vector_3.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Segment_3_Triangle_3_test : public CppUnit::TestCase
{
    typedef typename R::FT          Field_type;

    typedef CGAL::Object            Object;
    typedef CGAL::Point_3<R>        Point;
    typedef CGAL::Segment_3<R>      Segment;
    typedef CGAL::Vector_3<R>       Vector;
    typedef CGAL::Triangle_3<R>     Triangle;
    typedef CGAL::Plane_3<R>        Plane;
    

public:

    // Points of the triangle
    Point a,b,c;

    // coplanar points INSIDE triangle
    Point p,q;

    // coplanar points OUTSIDE triangle
    Point r,s;

    // Segment v,w intersects plane at point p
    // v above plane, w below
    Point v,w;

    // Segment x,y intersects plane at point r
    // x above plane, y below
    Point x,y;



    void setUp()
    {
	a = Point(0,0,0);
	b = Point(50,0,0);
	c = Point(13,500,0);
	Triangle T(a,b,c);
	Plane P = T.supporting_plane();

	p = Point(20,30,0);
	q = Point(40,3,0);
	CGAL_assertion( T.has_on(p) );
	CGAL_assertion( T.has_on(q) );

	r = Point(-99,-5,0);
	s = Point(400,-33,0);
	CGAL_assertion( P.has_on(r) );
	CGAL_assertion( P.has_on(s) );

	Vector d( 30,-3,1 );
	v = p + d * Field_type(13);
	w = p - d * Field_type(5);

	x = r + d * Field_type(79);
	y = r - d * Field_type(17);
    }



    void test_do_intersect( const Segment& s, bool expected )
    {
	CPPUNIT_ASSERT_EQUAL( expected, do_intersect(s, Triangle(a,b,c)) );
	CPPUNIT_ASSERT_EQUAL( expected, do_intersect(s, Triangle(a,c,b)) );
	CPPUNIT_ASSERT_EQUAL( expected, do_intersect(s, Triangle(b,a,c)) );
	CPPUNIT_ASSERT_EQUAL( expected, do_intersect(s, Triangle(b,c,a)) );
	CPPUNIT_ASSERT_EQUAL( expected, do_intersect(s, Triangle(c,a,b)) );
	CPPUNIT_ASSERT_EQUAL( expected, do_intersect(s, Triangle(c,b,a)) );
    }


    void test_coplanar_do_intersect( const Segment& s, bool expected )
    {
	CPPUNIT_ASSERT_EQUAL( expected, coplanar_do_intersect(s, Triangle(a,b,c)) );
	CPPUNIT_ASSERT_EQUAL( expected, coplanar_do_intersect(s, Triangle(a,c,b)) );
	CPPUNIT_ASSERT_EQUAL( expected, coplanar_do_intersect(s, Triangle(b,a,c)) );
	CPPUNIT_ASSERT_EQUAL( expected, coplanar_do_intersect(s, Triangle(b,c,a)) );
	CPPUNIT_ASSERT_EQUAL( expected, coplanar_do_intersect(s, Triangle(c,a,b)) );
	CPPUNIT_ASSERT_EQUAL( expected, coplanar_do_intersect(s, Triangle(c,b,a)) );

	test_do_intersect( s, expected );
    }



    // Test segment against all six possible triangles on the three points
    // This version CPPUNIT_ASSERTs no intersection.
    void test_intersection( const Segment& s )
    {
	test_do_intersect( s, false );

	CGAL::Object i;

	i = intersection(s, Triangle(a,b,c));
	ASSERT_NULL_OBJECT(i);
	i = intersection(s, Triangle(a,c,b));
	ASSERT_NULL_OBJECT(i);
	i = intersection(s, Triangle(b,a,c));
	ASSERT_NULL_OBJECT(i);
	i = intersection(s, Triangle(b,c,a));
	ASSERT_NULL_OBJECT(i);
	i = intersection(s, Triangle(c,a,b));
	ASSERT_NULL_OBJECT(i);
	i = intersection(s, Triangle(c,b,a));
	ASSERT_NULL_OBJECT(i);
    }


    // Test plane against all six possible triangles on the three points
    // This version CPPUNIT_ASSERTs no intersection.
    void test_coplanar_intersection( const Segment& s )
    {
	test_coplanar_do_intersect( s, false );

	CGAL::Object i;

	i = coplanar_intersection(s, Triangle(a,b,c));
	ASSERT_NULL_OBJECT(i);
	i = coplanar_intersection(s, Triangle(a,c,b));
	ASSERT_NULL_OBJECT(i);
	i = coplanar_intersection(s, Triangle(b,a,c));
	ASSERT_NULL_OBJECT(i);
	i = coplanar_intersection(s, Triangle(b,c,a));
	ASSERT_NULL_OBJECT(i);
	i = coplanar_intersection(s, Triangle(c,a,b));
	ASSERT_NULL_OBJECT(i);
	i = coplanar_intersection(s, Triangle(c,b,a));
	ASSERT_NULL_OBJECT(i);
    }


    // Test plane against all six possible triangles on the three points
    // This version CPPUNIT_ASSERTs nonempty intersection.
    template <class T>
    void test_intersection( const Segment& s, 
			    const T& expected )
    {
	test_do_intersect( s, true );

	CGAL::Object i;

	i = intersection(s, Triangle(a,b,c));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = intersection(s, Triangle(a,c,b));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = intersection(s, Triangle(b,a,c));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = intersection(s, Triangle(b,c,a));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = intersection(s, Triangle(c,a,b));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = intersection(s, Triangle(c,b,a));
	ASSERT_EQUAL_OBJECT( expected, i );
    }


    // Test plane against all six possible triangles on the three points
    // This version CPPUNIT_ASSERTs nonempty intersection.
    template <class T>
    void test_coplanar_intersection( const Segment& s, 
				     const T& expected )
    {
	test_coplanar_do_intersect( s, true );

	CGAL::Object i;

	i = coplanar_intersection(s, Triangle(a,b,c));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = coplanar_intersection(s, Triangle(a,c,b));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = coplanar_intersection(s, Triangle(b,a,c));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = coplanar_intersection(s, Triangle(b,c,a));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = coplanar_intersection(s, Triangle(c,a,b));
	ASSERT_EQUAL_OBJECT( expected, i );
	i = coplanar_intersection(s, Triangle(c,b,a));
	ASSERT_EQUAL_OBJECT( expected, i );
    }


    void t_outside()
    {
	// Test with segment endpoints outside the triangle.
	// Points r,s are on the triangle plane; the others are not.
	test_coplanar_intersection( Segment(r,s) );

	test_intersection( Segment(r,w) );
	test_intersection( Segment(s,y) );
	test_intersection( Segment(v,x) );
	test_intersection( Segment(w,y) );
	test_intersection( Segment(x,y) );
    }

    void t_coplanar_overlap()
    {
	test_coplanar_intersection( Segment(p,q), Segment(q,p) );
	test_coplanar_intersection( Segment(p,a), Segment(p,a) );
	test_coplanar_intersection( Segment(r,a), a );

	Point m = midpoint(a,b);
	test_coplanar_intersection( Segment(p,m), Segment(m,p) );
	test_coplanar_intersection( Segment(r,m), m );
    }

    void t_skew_overlap()
    {
	test_intersection( Segment(v,w), p );
	test_intersection( Segment(x,y) );
    }


    CPPUNIT_TEST_SUITE( Segment_3_Triangle_3_test );
    CPPUNIT_TEST( t_outside );
    CPPUNIT_TEST( t_coplanar_overlap );
    CPPUNIT_TEST( t_skew_overlap );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Segment_3_Triangle_3_test<R_test> );

