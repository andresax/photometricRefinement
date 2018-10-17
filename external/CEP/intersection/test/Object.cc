#include "tester.h"
#include <CEP/intersection/Object.h>
#include <CGAL/predicates_on_points_3.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Object_test : public CppUnit::TestCase
{
    typedef CGAL::Point_3<R>    Point;
    typedef CGAL::Segment_3<R>  Segment;
    typedef CGAL::Triangle_3<R> Triangle;

public:

    // Four coplanar points:
    //
    //    a-----b-----c
    //         /
    //        d

    Point a,b,c,d;

    void setUp()
    {
	a = Point(5, 8,0);
	b = Point(6,10,1);
	c = Point(7,12,2);
	d = Point(33,55,4);

	CGAL_assertion( CGAL::collinear(a,b,c) );
	CGAL_assertion( !CGAL::collinear(a,b,d) );
    }


    void t_point()
    {
	Segment s(a,a);
	Triangle t(a,a,a);

	ASSERT_EQUAL_OBJECT( a, make_object_nondegenerate(s) );
	ASSERT_EQUAL_OBJECT( a, make_object_nondegenerate(t) );

	ASSERT_EQUAL_OBJECT( a, make_object_nondegenerate<R>(make_object(s)) );
	ASSERT_EQUAL_OBJECT( a, make_object_nondegenerate<R>(make_object(t)) );
    }


    void t_segment()
    {
	Segment s(a,c);

	ASSERT_EQUAL_OBJECT( s, make_object_nondegenerate(s) );
	ASSERT_EQUAL_OBJECT( s, make_object_nondegenerate<R>(make_object(s)) );

	Triangle t(a,b,c);
	ASSERT_EQUAL_OBJECT( s, make_object_nondegenerate(t) );
	ASSERT_EQUAL_OBJECT( s, make_object_nondegenerate<R>(make_object(t)) );

	t = Triangle(a,c,c);
	ASSERT_EQUAL_OBJECT( s, make_object_nondegenerate(t) );
	ASSERT_EQUAL_OBJECT( s, make_object_nondegenerate<R>(make_object(t)) );
    }


    void t_triangle()
    {
	Triangle t(a,b,d);

	ASSERT_EQUAL_OBJECT( t, make_object_nondegenerate(t) );
	ASSERT_EQUAL_OBJECT( t, make_object_nondegenerate<R>(make_object(t)) );
    }


    CPPUNIT_TEST_SUITE( Object_test );
    CPPUNIT_TEST( t_point );
    CPPUNIT_TEST( t_segment );
    CPPUNIT_TEST( t_triangle );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Object_test<R_test> );
