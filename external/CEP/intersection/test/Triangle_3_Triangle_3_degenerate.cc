#include "tester.h"
#include <CEP/intersection/Triangle_3_Triangle_3.h>
#include <CEP/intersection/Object.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R_>
class Triangle_3_Triangle_3_degenerate_test : public CppUnit::TestCase
{
    typedef CGAL::Object             Object;
    typedef CGAL::Point_3<R_>        Point;
    typedef CGAL::Direction_3<R_>    Direction;
    typedef CGAL::Segment_3<R_>      Segment;
    typedef CGAL::Triangle_3<R_>     Triangle;
    typedef CGAL::Plane_3<R_>        Plane;


    void does_contain_point( const Triangle& T, const Point& p )
    {
	Object oT = make_object_nondegenerate( T );
	Triangle tT;
	Segment sT;
	Point pT;

	if ( assign(tT,oT) ) {
	    CPPUNIT_ASSERT( tT.has_on(p) );
	} else if ( assign(sT,oT) ) {
	    CPPUNIT_ASSERT( sT.has_on(p) );
	} else {
	    CPPUNIT_ASSERT( assign(pT,oT) );
	    CPPUNIT_ASSERT_EQUAL( pT, p );
	}
    }


    // No intersection
    void test( const Point& u0, const Point& u1, const Point& u2,
	       const Point& v0, const Point& v1, const Point& v2 )
    {
	Triangle T1 = Triangle(u0,u1,u2);
	Triangle T2 = Triangle(v0,v1,v2);
	
	//CPPUNIT_ASSERT( ! CEP::intersection::do_intersect(T1,T2) );
	ASSERT_NULL_OBJECT( CEP::intersection::intersection(T1,T2) );
    }


    template <class T>
    void test( const Point& u0, const Point& u1, const Point& u2,
	       const Point& v0, const Point& v1, const Point& v2,
	       const T& result )
    {
	Triangle T1 = Triangle(u0,u1,u2);
	Triangle T2 = Triangle(v0,v1,v2);

	//CPPUNIT_ASSERT( CEP::intersection::do_intersect(T1,T2) );
	CGAL::Object obj_i = CEP::intersection::intersection(T1,T2);
	ASSERT_EQUAL_OBJECT( result, obj_i );

	Point p;
	Segment s;
	if ( assign(p,obj_i) ) {
	    does_contain_point( T1, p );
	    does_contain_point( T2, p );
	} else if ( assign(s,obj_i) ) {
	    does_contain_point( T1, s.source() );
	    does_contain_point( T1, s.target() );
	    does_contain_point( T2, s.source() );
	    does_contain_point( T2, s.target() );
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


    /* Several points in common plane
                         x
                          \
           a---b---c---d---e---f
                            \
                             y  
    */

    Point a,b,c,d,e,f,x,y;

public:

    void setUp()
    {
	a = Point( -4,0,0 );
	b = Point( -3,0,0 );
	c = Point( -2,0,0 );
	d = Point( -1,0,0 );
	e = Point(  0,0,0 );
	f = Point(  1,0,0 );

	x = Point( -2,2,0 );
	y = Point( 1,-1,0 );
    }


    // The first few are not degenerate, in fact.
    void t_01()
    {
	run_tests( c,x,y, a,c,y, Segment(c,y) );
	run_tests( c,x,y, a,f,x, Triangle(c,e,x) );
    }


    // Check that the degenerate "triangles" used in triangle_vs_segment()
    // are really considered degenerate.
    void check_degenerate()
    {
	Triangle T;

	T = Triangle(a,a,a);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(a,b,b);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(a,b,c);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(a,b,d);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(a,b,e);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(a,b,f);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(c,c,c);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(c,d,d);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(c,d,e);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(c,d,f);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(d,d,d);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(d,e,e);
	CPPUNIT_ASSERT( T.is_degenerate() );

	T = Triangle(d,e,f);
	CPPUNIT_ASSERT( T.is_degenerate() );
    }


    // Triangle v.s. line segment
    // Tests are valid for triangles (c,x,y), (c,e,x), and (c,e,y)
    void triangle_vs_segment( const Point& u, const Point& v, const Point& w )
    {
	run_tests( u,v,w, a,a,a );

	run_tests( u,v,w, a,b,b );
	run_tests( u,v,w, a,b,c, c );
	run_tests( u,v,w, a,b,d, Segment(c,d) );
	run_tests( u,v,w, a,b,e, Segment(c,e) );
	run_tests( u,v,w, a,b,f, Segment(c,e) );

	run_tests( u,v,w, c,c,c, c );
	run_tests( u,v,w, c,d,d, Segment(c,d) );
	run_tests( u,v,w, c,d,e, Segment(c,e) );
	run_tests( u,v,w, c,d,f, Segment(c,e) );

	run_tests( u,v,w, d,d,d, d );

	run_tests( u,v,w, d,e,e, Segment(d,e) );
	run_tests( u,v,w, d,e,f, Segment(d,e) );
    }

    // Triangle c,x,y v.s. line segment
    void t_02()
    {
	triangle_vs_segment(c,x,y);
    }

    // Triangle c,e,x v.s. line segment
    void t_03()
    {
	triangle_vs_segment(c,e,x);
    }

    // Triangle c,e,y v.s. line segment
    void t_04()
    {
	triangle_vs_segment(c,e,y);
    }


    // Both triangles degenerate to segments
    void t_05()
    {
	run_tests( a,b,f, c,x,x, c );
	run_tests( a,b,f, x,y,e, e );
    }

    CPPUNIT_TEST_SUITE( Triangle_3_Triangle_3_degenerate_test );
    CPPUNIT_TEST( t_01 );
    CPPUNIT_TEST( check_degenerate );
    CPPUNIT_TEST( t_02 );
    CPPUNIT_TEST( t_03 );
    CPPUNIT_TEST( t_04 );
    CPPUNIT_TEST( t_05 );
    CPPUNIT_TEST_SUITE_END();

};


CPPUNIT_TEST_SUITE_REGISTRATION( Triangle_3_Triangle_3_degenerate_test<R_test> );

