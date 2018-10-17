// -*- C++ -*-

#include "tester.h"
#include <CEP/intersection/Segment_3_Segment_3.h>

using namespace CGAL;
using namespace CEP::intersection;


template <class R>
class Segment_3_Segment_3_test : public CppUnit::TestCase
{
    typedef CGAL::Point_3<R>    Point;
    typedef CGAL::Segment_3<R>  Segment;

    // Why "reimplement" these?  Because the CEP::intersection
    // functions take const reference arguments, while I want to
    // pass things like "s1.opposite()".
    //
    bool do_intersect( Segment s1, Segment s2 )
    {
	return CEP::intersection::do_intersect(s1,s2);
    }
    CGAL::Object intersection( Segment s1, Segment s2 )
    {
	return CEP::intersection::intersection(s1,s2);
    }

public:

    void check( const Segment& s1, const Segment& s2 )
    {
	CPPUNIT_ASSERT( !do_intersect(s1,s2) );
	CPPUNIT_ASSERT( !do_intersect(s1.opposite(),s2) );
	CPPUNIT_ASSERT( !do_intersect(s1,s2.opposite()) );
	CPPUNIT_ASSERT( !do_intersect(s1.opposite(),s2.opposite()) );

	ASSERT_NULL_OBJECT( intersection(s1,s2) );
	ASSERT_NULL_OBJECT( intersection(s1.opposite(),s2) );
	ASSERT_NULL_OBJECT( intersection(s1,s2.opposite()) );
	ASSERT_NULL_OBJECT( intersection(s1.opposite(),s2.opposite()) );
    }

    template <class T>
    void check( const Segment& s1, const Segment& s2, const T& inter )
    {
	CPPUNIT_ASSERT( do_intersect(s1,s2) );
	CPPUNIT_ASSERT( do_intersect(s1.opposite(),s2) );
	CPPUNIT_ASSERT( do_intersect(s1,s2.opposite()) );
	CPPUNIT_ASSERT( do_intersect(s1.opposite(),s2.opposite()) );

	ASSERT_EQUAL_OBJECT( inter, intersection(s1,s2) );
	ASSERT_EQUAL_OBJECT( inter, intersection(s1.opposite(),s2) );
	ASSERT_EQUAL_OBJECT( inter, intersection(s1,s2.opposite()) );
	ASSERT_EQUAL_OBJECT( inter, intersection(s1.opposite(),s2.opposite()) );
    }

    void coll_check( const Segment& s1, const Segment& s2 )
    {
	CPPUNIT_ASSERT( !collinear_do_intersect(s1,s2) );
	ASSERT_NULL_OBJECT( collinear_intersection(s1,s2) );

	check( s1,s2 );
    }

    template <class T>
    void coll_check( const Segment& s1, const Segment& s2, const T& inter )
    {
	CPPUNIT_ASSERT( collinear_do_intersect(s1,s2) );
	ASSERT_EQUAL_OBJECT( inter, collinear_intersection(s1,s2) );

	check( s1,s2, inter );
    }


    void coplanar_check( const Segment& s1, const Segment& s2 )
    {
	CPPUNIT_ASSERT( !coplanar_do_intersect(s1,s2) );
	//ASSERT_NULL_OBJECT( coplanar_intersection(s1,s2) );

	check( s1,s2 );
    }

    template <class T>
    void coplanar_check( const Segment& s1, const Segment& s2, const T& inter )
    {
	CPPUNIT_ASSERT( coplanar_do_intersect(s1,s2) );
	//ASSERT_EQUAL_OBJECT( inter, coplanar_intersection(s1,s2) );

	check( s1,s2, inter );
    }


    void test_intersect_collinear() 
    {
	// Eight points on the line (x,x+1,x-1)
	//   a--b--C--d--e--F--g--h
	Point a( 0, 1,-1);
	Point b(10,11, 9);
	Point c(20,21,19);
	Point d(30,31,29);
	Point e(40,41,39);
	Point f(50,51,49);
	Point g(60,61,59);
	Point h(70,71,69);

	// Do tests with cf as one segment.
	Segment cf(c,f);

	coll_check( cf,Segment(a,b) );
	coll_check( cf,Segment(a,c), c );
	coll_check( cf,Segment(a,d), Segment(c,d) );
	coll_check( cf,Segment(a,f), Segment(c,f) );
	coll_check( cf,Segment(a,g), Segment(c,f) );

	coll_check( cf,Segment(c,d), Segment(c,d) );
	coll_check( cf,Segment(c,f), Segment(c,f) );
	coll_check( cf,Segment(c,g), Segment(c,f) );

	coll_check( cf,Segment(d,e), Segment(d,e) );
	coll_check( cf,Segment(d,f), Segment(d,f) );
	coll_check( cf,Segment(d,g), Segment(d,f) );
	coll_check( cf,Segment(g,h) );

	coll_check( cf,Segment(f,g), f );
    }


    void test_intersect_endpoint() 
    {
	// The line (x,7x+3,5x-1) contains these three points
	Point a(0,3,-1);
	Point b(2,17,9);
	Point c(8,59,39);
	// .. but not this point
	Point d(-8,0,0);

	check( Segment(a,c), Segment(b,d), b );
	check( Segment(b,d), Segment(a,c), b );

	check( Segment(a,b), Segment(c,d) );	
    }


    void test_intersect_coplanar()
    {
	// The following points all lie on the plane (x,y,7x+13y-3)
	Point a(0,0,-3);
	Point b(1,1,17);
	Point c(0,1,10);
	Point d(1,0,4);

	coplanar_check( Segment(a,b), Segment(c,d), Point(1,1,14,2) );
	coplanar_check( Segment(a,c), Segment(b,d) );
	coplanar_check( Segment(a,d), Segment(b,c) );

	coplanar_check( Segment(a,b), Segment(a,b), Segment(b,a) );
	coplanar_check( Segment(a,b), Segment(b,a), Segment(b,a) );
    }
    
    
    CPPUNIT_TEST_SUITE( Segment_3_Segment_3_test );
    CPPUNIT_TEST( test_intersect_collinear );
    CPPUNIT_TEST( test_intersect_endpoint );
    CPPUNIT_TEST( test_intersect_coplanar );
    CPPUNIT_TEST_SUITE_END();

};


CPPUNIT_TEST_SUITE_REGISTRATION( Segment_3_Segment_3_test<R_test> );
