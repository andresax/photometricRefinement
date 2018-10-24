#include <list>

#include "tester.h"
#include <CEP/intersection/Polygon_2_Line_2.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Polygon_2_Line_2_test : public CppUnit::TestCase
{
    typedef CGAL::Point_2<R>        Point;
    typedef CGAL::Line_2<R>         Line;
    typedef CGAL::Triangle_2<R>     Triangle;

    typedef std::list<Point>        Point_list;

public:

    Point_list polygon;
    Point_list left;
    Point_list right;

    void setUp()
    {
	polygon.push_back( Point(0,0) );
	polygon.push_back( Point(1,0) );
	polygon.push_back( Point(1,1) );
	polygon.push_back( Point(0,1) );
    }

    void tearDown()
    {
	polygon.clear();
	left.clear();
	right.clear();
    }


    void no_cut_left()
    {
	Line l( Point(10,0), Point(10,1) );
	CEP::intersection::slice( polygon.begin(), polygon.end(), l,
				  std::back_inserter(left),
				  std::back_inserter(right) );
	CPPUNIT_ASSERT_EQUAL( polygon, left );
	CPPUNIT_ASSERT( right.empty() );
    }

    void no_cut_right()
    {
	Line l( Point(0,10), Point(1,10) );
	CEP::intersection::slice( polygon.begin(), polygon.end(), l,
				  std::back_inserter(left),
				  std::back_inserter(right) );
	CPPUNIT_ASSERT( left.empty() );
	CPPUNIT_ASSERT_EQUAL( polygon, right );
    }

    void cut_in_two()
    {
	Line l( Point(-100,-100), Point(0,0) );
	CEP::intersection::slice( polygon.begin(), polygon.end(), l,
				  std::back_inserter(left),
				  std::back_inserter(right) );

	// Square is split on diagonal into two triangles
	Point_list e_left;
	e_left.push_back( Point(0,0) );
	e_left.push_back( Point(1,1) );
	e_left.push_back( Point(0,1) );
	
	Point_list e_right;
	e_right.push_back( Point(0,0) );
	e_right.push_back( Point(1,0) );
	e_right.push_back( Point(1,1) );
	
	CPPUNIT_ASSERT_EQUAL( e_left, left );
	CPPUNIT_ASSERT_EQUAL( e_right, right );
    }


    void cut_bottom_edge()
    {
	Line l( Point(-100,0), Point(0,0) );
	CEP::intersection::slice( polygon.begin(), polygon.end(), l,
				  std::back_inserter(left),
				  std::back_inserter(right) );

	// Square is cut along bottom edge
	Point_list e_right;
	e_right.push_back( Point(0,0) );
	e_right.push_back( Point(1,0) );
	
	CPPUNIT_ASSERT_EQUAL( polygon, left );
	CPPUNIT_ASSERT_EQUAL( e_right, right );
    }


    CPPUNIT_TEST_SUITE( Polygon_2_Line_2_test );
    CPPUNIT_TEST( no_cut_left );
    CPPUNIT_TEST( no_cut_right );
    CPPUNIT_TEST( cut_in_two );
    CPPUNIT_TEST( cut_bottom_edge );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Polygon_2_Line_2_test<R_test> );

