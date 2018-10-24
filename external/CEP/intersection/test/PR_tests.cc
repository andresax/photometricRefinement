#include <CGAL/Cartesian.h>
#include <CGAL/Homogeneous.h>

#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Object.h> 
#include <CEP/intersection/Plane_3_Triangle_3.h>
#include <CEP/intersection/Segment_3_Triangle_3.h>

#include <CGAL/MP_Float.h> 
#include <CGAL/Quotient.h>


#include "tester.h"
#include <CEP/intersection/Segment_3_Triangle_3.h>
#include <CEP/intersection/Segment_3_Segment_3.h>




using namespace CGAL;
using namespace CEP::intersection;


// Test examples from problem reports.

class PR_test : public CppUnit::TestCase
{
    typedef CGAL::MP_Float coord_type;
    //typedef double coord_type;
    //typedef CGAL::Cartesian<coord_type>  K;
    typedef CGAL::Homogeneous<coord_type>  K;

    typedef K::Point_3  Point;
    typedef K::Segment_3  Segment;
    typedef K::Triangle_3  Triangle;


public:

    void PR_02()
    {
	/* Example from Shobha Potluri, 2002-11-03.
	 * Segment "l" intersects the plane of triangle t (z=0 plane)
	 * at coordinate (5/12,1/3,0), which is inside the triangle.
	 *
	 * Works with homogeneous representation, both using MP_Float
	 * and using double.  With cartesian, intersection(l,t)
	 * returns the empty object.
	 */
	Point p1(0,1,1,3);
	Point p2(5,1,-3,3);
	Segment l(p1,p2);

	Point a(0,0,0);
	Point b(1,0,0);
	Point c(1,1,0);
	Triangle t(a,b,c);

	/*
	std::cout << std::endl
		  << "l = " << l << std::endl
		  << "t = " << t << std::endl
		  << "intersection should be: "
		  << p1 + (p2-p1)/coord_type(4) << std::endl;
	*/

	K::Plane_3 t_plane = t.supporting_plane();

	CPPUNIT_ASSERT( do_intersect(l,t) );
	CPPUNIT_ASSERT( do_intersect(l,t_plane) );

	CGAL::Object obj = intersection(l,t_plane);
	CPPUNIT_ASSERT( !obj.is_empty() );

	K::Point_3 p_int;
	CPPUNIT_ASSERT( CGAL::assign(p_int,obj) );

	obj = intersection(l,t);
	CPPUNIT_ASSERT( !obj.is_empty() );

	ASSERT_EQUAL_OBJECT( p_int, obj );

	/*
	CGAL::assign(p_int,obj);
	std::cout << p_int << std::endl;
	*/
    }

    CPPUNIT_TEST_SUITE( PR_test );
    CPPUNIT_TEST( PR_02 );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( PR_test );

