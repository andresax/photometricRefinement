#include "tester.h"
#include <CEP/intersection/Line_3_Line_3.h>


using namespace CGAL;
using namespace CEP::intersection::Line_3_Line_3;


template <class R>
class Line_3_Line_3_test : public CppUnit::TestCase
{
    typedef CGAL::Point_3<R>    Point;

public:

    void t_parallel()
    {
	// Two points on line (x,x,0)
	Point p(0,0,0);
	Point q(1,1,0);

	// Two points on line (x,x+1,0)
	Point r(300,301,0);
	Point s(-77,-76,0);

	ASSERT_NULL_OBJECT( coplanar_intersection(p,q,r,s) );
	ASSERT_NULL_OBJECT( coplanar_intersection(p,q,s,r) );
	ASSERT_NULL_OBJECT( coplanar_intersection(q,p,r,s) );
	ASSERT_NULL_OBJECT( coplanar_intersection(q,p,s,r) );
    }

    void t_nonparallel()
    {
	// Two points on line (0,y,3y-3)
	Point p(0,5,12);
	Point q(0,-2,-9);

	// Two points on line (0,y,-2y+1)
	Point r(0,0,1);
	Point s(0,400,-799);

	// Intersection: 3y-3 = -2y+1 ==> 5y = 4
	Point i_point(0,4,-3,5);

	ASSERT_EQUAL_OBJECT( i_point, coplanar_intersection(p,q,r,s) );
	ASSERT_EQUAL_OBJECT( i_point, coplanar_intersection(p,q,s,r) );
	ASSERT_EQUAL_OBJECT( i_point, coplanar_intersection(q,p,r,s) );
	ASSERT_EQUAL_OBJECT( i_point, coplanar_intersection(q,p,s,r) );

	ASSERT_EQUAL_OBJECT( i_point, coplanar_nonparallel_intersection(p,q,r,s) );
	ASSERT_EQUAL_OBJECT( i_point, coplanar_nonparallel_intersection(p,q,s,r) );
	ASSERT_EQUAL_OBJECT( i_point, coplanar_nonparallel_intersection(q,p,r,s) );
	ASSERT_EQUAL_OBJECT( i_point, coplanar_nonparallel_intersection(q,p,s,r) );

    }

    CPPUNIT_TEST_SUITE( Line_3_Line_3_test );
    CPPUNIT_TEST( t_parallel );
    CPPUNIT_TEST( t_nonparallel );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Line_3_Line_3_test<R_test> );
