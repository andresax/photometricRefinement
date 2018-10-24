#include "tester.h"
#include <CEP/intersection/Halfplane_3.h>


using namespace CGAL;
using namespace CEP::intersection;



template <class R>
class Halfplane_3_test : public CppUnit::TestCase
{
    typedef CGAL::Point_3<R>    Point;
    typedef CGAL::Segment_3<R>  Segment;

public:

    // (p,q,r) describes the halfplane z=0, y>= x.
    // points a,b are inside halfplane
    // points x,y are outside
    // l is on the boundary
    // Points b,l,x on line (x+2y-15 = 0)
    Point a,b, l, p,q,r, x,y;


    void setUp()
    {
	p = Point(0,0,0);
	q = Point(1,1,0);
	r = Point(3,500,0);

	a = Point(5,8,0);
	b = Point(3,6,0);

	l = Point(5,5,0);

	x = Point(7,4,0);
	y = Point(22,20,0);
    }


    void t_both_on()
    {
	Segment ab(a,b);
	ASSERT_EQUAL_OBJECT( ab, Halfplane_3_Segment_3::coplanar_intersection(p,q,r, ab) );
    }

    void t_both_off()
    {
	Segment xy(x,y);
	ASSERT_NULL_OBJECT( Halfplane_3_Segment_3::coplanar_intersection(p,q,r, xy));
    }

    void t_border()
    {
	Segment pl(p,l);
	ASSERT_EQUAL_OBJECT( pl, Halfplane_3_Segment_3::coplanar_intersection(p,q,r, pl) );
    }

    void t_crossing()
    {
	Segment bx(b,x);
	Segment bl(b,l);
	ASSERT_EQUAL_OBJECT( bl, Halfplane_3_Segment_3::coplanar_intersection(p,q,r, bx) );
    }
	
    void t_py()
    {
	Segment py(p,y);
	ASSERT_EQUAL_OBJECT( p,
			     Halfplane_3_Segment_3::coplanar_intersection(p,q,r, py));
    }
	
    void t_bl()
    {
	Segment bl(b,l);
	ASSERT_EQUAL_OBJECT( bl,
			     Halfplane_3_Segment_3::coplanar_intersection(p,q,r, bl));
    }
			     
    

    CPPUNIT_TEST_SUITE( Halfplane_3_test );
    CPPUNIT_TEST( t_both_on );
    CPPUNIT_TEST( t_both_off );
    CPPUNIT_TEST( t_border );
    CPPUNIT_TEST( t_crossing );
    CPPUNIT_TEST( t_py );
    CPPUNIT_TEST( t_bl );
    CPPUNIT_TEST_SUITE_END();
};


CPPUNIT_TEST_SUITE_REGISTRATION( Halfplane_3_test<R_test> );
