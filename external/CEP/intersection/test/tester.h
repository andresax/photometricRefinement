#include <cppunit/SourceLine.h>
#include <cppunit/TestAssert.h>
#include <cppunit/extensions/HelperMacros.h>


#include <list>
#include <string>

#if USE_GMPZ
#   include <CGAL/Gmpz.h>
#   include <CGAL/Homogeneous.h>
    typedef CGAL::Homogeneous<CGAL::Gmpz>   R_test;
#endif

#if USE_CORE
#   include <CEP/core_expr.h>
#   include <CGAL/Cartesian.h>
    typedef CGAL::Cartesian<CORE::Expr> R_test;
#endif


#include <CGAL/circulator.h>
#include <CGAL/Object.h>
#include <CEP/intersection/Object.h>
#include <CGAL/Point_2.h>
#include <CGAL/Point_3.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_3.h>

#include <CGAL/IO/Verbose_ostream.h>


extern CGAL::Verbose_ostream verr;


using std::string;


namespace CppUnit {

    template <class T>
    struct cgal_assertion_traits
    {
	static bool equal( const T& x,
			   const T& y )
	{ return x == y; }
	
	static string toString( const T& x )
	{
	    CppUnit::OStringStream ost;
	    CGAL::set_pretty_mode(ost);
	    ost << x;
	    return ost.str();
	}
    };

    
    template <class R>
    struct assertion_traits<CGAL::Point_3<R> > 
	: public cgal_assertion_traits<CGAL::Point_3<R> >
    {};


    template <class R>
    struct assertion_traits<CGAL::Plane_3<R> > 
	: public cgal_assertion_traits<CGAL::Plane_3<R> >
    {};


    template <class R>
    struct assertion_traits<CGAL::Segment_3<R> > 
	: public cgal_assertion_traits<CGAL::Segment_3<R> >
    {
	static bool equal( const CGAL::Segment_3<R>& s,
			   const CGAL::Segment_3<R>& t )
	{
	    return s == t ||
		( s.source() == t.target() && s.target() == t.source() );
	}
    };


    template <class R>
    struct assertion_traits<CGAL::Triangle_3<R> > 
	: public cgal_assertion_traits<CGAL::Triangle_3<R> >
    {
	static bool equal( const CGAL::Triangle_3<R>& T,
			   const CGAL::Point_3<R>& a,
			   const CGAL::Point_3<R>& b,
			   const CGAL::Point_3<R>& c )
	{
	    return a == T[0] && b == T[1] && c == T[2];
	}

	static bool equal( const CGAL::Triangle_3<R>& T1,
			   const CGAL::Triangle_3<R>& T2 )
	{
	    return equal( T1, T2[0], T2[1], T2[2] )
		|| equal( T1, T2[0], T2[2], T2[1] )
		|| equal( T1, T2[1], T2[0], T2[2] )
		|| equal( T1, T2[1], T2[2], T2[0] )
		|| equal( T1, T2[2], T2[0], T2[1] )
		|| equal( T1, T2[2], T2[1], T2[0] );
	}
    };


    // Point list that represents a polygon
    template <class R>
    struct assertion_traits<std::list<CGAL::Point_2<R> > >
    {
	typedef CGAL::Point_2<R>            Point;
	typedef std::list<Point>            Point_list;
	typedef typename Point_list::const_iterator  Point_iterator;
	typedef CGAL::Circulator_from_iterator<Point_iterator>  
	                                    Point_circulator;

	// Test if two circulators point to the same sequence of points
	static bool same_sequence( Point_circulator begin1,
				   Point_circulator begin2 )
	{
	    Point_circulator p1 = begin1;
	    Point_circulator p2 = begin2;

	    do {
		if ( *p1++ != *p2++ )
		    return false;
	    } while ( p1 != begin1 && p2 != begin2 );

	    return p1 == begin1 && p2 == begin2;
	}

	static bool equal( const Point_list& poly1,
			   const Point_list& poly2 )
	{
	    Point_circulator begin1( poly1.begin(), poly1.end() );
	    Point_circulator begin2( poly2.begin(), poly2.end() );

	    Point_circulator c1 = begin1;
	    do {
		if ( same_sequence( c1, begin2 ) )
		    return true;
	    } while ( ++c1 != begin1 );
	    return false;
	}

	static string toString( const Point_list& poly )
	{
	    return "[polygon]";
	}
    };


}


template <class R>
std::string toString( const CGAL::Object& obj )
{
    CppUnit::OStringStream ost;
    CGAL::set_pretty_mode(ost);

    CGAL::Point_3<R> p;
    CGAL::Segment_3<R> s;
    CGAL::Triangle_3<R> t;
    CGAL::Plane_3<R> P;

    if ( obj.is_empty() )
	ost << "[empty object]";
    else if ( assign(p,obj) ) 
	ost << p;
    else if ( assign(s,obj) ) 
	ost << s;
    else if ( assign(t,obj) ) 
	ost << t;
    else if ( assign(P,obj) ) 
	ost << P;
    else
	ost << "[unknown object]";

    return ost.str();
}


template <class T>
inline void assert_equals_object( const T& expected,
				  const CGAL::Object& actual_obj,
				  CppUnit::SourceLine sourceline )
{
    typedef typename T::R R;
    CGAL::Object act_nondegen = 
	CEP::intersection::make_object_nondegenerate<R>( actual_obj );
    T actual;

    if (assign( actual, act_nondegen )) {
	CppUnit::TestAssert::assertEquals( expected, actual, sourceline );
    } else {
	CppUnit::Asserter::failNotEqual( CppUnit::assertion_traits<T>::toString(expected),
					 toString<R>(actual_obj),
					 sourceline );

    }
}


#define ASSERT_EQUAL_OBJECT(expected,actual) \
    assert_equals_object( (expected),(actual),CPPUNIT_SOURCELINE() )

#define ASSERT_NULL_OBJECT(actual) CPPUNIT_ASSERT( (actual).is_empty() )

