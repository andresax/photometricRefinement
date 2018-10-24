// -*- C++ -*-

template <class FT_>
struct Box_base_2
{
    static const int dimension = 2;
    typedef FT_ FT;

    FT low[dimension];
    FT high[dimension];

    Box_base_2() {}

    Box_base_2( FT min0, FT max0,
		FT min1, FT max1 )
    {
	low[0] = min0;  high[0] = max0;
	low[1] = min1;  high[1] = max1;
    }

    Box_base_2( FT min0, FT max0,
		FT min1, FT max1,
		FT min2, FT max2,
		FT min3, FT max3,
		FT min4, FT max4 )
    { CGAL_assertion(0); }
};


template <class FT_>
struct Box_base_5
{
    static const int dimension = 5;
    typedef FT_ FT;

    FT low[dimension];
    FT high[dimension];

    Box_base_5() {}

    Box_base_5( FT min0, FT max0,
		FT min1, FT max1 )
    { CGAL_assertion(0); }

    Box_base_5( FT min0, FT max0,
		FT min1, FT max1,
		FT min2, FT max2,
		FT min3, FT max3,
		FT min4, FT max4 )
    { 	low[0] = min0;  high[0] = max0;
	low[1] = min1;  high[1] = max1;
	low[2] = min2;  high[2] = max2;
	low[3] = min3;  high[3] = max3;
	low[4] = min4;  high[4] = max4;
    }
};


extern int next_box_id;


template <class Base_>
struct Box : public Base_
{
    typedef typename Base_::FT FT;

    int box_id;

    Box() : Base_() {}

    Box( FT min0, FT max0,
	 FT min1, FT max1 ) : Base_( min0,max0, min1,max1 ) 
    { 	
	box_id = next_box_id++;
    }

    Box( FT min0, FT max0,
	 FT min1, FT max1,
	 FT min2, FT max2,
	 FT min3, FT max3,
	 FT min4, FT max4 ) : Base_( min0,max0, min1,max1, min2,max2,
				     min3,max3, min4,max4 ) 
    { 	
	box_id = next_box_id++;
    }

    FT min(int d) const
    { return low[d]; }

    FT max(int d) const
    { return high[d]; }

    bool operator== ( const Box& b ) const
    {
	for( int d = 0; d < Box::dimension; ++d ) {
	    if ( min(d) != b.min(d) )
		return false;
	    if ( max(d) != b.max(d) )
		return false;
	}
	return true;
    }

    bool operator!= ( const Box& b ) const
    { 
	return !(*this == b);
    }
    
    bool operator< ( const Box& b ) const
    {
	for( int d = 0; d < Box::dimension; ++d )
	    if ( min(d) != b.min(d) )
		return min(d) < b.min(d);
	for( int d = 0; d < Box::dimension; ++d )
	    if ( max(d) != b.max(d) )
		return max(d) < b.max(d);
	return false; // boxes are equal
    }
};


template <class Base_>
std::ostream& operator<<( std::ostream& os, const Box<Base_>& b )
{
    os << "Box[" << b.box_id << "](";
    for( int d = 0; d < b.dimension; ++d )
	os << ' ' << b.low[d] << ':' << b.high[d];
    return os << ')';
}


typedef Box<Box_base_2<int> >           Box_2i;
typedef Box<Box_base_2<float> >         Box_2f;

typedef Box<Box_base_5<int> >           Box_5i;
typedef Box<Box_base_5<float> >         Box_5f;



template <class Box_pair>
Box_pair canonical_order( const Box_pair& bp )
{
    if ( bp.second < bp.first )
	return Box_pair(bp.second,bp.first);
    return bp;
}


template <class Base_>
std::ostream& operator<< ( std::ostream& os, 
			   const std::pair< Box<Base_>,Box<Base_> >& bp )
{
    return os << "pair<" << bp.first 
	      << ',' << bp.second
	      << '>';
}


