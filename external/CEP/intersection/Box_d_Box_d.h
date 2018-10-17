#ifndef CEP_INTERSECTION_BOX_D_BOX_D_H
#define CEP_INTERSECTION_BOX_D_BOX_D_H


namespace CEP {    
namespace intersection {


    /*! \brief Specialized Box/Box intersection functions.
     */
    namespace Box_d_Box_d {

	//! True if the boxes intersect on the d'th axis.
	template <class Box>
	inline 
	bool do_intersect( const Box& b1, const Box& b2, int d )
	{
	    return b1.min(d) <= b2.max(d) && b2.min(d) <= b1.max(d);
	}
    }


//! True if the boxes intersect.
/*! \ingroup grp_do_intersect
 */
template <class Box>
inline 
bool do_intersect( const Box& b1, const Box& b2 )
{
    for( int d = 0; d < Box::dimension; ++d )
	if ( !Box_d_Box_d::do_intersect(b1,b2,d) )
	    return false;
    return true;
}



}
}


#endif
