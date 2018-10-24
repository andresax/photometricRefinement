//
// C++ Interface: MeshError
//
// Description: 
//
//
// Author: Andrei Zaharescu <andrei.zaharescu@inrialpes.fr>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef MESH_ERROR_H
#define MESH_ERROR_H

#include "Mesh.h"
#include "PointSet.h"

class MeshError {

public:
	static void computeErrors(Mesh & m, PointSet & ps, float completeness_cutoff, float & total_error, float & total_completeness) {
		total_error=0.0f;
		total_completeness=0.0f;
		int total_elements=0;
/*		for ( Vertex_iterator v = m.p.vertices_begin(); v != m.p.vertices_end(); ++v ) {
			Kernel::Point_3 closest_point = ps.closestPoint(v->point());
			Kernel::Vector_3 closest_normal = ps.normalMapping(closest_point);
			float local_error = std::abs(closest_normal*(closest_point-v->point()));
			total_error += local_error;
			total_elements++;
		}

		for ( Facet_iterator f = m.p.facets_begin(); f != m.p.facets_end(); ++f ) {
			Point facet_p=f->center();
			Kernel::Point_3 closest_point = ps.closestPoint(facet_p);
			Kernel::Vector_3 closest_normal = ps.normalMapping(closest_point);
			float local_error = std::abs(closest_normal*(closest_point-facet_p));
			total_error += local_error;
			total_elements++;
		}
 */
		for(vector<PointUnit*>::iterator pi=ps.points.begin();pi!=ps.points.end();pi++) {
			
			Kernel::Point_3 local_point=(*pi)->point;
			Kernel::Vector_3 local_normal=(*pi)->normal;
			Vertex_handle v = m.closestVertex(local_point);
			Kernel::Point_3 closest_point = v->point();
			Kernel::Vector_3 closest_normal = v->normal();
			//float local_error = std::abs(closest_normal*(closest_point-local_point));
			float local_error = std::abs(local_normal*(closest_point-local_point));			
			if (local_error < completeness_cutoff) 
				total_completeness+=1.0f;
			total_error += local_error;
			total_elements++;
		}
		
		
		if (total_elements>0) {
			total_completeness=total_completeness/total_elements*100.0;
			total_error /= total_elements;			
		}		
	}

	float computeCompletenessErrors();


};

#endif