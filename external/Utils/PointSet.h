//
// C++ Interface: PointSet
//
// Description: 
//
//
// Author: Andrei Zaharescu <andrei.zaharescu@inrialpes.fr>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef POINT_SET_H
#define POINT_SET_H

#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits.h>

#include <map>
#include <vector>
#include <iostream>
#include <fstream>

#include "Mesh.h"

class PointUnit {
public:
	PointUnit(Point p, Vector n) {
		point = p;
		normal = n;
	}

	Vector normal;
	Point point;
};


class PointSet {
public:
	////////////////////////////////////////////////////////////////////////
	void add(Point p, Vector n) {
//		cout << "added p=" << p << " with normal=" << n << endl;
		points.push_back(new PointUnit(p,n));
	}

	////////////////////////////////////////////////////////////////////////
	void load(const char *filename) {
		cout << " Loading Pointset (" << filename << "): ";
		ifstream is ( filename );
		int n=0;
		Point p;
		Vector v;
		while (true) {
			if ((is >> p) && (is >> v)) {
				add(p,v);
				n++;
			}
			else 
				break;
		}
		cout << "[OK] - loaded " << n << " points with normals" << endl;
	}

	////////////////////////////////////////////////////////////////////////
	Point closestPoint(Point &the_point) {
		if ( !is_search_init ) {
			for(int i=0;i<points.size();i++) {
				search_tree.insert (points[i]->point);
			}
			is_search_init = true;
		}
		Neighbor_search search ( search_tree, the_point, 1 );	
		
		for ( Neighbor_search::iterator it = search.begin(); it != search.end(); ++it )
		{
			return it->first;
		}
	}

	////////////////////////////////////////////////////////////////////////
	Vector normalMapping(Point &the_point) { 
		// we can change the mapping to a VectorUnit if we need to retrieve more data
		if ( !is_mapping_init ) {
			for(int i=0;i<points.size();i++) {
				point_mapping[points[i]->point] = points[i]->normal;
			}
			is_mapping_init = true;
		}
		return point_mapping[the_point];
	}

	////////////////////////////////////////////////////////////////////////
	PointSet(const char * filename) {
		is_search_init=false;
		is_mapping_init=false;
		load(filename);
	}

	////////////////////////////////////////////////////////////////////////	
	float* getBoundingBox() {
		//init the bounding box with the first vertex	
		Point first_point(0,0,0);
		if (points.size()>0)
			first_point = points[0]->point;
		for ( int i=0;i<6;i++ )
			bounding_box[i] = first_point[i%3];
	
		//do a min/max
		for(int i=0;i<points.size();i++)
			for ( int j=0;j<3;j++ )
			{
				if ( bounding_box[j] > TO_FLOAT ( points[i]->point[j] ) ) bounding_box[j] = TO_FLOAT ( points[i]->point[j] );
				if ( bounding_box[j+3] < TO_FLOAT ( points[i]->point[j] ) ) bounding_box[j+3] = TO_FLOAT ( points[i]->point[j] );
			}
		return bounding_box;
	}

	////////////////////////////////////////////////////////////////////////
	PointSet() {
		is_search_init=false;
		is_mapping_init=false;
	}

	~PointSet() {
		for(int i=0;i<points.size();i++)
			delete points[i];
	}

//members
	vector<PointUnit*> points;
	Neighbor_search_tree search_tree;
	bool is_search_init;
	std::map<Kernel::Point_3,Kernel::Vector_3> point_mapping;
	bool is_mapping_init;
	float bounding_box[6];
};
#endif