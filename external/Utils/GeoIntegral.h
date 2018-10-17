//
// C++ Interface: GeoPatchion
//
// Description: 
//
//
// Author: Kiran Varanasi <varanasi@monoceros>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef GEOPATCHES_H
#define GEOPATCHES_H
#include "../TransforMesh/Mesh.h"
#include <algorithm>


class GeoPatch {

  public:

	GeoPatch() { 
		active = true;
		color[0] = color[1] = color[2] = 0; 
		size = 0; id = -1; 
		root = NULL;
		neigh.clear(); neighdist.clear();
	}
	GeoPatch(Vertex_handle vi, int _id) {
		active = true;
		color[0] = color[1] = color[2] = 0; 
		size = 1; id = _id; 
		root = vi;
		neigh.clear(); neighdist.clear();
	}

	void addNeighbour(int i, float dist) {
		int p; for(p=0; p < neigh.size(); p++) { 
			if(neigh[p] == i) break;
		}
		if (p == neigh.size()) {
			neigh.push_back(i);
			neighdist.push_back(dist);
		} else { 
			if(neighdist[p] > dist) 
				neighdist[p] = dist;
		}
	}

	void deleteNeighbour(int i) {
		vector<int>::iterator p = find(neigh.begin(), neigh.end(), i);
		if (p != neigh.end()) {
			neigh.erase(p);
			neighdist.erase(neighdist.begin() + std::distance(p, neigh.begin()));
		}
	}

	int size;
	int id;
	Vertex_handle root;

	vector<int> neigh;
	vector<float> neighdist;
	float color[3];
	bool active;

};

class GeoIntegral{
public:
	GeoIntegral();
	GeoIntegral(Mesh *m, int colorFacets);
	void load(Mesh *m, int colorFacets); 
	~GeoIntegral();
	void cleanUp();
	void patchWarshall(float patchThresh);
	void geoIntegral(); 
	void setColors();
	void setColors(int colorStyle);

	Mesh *m;
	int colorFacets; // 1 = by geointegral // 2 = by patch // 3 = by cluster 
private:
	int changePatchId(Vertex_handle v, int new_id);

	vector<GeoPatch *> patches;
	float **patchDists; 

	// data structures for vertex-wise storage
	vector<int> *nearPatch;
	vector<float> *nearGeo;
	// In the MeshVertex variables the following information will be stored
	// float qual; // which stores the geodesic integral
};


#endif
