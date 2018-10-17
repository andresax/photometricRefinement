//
// C++ Implementation: GeoIntegral
//
// Description: 
//
//
// Author: Kiran Varanasi <varanasi@monoceros>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "GeoIntegral.h"
#include "ColorMap.h"
#include <queue>

GeoIntegral::GeoIntegral() {
	m = NULL;
	patches.clear(); patchDists = NULL; 
	nearPatch = NULL; 
	nearGeo = NULL;   
}

GeoIntegral::GeoIntegral(Mesh* _m, int _colorFacets) {
	load(_m, _colorFacets);  
}

GeoIntegral::~GeoIntegral() {
	cleanUp();
}

void GeoIntegral::load(Mesh* _m, int _colorFacets) { 
	m = _m;
	colorFacets = _colorFacets;
}

////////////////////////////////////////////////////////////////////////////////////////////
int GeoIntegral::changePatchId(Vertex_handle f, int new_id) {
	vector<int> result;
	int current_id=f->id;
	std::queue<Halfedge_handle> edgeQueue;

	int neigh_size=0;
	HV_circulator hh = f->vertex_begin();
	f->id = new_id;
	neigh_size++;
	do {
		edgeQueue.push(hh);
	} while ( ++hh != f->vertex_begin() );

	while ( !edgeQueue.empty() ) {

		Halfedge_handle edge = edgeQueue.front(); edgeQueue.pop();


		if (edge->opposite()->facet()->id==current_id) {
			edge->opposite()->vertex()->id=new_id;
			neigh_size++;

			edgeQueue.push(edge->opposite()->next());
			edgeQueue.push(edge->opposite()->next()->next());
		}
	}
	return neigh_size;
}

void GeoIntegral::patchWarshall(float patchThresh) { 
	// build a vector of vertex handles
	Vertex_handle vi,vj; 
	vector<Vertex_handle> meshVerts; meshVerts.clear(); 
	vector<Vertex_handle> patchVerts; 
	int numVert = m->p.size_of_vertices();
	bool done = false;
	int remVerts = numVert, curPatch=-1; 
	float curGeo; 
	
	GeoPatch *newPatch; 
	HV_circulator hv; 

	int i=0,j=0,k=0,n=0, curk=0;

	nearPatch = new vector<int>[numVert];
	nearGeo = new vector<float>[numVert];
	for(vi = m->p.vertices_begin(); vi != m->p.vertices_end(); vi++) { 
		meshVerts.push_back(vi); 

		nearPatch[n].clear(); nearGeo[n].clear();
		vi->id = n++; 
	}

	remVerts = m->p.size_of_vertices();
	// Divide the mesh into patches	
	while(! done) { 
		done = false; 
		curPatch++;
		// build a patch from an unvisited random vertex
		k = (int)(rand()/(RAND_MAX+1.0)*(numVert));
		while(nearPatch[k].size() > 0) { // while(! visited[k])  
			k = (k+1)%numVert;
		}
		vi = meshVerts[k];

		nearPatch[vi->id].push_back(curPatch); nearGeo[vi->id].push_back(0);  
		remVerts--;
		newPatch = new GeoPatch(vi, curPatch);
		patches.push_back(newPatch); 

		//fprintf(stderr, "Patch(%d) from vertex(%d) : ", curPatch, vi->id);
		patchVerts.clear(); 
		patchVerts.push_back(vi);
		int numTraversed=0;
		while(numTraversed <  patchVerts.size()) {
			vi = patchVerts[numTraversed];
			for(curk=0; curk < nearPatch[vi->id].size(); curk++) { 
				if(nearPatch[vi->id][curk] == curPatch) break;
			}

			curGeo = nearGeo[vi->id][curk] + 1;
			numTraversed++;
			if(curGeo > patchThresh) continue; 
			hv = vi->vertex_begin();
			//fprintf(stderr, " %d, ", vi->id);
			do { 
				vj = hv->opposite()->vertex();
				vector<Vertex_handle>::iterator vfind = find(patchVerts.begin(), patchVerts.end(), vj); 
				if(vfind != patchVerts.end()) 
					continue;

				if(nearPatch[vj->id].size() == 0) { // a first visit by any patch 
					remVerts--;
					newPatch->size++;  // counts to the size of the current patch
					nearPatch[vj->id].push_back(curPatch);
					nearGeo[vj->id].push_back(curGeo);
				} else { 
					for(curk=0; curk < nearPatch[vj->id].size(); curk++) { 
						if(nearPatch[vj->id][curk] == curPatch) break;
					}
					if(curk == nearPatch[vj->id].size()) { // vertex unvisited by current patch
						nearPatch[vj->id].push_back(curPatch);
						nearGeo[vj->id].push_back(curGeo);
					} else { 
						if(nearGeo[vj->id][curk] > curGeo) 
							nearGeo[vj->id][curk] = curGeo;
					}

					for(k=0; k < nearPatch[vj->id].size(); k++) {
						if(k==curk) continue;
						newPatch->addNeighbour(nearPatch[vj->id][k], curGeo+nearGeo[vj->id][k]);
						patches[nearPatch[vj->id][k]]->addNeighbour(curPatch, curGeo + nearGeo[vj->id][k]);
					}
				}

				patchVerts.push_back(vj);
 			} while(++hv != vi->vertex_begin()); 
		}
		//fprintf(stderr, " # traversed(%d) new-tagged(%d) remVerts(%d) \n", numTraversed, newPatch->size, remVerts);

		if(remVerts == 0) done = true; 
	}
	meshVerts.clear();

	// Initialize the distances in the first level of neighborhood
	//fprintf(stderr, "Initial Neighbours ::: \n");
	patchDists = new float * [patches.size()];
	for(i=0; i < patches.size(); i++) {
		patchDists[i] = new float [patches.size()]; 
		for(j=0; j < patches.size();j++)
			patchDists[i][j] = 9999;
		patchDists[i][i] = 0; 

		//fprintf(stderr, "Patch(%d) : ", i);
		for(j=0; j < patches[i]->neigh.size(); j++) { 
			n = patches[i]->neigh[j]; 
			patchDists[i][n] = patches[i]->neighdist[j];
			//fprintf(stderr, "%d (%3.1f), ", n, patchDists[i][n]); 
		}
		//fprintf(stderr, "\n");
	}
	//fprintf(stderr, "Finished First level of adding neighbor-patch distances \n");
	// Transitive loop on the distances (warshall's algorithm on patch-roots) 
	for(k=0; k < patches.size(); k++) { 
		for(i=0; i < patches.size(); i++) { 
			for(j=0; j < patches.size(); j++) { 
				if(patchDists[i][j] > patchDists[i][k] + patchDists[k][j]) 
					patchDists[i][j] = patchDists[i][k] + patchDists[k][j]; 
			}
		}
	}
	//fprintf(stderr,  "Finished GeoPatches and the Warshall Algorithm\n");
}

void GeoIntegral::geoIntegral()
{
	Vertex_handle vi;
	int vid,j,n,p;
	float mind;
	float maxg=0, ming=99999;  

	for(vi=m->p.vertices_begin(); vi != m->p.vertices_end(); vi++) { 
		vid = vi->id;
		vi->qual=0; 

		for(n=0; n < patches.size(); n++) { 
			mind=9999;
			for(j=0; j < nearPatch[vid].size(); j++) { 
				p = nearPatch[vid][j];
				if(nearGeo[vid][j]+patchDists[p][n] < mind) 
					mind = nearGeo[vid][j]+patchDists[p][n];
			}
			vi->qual += mind * patches[n]->size;
		}
		if(vi->qual > maxg) maxg = vi->qual;
		if(vi->qual < ming) ming = vi->qual; 
	}
	fprintf(stderr, "GeoIntegral :: Crude Values :: maxg(%f) ming(%f) \n", maxg, ming);	
	for(vi=m->p.vertices_begin(); vi != m->p.vertices_end(); vi++) { 
		vi->qual = (vi->qual-ming)/maxg;
	}
	//fprintf(stderr, "Computed GeoIntegral\n");
	setColors();
	//setColors(1);

}

void GeoIntegral::setColors(int _colorFacets)
{
	colorFacets = _colorFacets;
	setColors();
}

void GeoIntegral::setColors()
{
	Vertex_handle vi; 
	int k, mink;

	for(vi = m->p.vertices_begin(); vi != m->p.vertices_end(); vi++) { 
		if(colorFacets == 1) { // color by geoIntegral :: "vi->qual" variable
			ColorMap::jet(vi->qual, (float *)vi->color);
		} else if(colorFacets == 2) { // color by geoPatch 
			mink=0;
			for(k=0; k < nearPatch[vi->id].size(); k++) { 
				if(nearGeo[vi->id][k] < nearGeo[vi->id][mink]) mink = k; 
			}
			ColorMap::jet((float)(nearPatch[vi->id][mink]+1.0)/patches.size(), (float *)vi->color);
		} else if(colorFacets == 3) { // color by segments 
			// implement later 
		} 
	}
	// deletePatches();
}

void GeoIntegral::cleanUp()
{
	if(m == NULL) return;
	int i;

	for(i=0; i < m->p.size_of_vertices(); i++) { 
		nearPatch[i].clear(); nearGeo[i].clear(); 
	}
	delete [] nearPatch; 
	delete [] nearGeo;

	for(i=0; i < patches.size(); i++){ 
		delete [] patchDists[i];

		patches[i]->neigh.clear(); 
		patches[i]->neighdist.clear();
		delete patches[i];
	}
	delete [] patchDists; 

	patches.clear();
}
