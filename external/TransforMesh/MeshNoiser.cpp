/*
 *  MeshNoiser.cpp
 *  MeshMatching
 *
 *  Created by Andrei Zaharescu on 10-10-11.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "MeshNoiser.h"


////////////////////////////////////////////////////////////////
// Generates noise with a noise model (0=uniform, 1 = gaussian, 2 = salt and pepper)
// Returns: a value between {-1, 1}
float getNoise(float noise_sigma, int noise_type=0) {
	float random_value=noise_sigma*((rand()*1.0f)/RAND_MAX);
	if (rand()>RAND_MAX/2) random_value*=-1;
	if (noise_type==0) // uniform
		return random_value;
	if (noise_type==1) // gaussian
		return GAUSSIAN(noise_sigma,random_value*4);
	if (noise_type==2) // salt and pepper noise (also called shot noise)
	{
		if (noise_sigma > rand()*1.0f/RAND_MAX)
			return random_value/noise_sigma;		
		else
			return 0;
	}
	
}

////////////////////////////////////////////////////////////////
bool MeshNoiser::preNoise(Mesh &m, float noise_sigma)
{
	if (noise_sigma<0.00001f) return false;
	return (noise_sigma>=0.0f) && (noise_sigma<=1.0f);
}

////////////////////////////////////////////////////////////////
void MeshNoiser::postNoise(Mesh &m)
{
	m.updateMeshData();
}

void MeshNoiser::noiseColour(Mesh &m, float noise_sigma, int noise_type)
{
	if (!preNoise(m,noise_sigma)) return;
	if (noise_type==0) //gaussian
	{
		cout << " - noise colour: gaussian noise by " << noise_sigma <<endl;
		noise_sigma=noise_sigma*m.edge_avg;
		for ( Vertex_iterator vi = m.p.vertices_begin(); vi!=m.p.vertices_end(); vi++ )
		{
			for(int i=0;i<3;i++)
				vi->color[i] = std::min(std::max(getNoise(noise_sigma*255,0) + vi->color[i] ,0.0f),255.0f);
		}
		
	}
	else
		if (noise_type==2) // shot noise
		{ 
			cout << " - noise colour: shot noise by " << noise_sigma <<endl;
			for ( Vertex_iterator vi = m.p.vertices_begin(); vi!=m.p.vertices_end(); vi++ )
			{
				float rnd_value = std::min(std::max(abs(getNoise(noise_sigma,2)),0.0f),1.0f);
				if (rnd_value > noise_sigma)
					for(int i=0;i<3;i++)
						vi->color[i] = std::min(std::max(getNoise(50,0) + vi->color[i] ,0.0f),255.0f);
			}
		}

	
	postNoise(m);	
}

///////////////////////////////////////////////////////////////////////////
void MeshNoiser::noiseGeom(Mesh &m, float noise_sigma, int noise_type)
{
	if (!preNoise(m,noise_sigma)) return;
	
	if (noise_type==0) //gaussian
	{
		cout << " - noise geometry: gaussian noise by " << noise_sigma <<endl;
		noise_sigma=noise_sigma*m.edge_avg;
		for ( Vertex_iterator vi = m.p.vertices_begin(); vi!=m.p.vertices_end(); vi++ )
		{
			vi->point() = vi->point() + Vector(getNoise(noise_sigma,noise_type),getNoise(noise_sigma,noise_type),getNoise(noise_sigma,noise_type));
		}
		
	}
	else
	if (noise_type==2) // shot noise
	{ 
		cout << " - noise geometry: shot noise by " << noise_sigma <<endl;
		float noise_sigma_edge=20*m.edge_avg;
		for ( Vertex_iterator vi = m.p.vertices_begin(); vi!=m.p.vertices_end(); vi++ )
		{
			float rnd_value = std::min(std::max(abs(getNoise(noise_sigma,2)),0.0f),1.0f);
			if (rnd_value > noise_sigma)
				vi->point() = vi->point() + vi->normal()*abs(getNoise(noise_sigma_edge,0));					
		}
	}
	postNoise(m);	
}



///////////////////////////////////////////////////////////////////////////
void MeshNoiser::noiseGeomScale(Mesh &m, float noise_sigma)
{
	if (!preNoise(m,noise_sigma)) return;
	
	noise_sigma=(2*noise_sigma);
	for ( Vertex_iterator vi = m.p.vertices_begin(); vi!=m.p.vertices_end(); vi++ )
	{
		Point p = vi->point();
		vi->point() = Point(p.x()*noise_sigma,p.y()*noise_sigma,p.z()*noise_sigma);
	}
	cout << " - noise geometry: scale by " << noise_sigma <<endl;	
	
	postNoise(m);
}

///////////////////////////////////////////////////////////////////////////
void MeshNoiser::noiseGeomLocalScale(Mesh &m, float noise_sigma)
{
	if (!preNoise(m,noise_sigma)) return;

	int noIters = floor(noise_sigma*10);
	for(int i=0;i< noIters; i++)
		m.dilate(m.edge_avg/3);		
	cout << " - noise geometry: local scale by " << noise_sigma <<endl;
	
	postNoise(m);
}


////////////////////////////////////////////////////////////////
void MeshNoiser::noiseGeomRotate(Mesh &m, float noise_sigma, int noise_type)
{
	if (!preNoise(m,noise_sigma)) return;
	float rx = std::min(std::max(abs(getNoise(noise_sigma,noise_type)),0.0f),1.0f)*360.0f;
	float ry = std::min(std::max(abs(getNoise(noise_sigma,noise_type)),0.0f),1.0f)*360.0f;
	float rz = std::min(std::max(abs(getNoise(noise_sigma,noise_type)),0.0f),1.0f)*360.0f;
	cout << " - noise geometry: rotate by (" << rx << "," << ry << "," << rz << ") from sigma=" << noise_sigma <<endl;
	m.rotate(rx,ry,rz,true);
	postNoise(m);		
}

////////////////////////////////////////////////////////////////

void MeshNoiser::noiseGeomHoles(Mesh &m, float noise_sigma, int no_rings)
{
	if (!preNoise(m,noise_sigma)) return;
	int noRemovals  = floor(noise_sigma*10.0f);		
	int noRings  = no_rings;
	cout << " - noise geometry: holes with sigma=" << noise_sigma << " (no_removals=" << noRemovals << " no_rings=" << noRings << ")" << endl;
	for(int k=0; k <noRemovals; k++)
	{
		//pick a random vertex
		int id  = floor(std::min(std::max(abs(getNoise(noise_sigma,0)),0.0f),1.0f)*(m.p.size_of_vertices()-1));
		Vertex *el = m.getVertexById(id);
		if (el)	m.makeHole(el,noRings);
	}	
	postNoise(m);		
}

////////////////////////////////////////////////////////////////
 void MeshNoiser::noiseGeomSampling(Mesh &m, float noise_sigma)
 {
	 if (!preNoise(m,noise_sigma)) return;
	 float targetEdge = m.edge_avg*(1.0f+noise_sigma*10);
	 cout << " - noise geometry: sub-sampling n=" << noise_sigma <<endl;
	 m.ensureEdgeSizes(targetEdge*0.49,targetEdge*1.51,0.2,150);
	 //remesh(Remesh_Sqrt3,noIters);				
	 postNoise(m);		
 }

////////////////////////////////////////////////////////////////
void MeshNoiser::noiseGeomTopology(Mesh &m, float noise_sigma, Mesh &m2)
{
	if (!preNoise(m,noise_sigma)) return;
	float *bbox_m = m.getBoundingBox();
	float *bbox_m2 = m2.getBoundingBox(); // reference plane

	// gotto decide which is the biggest dimension (axis)
	float dim[3];
	float maxDim=dim[0];
	int axis = 0;
	
	float planeMaxDim=0;
	
	for(int i=0;i<3;i++)
	{
		dim[i] = abs(bbox_m[i]-bbox_m[i+3]);
		if (dim[i]>maxDim)
		{
			maxDim = dim[i];
			axis = i;
		}
		planeMaxDim=std::max(planeMaxDim,abs(bbox_m2[i]-bbox_m2[i+3]));
	}
	
//	m2.scale(1,1,0.3);
	// act accordingly if not the Z axis
	if (axis==0) m2.rotate(0,90,0,false);
	if (axis==1) m2.rotate(90,0,0,false);
	m2.scale(maxDim/planeMaxDim,maxDim/planeMaxDim,maxDim/planeMaxDim);
	
	int noSlices=floor(10*noise_sigma);

	// set the proper step size
	float vSteps[3];
	for(int i=0;i<3;i++) vSteps[i] = 0;
	vSteps[axis] = (bbox_m[axis+3]-bbox_m[axis])/(noSlices+1);

	
	m2.invert();
	
	// translate plane to the center of the mesh
	m2.move((bbox_m[0]+bbox_m[3])/2 - (bbox_m2[0]+bbox_m2[3])/2,
			(bbox_m[1]+bbox_m[4])/2 - (bbox_m2[1]+bbox_m2[4])/2,
			(bbox_m[2]+bbox_m[5])/2	- (bbox_m2[2]+bbox_m2[5])/2);
/*	m2.move((bbox_m[0]+bbox_m[3])/2,
			(bbox_m[1]+bbox_m[4])/2,
			(bbox_m[2]+bbox_m[5])/2);
*/	
	// translate to the bottom of the axis
	m2.move(-vSteps[0]*(noSlices+1)/2, -vSteps[1]*(noSlices+1)/2, -vSteps[2]*(noSlices+1)/2);

	for(int i=0; i<noSlices; i++)
	{
		m2.move(vSteps[0],vSteps[1],vSteps[2]);
		m.unionWith(m2);
	}
	m.removeSelfIntersections();
	cout << " - noise geometry: topology with n=" << noise_sigma <<endl;
	postNoise(m);		
}

