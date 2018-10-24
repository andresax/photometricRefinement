/*
 *  MeshNoiser.h
 *  MeshMatching
 *
 *  Created by Andrei Zaharescu on 10-10-11.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "Mesh.h"

class MeshNoiser {
	
public:
	enum NoiseType {NoiseUniform=0, NoiseGaussian=1, NoiseSaltAndPepper=2};
	MeshNoiser();

//	ModifGeomLocalScale, ModifGeomRotate, ModifGeomNoise, ModifGeomShotNoise, ModifColourNoise, ModifGeomSampling, ModifGeomMicroHoles, ModifGeomHoles, ModifGeomTopology};	

//	static void noiseColorNoise(Mesh &m, float sigma);
//	static void noiseShotNoise(Mesh &m, float sigma);	

	static bool preNoise(Mesh &m, float sigma);
	static void postNoise(Mesh &m);	
	
	static void noiseColour(Mesh &m, float noise_sigma, int noise_type);
	
	static void noiseGeom(Mesh &m, float noise_sigma, int noise_type);
	static void noiseGeomRotate(Mesh &m, float noise_sigma, int noise_type);
	static void noiseGeomScale(Mesh &m, float noise_sigma);
	static void noiseGeomLocalScale(Mesh &m, float noise_sigma);	
	static void noiseGeomHoles(Mesh &m, float noise_sigma, int no_rings);
	static void noiseGeomSampling(Mesh &m, float noise_sigma);
	static void noiseGeomTopology(Mesh &m, float noise_sigma, Mesh &m2);

	
};