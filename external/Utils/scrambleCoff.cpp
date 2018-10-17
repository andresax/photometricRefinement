#include<stdlib.h>
#include <stdio.h>
#include <vector>

typedef struct {
	int id;
	float coord[3];
	float rgb[4];
} sVertex;

typedef struct {
	int id;
	int v[3];
	int matid;
} sFace;

std::vector<sVertex> vertices;
std::vector<sFace> faces;
std::vector<int> vGT; // input groundtruth


std::vector<int> vIndices;


#define BUFFSIZE 20000

int main(int argc, char**argv)
{
	
	FILE *F;
	
	if (argc==1)
	{
		printf("COFF vertex index scrambler.\n");	
		printf("Usage: ./scrambleCoff InputCoffFile InputIndexFile OutputCoffFile OutputIndexFile\n");
		printf("        InputIndexFile can be 0 if not existent.\n");			
		return 0;
	}

	//
	// read in the input mesh
	//

	F=fopen(argv[1],"r");
	if (!F)
	{
		printf("Could not read source file %s\n",argv[1]);
		return -1;
	}
	printf("Reading input mesh %s\n",argv[1]);
	
	char buff[BUFFSIZE];
	int noVertices;
	int noFacets;	
	int noSomethingElse;		

	fscanf(F,"%s\n",buff);
	if ( (strcmp(buff,"COFF")!=0) && (strcmp(buff,"coff")!=0) )
	{
		printf("Input file is not of COFF format");
		exit(EXIT_FAILURE);
	}
	fscanf(F,"%d %d %d\n",&noVertices,&noFacets,&noSomethingElse);	
	
	sVertex v;
	for(int i=0;i<noVertices;i++)
	{
		fscanf(F, "%f %f %f %f %f %f %f\n", &v.coord[0], &v.coord[1], &v.coord[2], &v.rgb[0], &v.rgb[1], &v.rgb[2], &v.rgb[3]);
		vertices.push_back(v);
	}
			
	sFace f;
	for(int i=0;i<noFacets;i++)
	{
		int iFaceNoVertices;
		fscanf(F, "%d", &iFaceNoVertices);
		if (iFaceNoVertices!=3)
		{
			printf("Only 3 vertices per facets are currently supported.\n");
			exit(EXIT_FAILURE);
		}
		for(int j=0;j<iFaceNoVertices;j++)
			fscanf(F, "%d", &f.v[j]);
		faces.push_back(f);
	}
	fclose(F);


	//
	// read in the input index groundtruth file
	//

	F=fopen(argv[2],"r");
	if (!F)
	{
		printf("No input index file found\n",argv[2]);
		for(int i=0;i<noVertices;i++)
			vGT.push_back(i);
	}
	else
	{
		printf("Reading input index %s\n",argv[2]);
		int noElems;
		int tmpElem;
		fscanf(F,"%d",&noElems);
		if (noElems!=noVertices)
		{
			printf("Input Index file does not contain the same number of correspondences as the number of vertices from the input mesh (%d!=%d)\n",noElems,noVertices);
			exit(EXIT_FAILURE);
		}
		for(int i=0;i<noVertices; i++)
		{
			fscanf(F,"%d",&tmpElem);
			vGT.push_back(tmpElem);
		}
			
	}

	fclose(F);


	//
	// produce permutation
	//
	
	std::vector<int> vTmpIndices;
	std::vector<int> vNewIndices;
	
	vIndices.resize(noVertices);
	vTmpIndices.reserve(noVertices);
	vIndices.clear();
	vTmpIndices.clear();
	
	for(int i=0;i<noVertices; i++)
		vTmpIndices.push_back(i);
		
	for(int i=0;i<noVertices; i++)
	{
		int pos = rand() % vTmpIndices.size();
		int idx = vTmpIndices[pos];
		vTmpIndices.erase(vTmpIndices.begin() + pos);

		vNewIndices.push_back(idx); // new to old 
		vIndices[idx] = i; // old to new 
	}
	
	//
	// write out the output mesh
	//

	printf("Writing output mesh %s\n",argv[3]);

	F=fopen(argv[3],"w");
	if (!F)
	{
		printf("Could not open destination file %s\n",argv[3]);
		return -1;
	}
	
	fprintf(F,"COFF\n");
	fprintf(F,"%d %d %d\n",(int)vertices.size(), (int)faces.size(), noSomethingElse);

	for(int i=0;i<vertices.size();i++)
	{
		for(int k=0; k<3; k++)
			fprintf(F,"%f ",vertices[vNewIndices[i]].coord[k]);
		for(int k=0; k<3; k++)
			fprintf(F,"%f ",vertices[vNewIndices[i]].rgb[k]);
		fprintf(F,"%d",(int)vertices[vNewIndices[i]].rgb[3]);			
		fprintf(F,"\n");
	}

	for(int i=0;i<faces.size();i++)
	{
		fprintf(F,"3 ");
		for(int k=0; k<3; k++)
			fprintf(F,"%d ",vIndices[faces[i].v[k]]);
		fprintf(F,"\n");
	}

	fclose(F);
	
	
	//
	// write out the permutation
	//

	printf("Writing output index %s\n",argv[4]);

	F=fopen(argv[4],"w");
	if (!F)
	{
		printf("Could not create destination index file %s\n",argv[4]);
		return -1;
	}
	
	fprintf(F,"%d\n",(int)vertices.size());
	for(int i=0;i<vertices.size();i++)
		fprintf(F,"%d\n",vGT[vNewIndices[i]]);
		
	fclose(F);
	
	
}