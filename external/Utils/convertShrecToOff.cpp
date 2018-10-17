#include <stdio.h>
#include <vector>

typedef struct {
	int id;
	float coord[3];
	int wid;
	float rgb[3];
	float normal[3];
	float uv[2];	
} sVertex;

typedef struct {
	int id;
	int v[3];
	int matid;
} sFace;

std::vector<sVertex> vertices;
std::vector<sFace> faces;
#define BUFFSIZE 20000

int main(int argc, char**argv)
{
	
	FILE *F;
	
	if (argc==0)
	{
		printf(" input file in m format required\n");
		return 0;
	}
	F=fopen(argv[1],"r");
	if (!F)
	{
		printf("Could not read source file %s\n",argv[1]);
		return -1;
	}
	printf("Reading %s\n",argv[1]);
	
	char buff[BUFFSIZE];
	int noVertices;
	int noFacets;	
	int noSomethingElse;		

	// read in the input
	
	fscanf(F,"%s\n",buff);
	fscanf(F,"%d %d %d\n",&noVertices,&noFacets,&noSomethingElse);	
	
	sVertex v;
	for(int i=0;i<noVertices;i++)
	{
		fscanf(F, "%f %f %f\n", &v.coord[0], &v.coord[1], &v.coord[2]);
			vertices.push_back(v);
	}
			
	sFace f;
	for(int i=0;i<noFacets;i++)
	{
		fscanf(F, "%d %d %d\n", &f.v[0], &f.v[1], &f.v[2]);
			faces.push_back(f);
	}
	fclose(F);
	
	F=fopen(argv[2],"w");
	if (!F)
	{
		printf("Could not open destination file %s\n",argv[2]);
		return -1;
	}
	printf("Writing %s\n",argv[2]);
	
	// write out the output
	fprintf(F,"OFF\n");
	fprintf(F,"%d %d %d\n",(int)vertices.size(), (int)faces.size(),0);
	for(int i=0;i<vertices.size();i++)
		fprintf(F,"%f %f %f\n",vertices[i].coord[0], vertices[i].coord[1], vertices[i].coord[2]);

	for(int i=0;i<faces.size();i++)
		fprintf(F,"3 %d %d %d\n",faces[i].v[0]-1, faces[i].v[1]-1, faces[i].v[2]-1);

	fclose(F);
}