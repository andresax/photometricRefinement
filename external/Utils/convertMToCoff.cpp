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
	
	sFace f;
	sVertex v;
	while (fgets(buff,BUFFSIZE,F))
	{
		//printf("%s",buff);
		if (buff[0]=='#') continue;
		if (buff[0]=='V') //vertex
		{
			sscanf(buff,"Vertex %d %f %f %f {wid=%d rgb=(%f %f %f) normal=(%f %f %f) uv=(%f %f)}",&v.id, &v.coord[0], &v.coord[1], &v.coord[2], &v.wid, &v.rgb[0], &v.rgb[1], &v.rgb[2], &v.normal[0], &v.normal[1], &v.normal[2], &v.uv[0], &v.uv[1]);
			//printf("Vertex %d %f %f %f {wid=%d rgb=(%f %f %f) normal=(%f %f %f) uv=(%f %f)\n", v.id, v.coord[0], v.coord[1], v.coord[2], v.wid, v.rgb[0], v.rgb[1], v.rgb[2], v.normal[0], v.normal[1], v.normal[2], v.uv[0], v.uv[1]);
			vertices.push_back(v);
			
		}
		if (buff[0]=='F') //face
		{
			sscanf(buff,"Face %d %d %d %d {matid=%d}",&f.id, &f.v[0], &f.v[1], &f.v[2], &f.matid);
			//printf(buff,"Face %d %d %d %d {matid=%d}",f.id, f.v[0], f.v[1], f.v[2], f.matid);
			faces.push_back(f);
		}
	}

	fclose(F);
	
	F=fopen(argv[2],"w");
	if (!F)
	{
		printf("Could not open destination file %s\n",argv[2]);
		return -1;
	}
	printf("Writing %s\n",argv[2]);
	
	fprintf(F,"COFF\n");
	fprintf(F,"%d %d %d\n",(int)vertices.size(), (int)faces.size(),0);
	for(int i=0;i<vertices.size();i++)
		fprintf(F,"%f %f %f %f %f %f %f\n",vertices[i].coord[0], vertices[i].coord[1], vertices[i].coord[2], vertices[i].rgb[0], vertices[i].rgb[1], vertices[i].rgb[2], 0.8);

	for(int i=0;i<faces.size();i++)
		fprintf(F,"3 %d %d %d\n",faces[i].v[0]-1, faces[i].v[1]-1, faces[i].v[2]-1);

	fclose(F);
}