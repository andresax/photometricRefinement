/** @file PolygonTesselator.h
*
* Very simple polygon tesselator class, current implementation based on glu
* 1.2.
*
* @date 2003/02/17
* @author Jean-Sebastien Franco
*/

#ifndef _PolygonTesselator_h_
#define _PolygonTesselator_h_

#ifdef WIN32
#include <GL/glew.h>
#define CALLAPI __stdcall

#else
#define CALLAPI
#endif

typedef void (CALLAPI* VoidCallback)(void);

#if defined(__APPLE__)&& defined(__MACH__)
//#include <GLUT/glut.h >
//#include <OpenGL/gl.h >
#include <glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif


template <class Pt>
class PolygonTesselator
{
	
	/// override this method for emitted triangles and vertices
	virtual void emitTriangle( int *idx )  = 0;
	virtual void emitVertex( GLdouble *coords )  = 0;

public:
	int	m_noVertices;
	int	m_noNewVertices;	
	
	vector<GLdouble> v_coords;
	/// constructor
	PolygonTesselator(int no_vertices)
		:m_pTess(gluNewTess()),v_coords(no_vertices*3)
{
		m_noVertices= no_vertices;
		m_noNewVertices = 0;
		for(int i=0;i<no_vertices*3;i++) v_coords[i]=0;
		assert(m_pTess);
		gluTessCallback(m_pTess, GLU_TESS_BEGIN_DATA,  (VoidCallback)tessBegin);
		gluTessCallback(m_pTess, GLU_TESS_VERTEX_DATA, (VoidCallback)tessVertex);
		gluTessCallback(m_pTess, GLU_TESS_ERROR,  (VoidCallback)errorCallback);		
		gluTessCallback(m_pTess, GLU_TESS_COMBINE_DATA,  (VoidCallback)combineCallback);		
//		GLdouble* tmpData;
//		gluGetTessProperty(m_pTess,GLU_TESS_TOLERANCE,tmpData);
		
		gluTessProperty(m_pTess,GLU_TESS_TOLERANCE,0);
//		cout << "GLU_TESS_TOLERANCE=" <<*tmpData <<endl;
}

/// begin polygon
inline void beginPolygon()
{
	gluTessBeginPolygon(m_pTess, this);
}

/// end polygon
inline void endPolygon()
{
	gluTessEndPolygon(m_pTess);
}

/// begin contour
inline void beginContour()
{
	gluTessBeginContour(m_pTess);
}

/// end contour
inline void endContour()
{
	gluTessEndContour(m_pTess);
}

inline void vertex(int idx, const Pt& p)
{
	GLdouble *v = &v_coords[3*idx];
	v_coords[3*idx+0]=p[0];
	v_coords[3*idx+1]=p[1];
	v_coords[3*idx+2]=p[2];	
	//GLdouble* dum1 = (GLdouble*)((int)(*m_pVertexPool)[idx].data());
	void *dum2 = (void *) ( idx );
	gluTessVertex(m_pTess, v, dum2);
}

/// destructor
virtual ~PolygonTesselator()
{
	gluDeleteTess(m_pTess);
}

private:

/// GLU tesselator object
GLUtesselator* m_pTess;

/// store indices of points used for current triangle being built
int m_idx[3];

/// number of accumulated vertices <=3
int m_nbVerts;

/// store triangle building mode (GL_TRIANGLES, GL_TRIANGLE_FAN, or
/// GL_TRIANGLE_STRIP)
GLenum m_Tmode;

/// GLU callback for glBegin
static GLvoid CALLAPI tessBegin(GLenum mode, GLvoid *poly_data)
{
	PolygonTesselator* tess = static_cast<PolygonTesselator*>(poly_data);
	tess->m_Tmode=mode; tess->m_nbVerts=0;
}


static GLvoid CALLAPI errorCallback(GLenum errorCode)
{
	const GLubyte *estring;
	estring = gluErrorString(errorCode);
	cerr << "Tessellation error : " << estring << endl;
}


static GLvoid CALLAPI combineCallback(GLdouble coords[3], 
                     void *vertex_data[4],
                     GLfloat weight[4], void **dataOut, GLvoid *poly_data)
{
	cout << "combineCallback called !!!" << endl;
	PolygonTesselator* tess = static_cast<PolygonTesselator*>(poly_data);
	
	GLdouble v[3]; v[0]=0; v[1]=0; v[2]=0;		 
	for(int i=0;i<4;i++) {
		for(int c=0;c<3;c++)
			v[c]+=weight[i]*tess->v_coords[3*((int)vertex_data[i])+c];
		cout << "w[" << i << "]=" << weight[i] << " for id=" << (int)vertex_data[i] << endl;
	}
	
	//add new element to the crowd
	for(int c=0;c<3;c++)
		tess->v_coords.push_back(v[c]);
	tess->m_noNewVertices++;
	
	int new_id = tess->m_noVertices + tess->m_noNewVertices - 1;
	void *dum2 = (void *) (new_id);
	*dataOut = dum2;
	cout << "initial no of vertices" << tess->m_noVertices << endl;
	cout << "additional no of vertices" << tess->m_noNewVertices << endl;	
	cout << "new vertex id=" << new_id << endl;
	tess->emitVertex(&(tess->v_coords[3*new_id]));
}

/// GLU callback for glVertex
static GLvoid CALLAPI tessVertex(GLvoid *vert_data, GLvoid *poly_data)
{
	int idx=(int)vert_data;
	PolygonTesselator* tess = static_cast<PolygonTesselator*>(poly_data);
//	cout << "MODE STRIP = " << tess->m_Tmode <<endl;							
	if (tess->m_nbVerts<3)
	{
		tess->m_idx[tess->m_nbVerts++]=idx;
		if (tess->m_nbVerts==3)
		{
			tess->emitTriangle(&(tess->m_idx[0]));
			if (tess->m_Tmode == GL_TRIANGLES)
				tess->m_nbVerts=0;
		}
	}
	else{
		switch(tess->m_Tmode)
		{
			case GL_TRIANGLE_FAN:
				tess->m_idx[1]=tess->m_idx[2]; tess->m_idx[2]=idx;
				tess->emitTriangle(&(tess->m_idx[0]));
				break;
			case GL_TRIANGLE_STRIP:
				tess->m_nbVerts++;
				if ((tess->m_nbVerts&1)==1)
				{
					tess->m_idx[0]=tess->m_idx[1];
					tess->m_idx[1]=tess->m_idx[2]; tess->m_idx[2]=idx;
					tess->emitTriangle( &(tess->m_idx[0]) );
				}
					else
					{
						tess->m_idx[0] = tess->m_idx[2]; tess->m_idx[2] = idx;
						tess->emitTriangle( &(tess->m_idx[0]) );
						tess->m_idx[1] = tess->m_idx[0];
					}
					break;
			default:
				assert(false);
		}		
	}
}
};

#endif
