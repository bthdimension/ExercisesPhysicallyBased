#ifndef B_GEOMETRY_DATA_H
#define B_GEOMETRY_DATA_H

#include <vector>
#include "Renderer_GL.h"
#include "vmmlib/vector.hpp"
#include "MaterialData.h"
#include "Texture.h"

struct IndexData
{
    GLushort vertexIndex;
    GLushort texCoordsIndex;
    GLushort normalIndex;
    
    IndexData()
    : vertexIndex(0)
    , texCoordsIndex(0)
    , normalIndex(0)
    {}

	IndexData(GLushort vIndex, GLushort tIndex, GLushort nIndex)
		: vertexIndex(vIndex)
		, texCoordsIndex(tIndex)
		, normalIndex(nIndex)
	{}
};

//struct TexCoord
//{
//    GLfloat s;
//    GLfloat t;
//};
//
//struct Color
//{
//    GLubyte r;
//    GLubyte g;
//    GLubyte b;
//    GLubyte a;
//};
//
//struct Point3
//{
//    GLfloat x;
//    GLfloat y;
//    GLfloat z;
//};
//
//struct Vector3
//{
//    GLfloat x;
//    GLfloat y;
//    GLfloat z;
//};

struct Vertex
{
	Vertex()
	{
		position[0] = 0.0f;
		position[1] = 0.0f;
		position[2] = 0.0f;
		normal[0] = 0.0f;
		normal[1] = 0.0f;
		normal[2] = 0.0f;
		tangent[0] = 0.0f;
		tangent[1] = 0.0f;
		tangent[2] = 0.0f;
		bitangent[0] = 0.0f;
		bitangent[1] = 0.0f;
		bitangent[2] = 0.0f;
		texCoord[0] = 0.0f;
		texCoord[1] = 0.0f;
	}

	Vertex(GLfloat pX, GLfloat pY, GLfloat pZ, GLfloat tS, GLfloat tT)
	{
		position[0] = pX;
		position[1] = pY;
		position[2] = pZ;
		normal[0] = 0.0f;
		normal[1] = 0.0f;
		normal[2] = 0.0f;
		tangent[0] = 0.0f;
		tangent[1] = 0.0f;
		tangent[2] = 0.0f;
		bitangent[0] = 0.0f;
		bitangent[1] = 0.0f;
		bitangent[2] = 0.0f;
		texCoord[0] = tS;
		texCoord[1] = tT;
	}

	Vertex(GLfloat pX, GLfloat pY, GLfloat pZ, GLfloat nX, GLfloat nY, GLfloat nZ, GLfloat tX, GLfloat tY, GLfloat tZ, GLfloat bX, GLfloat bY, GLfloat bZ, GLfloat tS, GLfloat tT)
	{
		position[0] = pX;
		position[1] = pY;
		position[2] = pZ;
		normal[0] = nX;
		normal[1] = nY;
		normal[2] = nZ;
		tangent[0] = tX;
		tangent[1] = tY;
		tangent[2] = tZ;
		bitangent[0] = bX;
		bitangent[1] = bY;
		bitangent[2] = bZ;
		texCoord[0] = tS;
		texCoord[1] = tT;
	}			

	vmml::Vector3f	position;
	vmml::Vector3f	normal;
	vmml::Vector3f	tangent;
	vmml::Vector3f	bitangent;
	vmml::Vector2f	texCoord;
};

typedef GLushort Index;

/** @brief The underlying data of a geometry object.
*	@author David Steiner
*/
struct GeometryData
{
    typedef std::vector< Vertex >   VboVertices;
    typedef std::vector< GLushort > VboIndices;
    
    std::vector< IndexData > indices;
    
    VboVertices vboVertices;
    VboIndices  vboIndices;
    
    MaterialData materialData;
};

typedef std::shared_ptr< GeometryData > GeometryDataPtr;

#endif /* defined(B_GEOMETRY_DATA_H) */
