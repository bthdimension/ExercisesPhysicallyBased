/*
	Contains GJK EPA algorithm based on Lattice3D
	https://bitbucket.org/Hacktank/lattice3d/src/adfb28ffe5b5?at=master

	Contains triangle intersection based on Tomas Möller
	http://fileadmin.cs.lth.se/cs/personal/tomas_akenine-moller/code/

*/

#ifndef B_MESHCOLLIDER_H
#define B_MESHCOLLIDER_H

#include <memory>
#include "Model.h"
#include "headers/ModelData.h"
#include "vmmlib/aabb.hpp"
#include <float.h>

typedef std::vector<vmml::Vector3f>	MeshTriangle;
typedef std::vector<vmml::Vector2f>	MeshTriangle2D;
typedef std::vector<MeshTriangle>	GeometryTriangles;


class IRigidBody;

static vmml::Vector3f getNormal(const vmml::Vector3f &a, const vmml::Vector3f &b, const vmml::Vector3f &c)
{
	vmml::Vector3f n  = ((b - a).cross(c - a));
	n.normalize();
	return n;
}

struct CollisionInformation
{
	bool collisionOccured = false;
	vmml::Vector3f colPoint;
	vmml::Vector3f colNormal;
	float penetratonDepth;
};

struct SupportPoint
{
	vmml::Vector3f a;
	vmml::Vector3f b;
	vmml::Vector3f diff;

	bool operator==(const SupportPoint &p) const { return diff == p.diff; }
};
struct	SupportPointTriangle
{
	SupportPoint points[3];
	vmml::Vector3f normal;

	SupportPointTriangle(const SupportPoint &a, const SupportPoint &b, const SupportPoint &c)
	{
		points[0] = a;
		points[1] = b;
		points[2] = c;
		normal = getNormal(a.diff, b.diff, c.diff);
	}

	SupportPoint at(int i) { return points[i]; }
	vmml::Vector3f atDiff(int i) { return points[i].diff; }
};
typedef std::vector<SupportPointTriangle>	SupportPointTrianglesList;
typedef std::vector<SupportPoint>	SupportPointEdge;
typedef std::vector<SupportPointEdge>	SupportPointEdgesList;

/** @brief A mesh collider can be used to detect collisions between meshes.
*	@author Benjamin Buergisser
*/
class MeshCollider
{
public:

	/* Typedefs */		
	typedef std::unordered_map< std::string, GeometryTriangles >	GeometryTrianglesMap;

	/* Functions */

	/**	@brief Constructor creating a mesh collider from a model
	*	@param[in] model 
	*	@param[in] rigidBody
	*	@param[in] isPerfectSphere Decides wether only a bounding sphere is used for collisions
	*/
	MeshCollider(ModelPtr model, IRigidBody *rigidBody, bool isPerfectSphere = false);

	/**	@brief Constructor creating a mesh collider from  model data containing the geometry data
	*	@param[in] model
	*	@param[in] modelData
	*	@param[in] rigidBody
	*	@param[in] isPerfectSphere Decides wether only a bounding sphere is used for collisions
	*/
	MeshCollider(ModelPtr model, const ModelData &modelData, IRigidBody *rigidBody, bool isPerfectSphere = false);

	/**	@brief Get all polygons of the mesh collider
	*/
	GeometryTrianglesMap &getTriangles() { return _triangles; }

	/**	@brief Get all polygons of a specific part of the mesh collider
	*	@param[in] geometryName
	*/
	GeometryTriangles &getTriangles(std::string geometryName) { return _triangles.at(geometryName); }

	/**	@brief Intersect this collider with another collider
	*	@param[in] meshCollider
	*	@param[in] collisionInformation Will be set by the function if there was an intersection
	*/
	bool doesIntersect(MeshCollider *meshCollider, CollisionInformation *collisionInformation);

	/**	@brief Intersect this collider with another collider based on triangle intersection
	*	@param[in] meshCollider
	*/
	bool doesIntersectTriangleIntersection(MeshCollider *meshCollider);

	/**	@brief Intersect this collider with a single triangle
	*	@param[in] colliderTriangle
	*/
	bool doesIntersect(MeshTriangle &colliderTriangle);

	/**	@brief Intersect this collider's bounding volume with a bounding volume
	*	@param[in] boundingVolumeWorld Bounding volume in world space
	*	@param[in] collisionInformation Will be set by the function if there was an intersection
	*	@param[in] isPerfectSphere Decides wether the passed volume is a bounding sphere or box
	*/
	bool intersectBoundingVolumes(const vmml::AABBf &boundingVolumeWorld, CollisionInformation *collisionInformation, bool isSphere);

	/**	@brief Decides wether only a bounding sphere is used for collisions
	*/
	bool isPerfectSphere() { return _isPerfectSphere; }

	/**	@brief Decides wether only a bounding sphere is used for collisions
	*	@param[in] isPerfectSphere
	*/
	void setIsPerfectSphere(bool isPerfectSphere);

	/**	@brief Returns the bounding volume in object space
	*/
	vmml::AABBf getBoundingVolumeObjectSpace() { return _model->getBoundingBoxObjectSpace(); }

	/**	@brief Returns the bounding volume in world space
	*/
	vmml::AABBf getBoundingVolumeWorldSpace();

	/**	@brief Returns the radius of the bounding volume
	*/
	float getRadiusObjectSpace() { return _radiusObjectSpace; }

	
	bool GJK(MeshCollider *meshCollider, CollisionInformation *collisionInformation);

	bool EPA(MeshCollider *meshCollider, CollisionInformation *collisionInformation, std::vector<SupportPoint> &simplex);
	
	SupportPoint supportFunction(MeshCollider *meshCollider, vmml::Vector3f & direction);
	
	vmml::Vector3f farthestPointInDirection(const vmml::Vector3f & direction);

	bool checkSimplex(std::vector<SupportPoint> &simplex, vmml::Vector3f & direction);

	void addEdge(SupportPointEdgesList &edges, SupportPoint &a, SupportPoint &b);

	bool contactInformation(SupportPointTriangle *triangle, CollisionInformation *collisionInformation);

	std::vector< vmml::Vector3f > getVertices();

	// Static Functions
	static bool doesIntersect(MeshTriangle &triangleWorld1, MeshTriangle &triangleWorld2);
	static bool intersectBoundingBoxes(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2);
	static bool intersectBoundingSpheres(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2, CollisionInformation *collisionInformation);
	//static bool intersectBoundingBoxWithSphere(const vmml::AABBf &box, const vmml::AABBf &sphere);
	//static bool doesIntersectBoundingSphere(const vmml::AABBf &boundingSphere, const MeshTriangle &triangleWorld, bool isPerfectSphere);
	static float getMaxAbsVectorValue(const vmml::Vector3f &vector);


protected:

	/* Functions */

	MeshTriangle getColliderTriangleWorld(const MeshTriangle &triangle);

	void initializeTriangles(ModelPtr model);

	void initializeTrianglesGeometry(std::string geometryName, const GeometryData::VboVertices &vertices, const GeometryData::VboIndices &indices);

private:

	/* Variables */
	IRigidBody *_rigidBody;
	GeometryTrianglesMap _triangles;
	ModelPtr _model;
	bool _isPerfectSphere = false;
	bool _isCollider = true;
	float _radiusObjectSpace = 1.f;
	std::vector< vmml::Vector3f > _vertices;
	const int MAXITER = 80;
};

typedef std::shared_ptr<MeshCollider> MeshColliderPtr;

#endif /* defined(B_MESHCOLLIDER_H) */
