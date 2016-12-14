#ifndef B_MESHCOLLIDER_H
#define B_MESHCOLLIDER_H

#include <memory>
#include "Model.h"
#include "headers/ModelData.h"
#include "vmmlib/aabb.hpp"

typedef std::vector<vmml::Vector3f>	MeshTriangle;
typedef std::vector<vmml::Vector2f>	MeshTriangle2D;
class IRigidBody;

/** @brief A mesh collider can be used to detect collisions between meshes.
*	@author Benjamin Buergisser
*/
class MeshCollider
{
public:

	/* Typedefs */	
	typedef std::vector<MeshTriangle>	GeometryTriangles;
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
	*	@param[in] minimumTranslationVector Will be set by the function if there was an intersection
	*/
	bool doesIntersect(MeshCollider *meshCollider, vmml::Vector3f *minimumTranslationVector);

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
	*	@param[in] minimumTranslationVector Will be set by the function if there was an intersection
	*	@param[in] isPerfectSphere Decides wether the passed volume is a bounding sphere or box
	*/
	bool intersectBoundingVolumes(const vmml::AABBf &boundingVolumeWorld, vmml::Vector3f *minimumTranslationVector, bool isSphere);

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

	
	bool GJK(MeshCollider *meshCollider, vmml::Vector3f *minimumTranslationVector);
	
	vmml::Vector3f supportFunction(MeshCollider *meshCollider, const vmml::Vector3f & direction);
	
	vmml::Vector3f farthestPointInDirection(const vmml::Vector3f & direction);

	bool checkSimplex(std::vector<vmml::Vector3f> &simplex, vmml::Vector3f & direction);



	// Static Functions
	static bool doesIntersect(MeshTriangle &triangleWorld1, MeshTriangle &triangleWorld2);
	static bool intersectBoundingBoxes(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2);
	static bool intersectBoundingSpheres(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2, vmml::Vector3f *minimumTranslationVector);
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
