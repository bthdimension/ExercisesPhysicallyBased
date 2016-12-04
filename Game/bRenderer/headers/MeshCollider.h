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
	*/
	bool doesIntersect(MeshCollider *meshCollider);

	/**	@brief Intersect this collider with a single triangle
	*	@param[in] colliderTriangle
	*/
	bool doesIntersect(MeshTriangle colliderTriangle);

	/**	@brief Intersect this collider's bounding volume with a bounding volume
	*	@param[in] boundingVolumeWorld Bounding volume in world space
	*	@param[in] isPerfectSphere Decides wether the passed volume is a bounding sphere or box
	*/
	bool intersectBoundingVolumes(vmml::AABBf boundingVolumeWorld, bool isSphere);

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


	// Static Functions
	static bool doesIntersect(MeshTriangle triangleWorld1, MeshTriangle triangleWorld2);
	//static bool doesIntersect(MeshTriangle2D triangleWorld1, MeshTriangle2D triangleWorld2);
	static bool intersectBoundingBoxes(vmml::AABBf boundingVolume1, vmml::AABBf boundingVolume2);
	static bool intersectBoundingSpheres(vmml::AABBf boundingVolume1, vmml::AABBf boundingVolume2);
	static bool intersectBoundingBoxWithSphere(vmml::AABBf box, vmml::AABBf sphere);
	static bool doesIntersectBoundingSphere(vmml::AABBf boundingSphere, MeshTriangle triangleWorld, bool isPerfectSphere);
	static float getMaxAbsVectorValue(vmml::Vector3f vector);
	static float getVertexDistanceToPlane(vmml::Vector3f vertexToTest, vmml::Vector3f planeNormal, float d);
	static float getLineParameterValue(vmml::Vector3f vertOnSide1, float distance1, vmml::Vector3f vertOnSide2, float distance2, vmml::Vector3f normalPlane1, vmml::Vector3f normalPlane2);

protected:

	/* Functions */

	MeshTriangle getColliderTriangleWorld(MeshTriangle triangle);

	void initializeTriangles(ModelPtr model);

	void initializeTrianglesGeometry(std::string geometryName, GeometryData::VboVertices vertices, GeometryData::VboIndices indices);	

private:

	/* Variables */
	IRigidBody *_rigidBody;
	GeometryTrianglesMap _triangles;
	ModelPtr _model;
	bool _isPerfectSphere = false;
	bool _isCollider = true;
	float _radiusObjectSpace = 1.f;
};

typedef std::shared_ptr<MeshCollider> MeshColliderPtr;

#endif /* defined(B_MESHCOLLIDER_H) */
