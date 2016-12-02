#include "headers/MeshCollider.h"
#include "headers/IRigidBody.h"

MeshCollider::MeshCollider(ModelPtr model, IRigidBody *rigidBody, bool isPerfectSphere)
{
	_model = model;
	_rigidBody = rigidBody;
	initializeTriangles(model);

	setIsPerfectSphere(isPerfectSphere);
}

MeshCollider::MeshCollider(ModelPtr model, const ModelData &modelData, IRigidBody *rigidBody, bool isPerfectSphere)
{
	_model = model;
	_rigidBody = rigidBody;
	ModelData::GroupMap data = modelData.getData();

	for (auto i = data.begin(); i != data.end(); ++i)
	{
		GeometryDataPtr gData = i->second;
		initializeTrianglesGeometry(i->first, gData->vboVertices, gData->vboIndices);
	}

	setIsPerfectSphere(isPerfectSphere);
}

void MeshCollider::initializeTriangles(ModelPtr model)
{
	if (model && model->getGroups().size() > 0)
	{
		for (auto i = model->getGroups().begin(); i != model->getGroups().end(); ++i)
		{
			GeometryData::VboVertices v_vector;
			GeometryData::VboIndices i_vector;

			GeometryPtr geometry = i->second;
			Geometry::VertexDataPtr vertices = geometry->getVertexData();
			Geometry::IndexDataPtr indices = geometry->getIndexData();
			size_t numVert = geometry->getNumVertices();
			size_t numInd = geometry->getNumIndices();


			for (int n = 0; n < numVert; n++) {
				v_vector.push_back(*(vertices.get() + n));
			}
			for (int n = 0; n < numInd; n++) {
				i_vector.push_back(*(indices.get() + n));
			}

			initializeTrianglesGeometry(i->first, v_vector, i_vector);
		}
	}
}

void MeshCollider::initializeTrianglesGeometry(std::string geometryName, GeometryData::VboVertices vertices, GeometryData::VboIndices indices)
{
	MeshTriangle currentTriangle;
	GeometryTriangles &triangles = _triangles[geometryName];

	int numOfTriangles = indices.size() / 3;

	for (int i = 0; i != numOfTriangles; ++i)
	{
		triangles.push_back({ vertices[indices[i * 3 + 0]].position, vertices[indices[i * 3 + 1]].position, vertices[indices[i * 3 + 2]].position });
	}
}

bool MeshCollider::doesIntersect(MeshCollider *meshCollider)
{
		if (_isPerfectSphere && meshCollider->isPerfectSphere())
		{
			return intersectBoundingVolumes(meshCollider->getBoundingVolumeWorldSpace(), true);
		}
		else if (_isPerfectSphere)
		{
			// go through all polygons of other mesh collider -> the other collider should do that itself
			return meshCollider->doesIntersect(this);
		}
		else {
			// we are no perfect sphere, so we go through our polygons to find intersections
			if (intersectBoundingBoxes(getBoundingVolumeWorldSpace(), meshCollider->getBoundingVolumeWorldSpace())) {
				for (auto i = _triangles.begin(); i != _triangles.end(); ++i) {
					for (auto t = i->second.begin(); t != i->second.end(); ++t)
						// see if our polygon intersects the other collider (other collider could also be perfect sphere and just collide with that)
						if (meshCollider->doesIntersect(getColliderTriangleWorld(*t)))
							return true;
					return false;
				}
			}
		}
		return false;
}

bool MeshCollider::doesIntersect(MeshTriangle colliderTriangle)
{
	// TODO
	/*if (_isPerfectSphere) {
		return doesIntersectBoundingSphere(getBoundingBoxWorldSpace(), colliderTriangle, true);
	}
	else*/ {
		for (auto i = _triangles.begin(); i != _triangles.end(); ++i) {
			for (auto t = i->second.begin(); t != i->second.end(); ++t)
				if (doesIntersect(getColliderTriangleWorld(*t), colliderTriangle))
					return true;
		}
	}
	return false;
}

bool MeshCollider::intersectBoundingVolumes(vmml::AABBf boundingVolumeWorld, bool isSphere)
{
	if (_isPerfectSphere && isSphere) {
		return intersectBoundingSpheres(boundingVolumeWorld, getBoundingVolumeWorldSpace());
	}
	else if (_isPerfectSphere) {
		return intersectBoundingBoxWithSphere(boundingVolumeWorld, getBoundingVolumeWorldSpace());
	}
	else if (isSphere) {
		return intersectBoundingBoxWithSphere(getBoundingVolumeWorldSpace(), boundingVolumeWorld);
	}
	else {
		return intersectBoundingBoxes(getBoundingVolumeWorldSpace(), boundingVolumeWorld);
	}
	return false;
}

void MeshCollider::setIsPerfectSphere(bool isPerfectSphere) {
	_isPerfectSphere = isPerfectSphere;
	if (_isPerfectSphere) {
		_radiusObjectSpace = getBoundingVolumeObjectSpace().getMax().x() - getBoundingVolumeObjectSpace().getCenter().x();
	}
	else {
		_radiusObjectSpace = 0.5*getBoundingVolumeObjectSpace().getDimension().norm();
	}
}

MeshTriangle MeshCollider::getColliderTriangleWorld(MeshTriangle triangle)
{
	vmml::Matrix4f world = _rigidBody->getWorldMatrix();
	return MeshTriangle({world*triangle[0], world*triangle[1], world*triangle[2]});
}

vmml::AABBf MeshCollider::getBoundingVolumeWorldSpace()
{
	vmml::AABBf boundingVolumeWorld = getBoundingVolumeObjectSpace();
	if (_isPerfectSphere)
	{		
		vmml::Vector3f center = boundingVolumeWorld.getCenter() + _rigidBody->getPosition() * _rigidBody->getScale();
		float radius = getRadiusObjectSpace() *  getMaxAbsVectorValue(_rigidBody->getScale());
		boundingVolumeWorld = vmml::AABBf(vmml::Vector4f(center, radius));
	}
	else
	{
		const int numVertices = 8;
		vmml::Vector3f vertices[numVertices];
		// create all 8 vertices of the axis aligned bounding box
		vertices[0] = boundingVolumeWorld.getMax();
		vertices[1][0] = boundingVolumeWorld.getMax().x(); vertices[1][1] = boundingVolumeWorld.getMax().y(); vertices[1][2] = boundingVolumeWorld.getMin().z();
		vertices[2][0] = boundingVolumeWorld.getMax().x(); vertices[2][1] = boundingVolumeWorld.getMin().y(); vertices[2][2] = boundingVolumeWorld.getMax().z();
		vertices[3][0] = boundingVolumeWorld.getMin().x(); vertices[3][1] = boundingVolumeWorld.getMax().y(); vertices[3][2] = boundingVolumeWorld.getMax().z();
		vertices[4][0] = boundingVolumeWorld.getMax().x(); vertices[4][1] = boundingVolumeWorld.getMin().y(); vertices[4][2] = boundingVolumeWorld.getMin().z();
		vertices[5][0] = boundingVolumeWorld.getMin().x(); vertices[5][1] = boundingVolumeWorld.getMax().y(); vertices[5][2] = boundingVolumeWorld.getMin().z();
		vertices[6][0] = boundingVolumeWorld.getMin().x(); vertices[6][1] = boundingVolumeWorld.getMin().y(); vertices[6][2] = boundingVolumeWorld.getMax().z();
		vertices[7] = boundingVolumeWorld.getMin();
		
		// multiply all vertices by world matrix
		vmml::Matrix4f world = _rigidBody->getWorldMatrix();
		for (int i = 0; i < numVertices; i++)
			vertices[i] = world * vertices[i];

		
		vmml::Vector3f min = vertices[0];
		vmml::Vector3f max = vertices[0];
		for (int i = 0; i < numVertices; i++)
		{
			if (vertices[i][0] < min[0])
				min[0] = vertices[i][0];
			else if (vertices[i][0] > max[0])
				max[0] = vertices[i][0];

			if (vertices[i][1] < min[1])
				min[1] = vertices[i][1];
			else if (vertices[i][1] > max[1])
				max[1] = vertices[i][1];

			if (vertices[i][2] < min[2])
				min[2] = vertices[i][2];
			else if (vertices[i][2] > max[2])
				max[2] = vertices[i][2];
		}

		boundingVolumeWorld = vmml::AABBf(min, max);
		
	}
	return boundingVolumeWorld;
}

// Static Functions

bool MeshCollider::intersectBoundingBoxes(vmml::AABBf boundingVolume1, vmml::AABBf boundingVolume2)
{
	return
		(boundingVolume1.getMin().x() <= boundingVolume2.getMax().x() && boundingVolume1.getMax().x() >= boundingVolume2.getMin().x())
		&& (boundingVolume1.getMin().y() <= boundingVolume2.getMax().y() && boundingVolume1.getMax().y() >= boundingVolume2.getMin().y())
		&& (boundingVolume1.getMin().z() <= boundingVolume2.getMax().z() && boundingVolume1.getMax().z() >= boundingVolume2.getMin().z());
}

bool MeshCollider::intersectBoundingSpheres(vmml::AABBf boundingVolume1, vmml::AABBf boundingVolume2)
{
	float radius1 = boundingVolume1.getMax().x() - boundingVolume1.getCenter().x();
	float radius2 = boundingVolume2.getMax().x() - boundingVolume2.getCenter().x();
	float distance = (boundingVolume1.getCenter() - boundingVolume2.getCenter()).norm();

	if (distance <= (radius1 + radius2))
		return true;

	return false;
}

bool MeshCollider::intersectBoundingBoxWithSphere(vmml::AABBf box, vmml::AABBf sphere)
{
	//float radius = sphere.getMax().x() - sphere.getCenter().x();

	//float distance = (sphere.getCenter() - box.getCenter()).norm();
	//float radiusBox = box.getDimension().norm()*0.5;
	//float radiusSphere = sphere.getDimension().norm()*0.5;

	return intersectBoundingSpheres(box, sphere);
}

bool MeshCollider::doesIntersect(MeshTriangle colliderTriangleWorld1, MeshTriangle colliderTriangleWorld2)
{
	//**** distances of vertices of triangle 1 to plane of triangle 2 ****************

	vmml::Vector3f normal2 = (colliderTriangleWorld2[1] - colliderTriangleWorld2[0]).cross(colliderTriangleWorld2[2] - colliderTriangleWorld2[0]);
	float d2 = (-normal2).dot(colliderTriangleWorld2[0]);

	float distance1_1 = getVertexDistanceToPlane(colliderTriangleWorld1[0], normal2, d2);
	float distance1_2 = getVertexDistanceToPlane(colliderTriangleWorld1[1], normal2, d2);
	float distance1_3 = getVertexDistanceToPlane(colliderTriangleWorld1[2], normal2, d2);

	// if distances are all non-zero (no vertex lies on the plane) and all signs are the same (all vertices on one side of the plane) we have no intersection
	if ((distance1_1 > 0 && distance1_2 > 0 && distance1_3 > 0) ||
		(distance1_1 < 0 && distance1_2 < 0 && distance1_3 < 0))
		return false;
	//********************************************************************************

	//**** distances of vertices of triangle 2 to plane of triangle 1 ****************
	vmml::Vector3f normal1 = (colliderTriangleWorld1[1] - colliderTriangleWorld1[0]).cross(colliderTriangleWorld1[2] - colliderTriangleWorld1[0]);
	float d1 = (-normal1).dot(colliderTriangleWorld1[0]);

	float distance2_1 = getVertexDistanceToPlane(colliderTriangleWorld2[0], normal1, d1);
	float distance2_2 = getVertexDistanceToPlane(colliderTriangleWorld2[1], normal1, d1);
	float distance2_3 = getVertexDistanceToPlane(colliderTriangleWorld2[2], normal1, d1);

	// if distances are all non-zero (no vertex lies on the plane) and all signs are the same (all vertices on one side of the plane) we have no intersection
	if ((distance2_1 > 0 && distance2_2 > 0 && distance2_3 > 0) ||
		(distance2_1 < 0 && distance2_2 < 0 && distance2_3 < 0))
		return false;
	//********************************************************************************

	// If not coplanar, the two planes intersect in a line L = O + tD where D = normal1 x normal 2 and O is some point on it
	// Both triangles are guranteed to interect L after passing the tests above
	if (distance1_1 != 0 && distance1_2 != 0 && distance1_3 != 0)
	{
		float t1_1 = 0.f, t1_2 = 0.f; // line parameter values for triangle 1
		if ((distance1_1 > 0 && distance1_2 < 0) || (distance1_1 < 0 && distance1_2 > 0)) // vert 1 and 2 lie on opposite sides of L
			t1_1 = getLineParameterValue(colliderTriangleWorld1[0], distance1_1, colliderTriangleWorld1[1], distance1_2, normal1, normal2);
		else if ((distance1_1 > 0 && distance1_3 < 0) || (distance1_1 < 0 && distance1_3 > 0)) // vert 1 and 3 lie on opposite sides of L
			t1_1 = getLineParameterValue(colliderTriangleWorld1[0], distance1_1, colliderTriangleWorld1[2], distance1_3, normal1, normal2);

		if ((distance1_3 > 0 && distance1_2 < 0) || (distance1_3 < 0 && distance1_2 > 0)) // vert 2 and 3 lie on opposite sides of L
			t1_2 = getLineParameterValue(colliderTriangleWorld1[1], distance1_2, colliderTriangleWorld1[2], distance1_3, normal1, normal2);
		else if ((distance1_1 > 0 && distance1_3 < 0) || (distance1_1 < 0 && distance1_3 > 0)) // vert 1 and 3 lie on opposite sides of L
			t1_2 = getLineParameterValue(colliderTriangleWorld1[0], distance1_1, colliderTriangleWorld1[2], distance1_3, normal1, normal2);

		float t2_1 = 0.f, t2_2 = 0.f; // line parameter values for triangle 2
		if ((distance2_1 > 0 && distance2_2 < 0) || (distance2_1 < 0 && distance2_2 > 0)) // vert 1 and 2 lie on opposite sides of L
			t2_1 = getLineParameterValue(colliderTriangleWorld2[0], distance2_1, colliderTriangleWorld2[1], distance2_2, normal1, normal2);
		else if ((distance2_1 > 0 && distance2_3 < 0) || (distance2_1 < 0 && distance2_3 > 0)) // vert 1 and 3 lie on opposite sides of L
			t2_1 = getLineParameterValue(colliderTriangleWorld2[0], distance2_1, colliderTriangleWorld2[2], distance2_3, normal1, normal2);

		if ((distance2_3 > 0 && distance2_2 < 0) || (distance2_3 < 0 && distance2_2 > 0)) // vert 2 and 3 lie on opposite sides of L     
			t2_2 = getLineParameterValue(colliderTriangleWorld2[1], distance2_2, colliderTriangleWorld2[2], distance2_3, normal1, normal2);
		else if ((distance2_1 > 0 && distance2_3 < 0) || (distance2_1 < 0 && distance2_3 > 0)) // vert 1 and 3 lie on opposite sides of L      
			t2_2 = getLineParameterValue(colliderTriangleWorld2[0], distance2_1, colliderTriangleWorld2[2], distance2_3, normal1, normal2);

		// interval 1 includes interval 2 or the other way around
		if ((t1_1 > t2_1 && t1_2 < t2_2) || (t1_1 < t2_1 && t1_2 > t2_2))
			return true;
		else if ((t1_1 > t2_2 && t1_2 < t2_1) || (t1_1 < t2_2 && t1_2 > t2_1))
			return true;

		else if ((t1_1 < t2_2 && t1_1 > t2_1) || (t1_1 > t2_2 && t1_1 < t2_1)) // t1_1 is in between t2_1 and t2_2
			return true;
		else if ((t1_2 < t2_2 && t1_2 > t2_1) || (t1_2 > t2_2 && t1_2 < t2_1)) // t1_2 is in between t2_1 and t2_2
			return true;

	}
	else // coplanar
	{
		// not really necessary because happens not that often. will leave it out for performance reasons
	}

	return false;
}

bool MeshCollider::doesIntersectBoundingSphere(vmml::AABBf boundingSphere, MeshTriangle triangleWorld, bool isPerfectSphere)
{
	// TODO
	return false;
}

float MeshCollider::getMaxAbsVectorValue(vmml::Vector3f vector)
{
	float max = abs(vector.x());
	if (abs(vector.y()) > max) max = abs(vector.y());
	if (abs(vector.z()) > max) max = abs(vector.z());
	return max;
}

float MeshCollider::getVertexDistanceToPlane(vmml::Vector3f vertexToTest, vmml::Vector3f planeNormal, float d)
{
	return planeNormal.dot(vertexToTest) + d;
}

float MeshCollider::getLineParameterValue(vmml::Vector3f vertOnSide1, float distance1, vmml::Vector3f vertOnSide2, float distance2, vmml::Vector3f normalPlane1, vmml::Vector3f normalPlane2)
{
	vmml::Vector3f D = normalPlane1.cross(normalPlane2);
	float p1 = D.dot(vertOnSide1);
	float p2 = D.dot(vertOnSide2);

	return p1 + (p2 - p1) * (distance1 / (distance1 - distance2));
}