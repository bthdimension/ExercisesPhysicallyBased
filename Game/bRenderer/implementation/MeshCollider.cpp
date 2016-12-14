#include "headers/MeshCollider.h"
#include "headers/IRigidBody.h"
#include "headers/CollisionFunctions.h"
#include "headers/Logger.h"

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
			GeometryData::VboIndices i_vector;
			GeometryData::VboVertices v_vector;

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

			for (int n = 0; n < numVert; n++) {
				if (std::find(_vertices.begin(), _vertices.end(), v_vector[n].position) == _vertices.end())
					_vertices.push_back(v_vector[n].position);
			}

		}
	}
}

void MeshCollider::initializeTrianglesGeometry(std::string geometryName, const GeometryData::VboVertices &vertices, const GeometryData::VboIndices &indices)
{
	MeshTriangle currentTriangle;
	GeometryTriangles &triangles = _triangles[geometryName];

	int numOfTriangles = indices.size() / 3;

	for (int i = 0; i != numOfTriangles; ++i)
	{
		triangles.push_back({ vertices[indices[i * 3 + 0]].position, vertices[indices[i * 3 + 1]].position, vertices[indices[i * 3 + 2]].position });
	}
}



bool MeshCollider::doesIntersect(MeshCollider *meshCollider, vmml::Vector3f *minimumTranslationVector)
{
		if (_isPerfectSphere && meshCollider->isPerfectSphere())
		{
			return intersectBoundingVolumes(meshCollider->getBoundingVolumeWorldSpace(), minimumTranslationVector, true);
		}
		else if (_isPerfectSphere)
		{
			// go through all polygons of other mesh collider -> the other collider should do that itself
			return meshCollider->doesIntersect(this, minimumTranslationVector);
		}
		else {
			// we are no perfect sphere, so we go through our polygons to find intersections
			if (intersectBoundingBoxes(getBoundingVolumeWorldSpace(), meshCollider->getBoundingVolumeWorldSpace())) {
				return GJK(meshCollider, minimumTranslationVector);
			}
		}
		return false;
}

bool MeshCollider::doesIntersectTriangleIntersection(MeshCollider *meshCollider)
{
		if (_isPerfectSphere && meshCollider->isPerfectSphere())
		{
			return intersectBoundingVolumes(meshCollider->getBoundingVolumeWorldSpace(), nullptr, true);
		}
		else if (_isPerfectSphere)
		{
			// go through all polygons of other mesh collider -> the other collider should do that itself
			return meshCollider->doesIntersect(this, nullptr);
		}
		else {
			// we are no perfect sphere, so we go through our polygons to find intersections
			if (intersectBoundingBoxes(getBoundingVolumeWorldSpace(), meshCollider->getBoundingVolumeWorldSpace())) {
				for (auto i = _triangles.begin(); i != _triangles.end(); ++i) {
					for (auto t = i->second.begin(); t != i->second.end(); ++t)
						// see if our polygon intersects the other collider (other collider could also be perfect sphere and just collide with that)
						if (meshCollider->doesIntersect(getColliderTriangleWorld(*t)))
							return true;
				}
			}
		}
		return false;
}

bool MeshCollider::doesIntersect(MeshTriangle &colliderTriangle)
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

bool MeshCollider::intersectBoundingVolumes(const vmml::AABBf &boundingVolumeWorld, vmml::Vector3f *minimumTranslationVector, bool isSphere)
{
	if (_isPerfectSphere && isSphere) {
		return intersectBoundingSpheres(boundingVolumeWorld, getBoundingVolumeWorldSpace(), minimumTranslationVector);
	}
	//else if (_isPerfectSphere) {
	//	return intersectBoundingBoxWithSphere(boundingVolumeWorld, getBoundingVolumeWorldSpace());
	//}
	//else if (isSphere) {
	//	return intersectBoundingBoxWithSphere(getBoundingVolumeWorldSpace(), boundingVolumeWorld);
	//}
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

MeshTriangle MeshCollider::getColliderTriangleWorld(const MeshTriangle &triangle)
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

bool MeshCollider::intersectBoundingBoxes(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2)
{
	return
		(boundingVolume1.getMin().x() <= boundingVolume2.getMax().x() && boundingVolume1.getMax().x() >= boundingVolume2.getMin().x())
		&& (boundingVolume1.getMin().y() <= boundingVolume2.getMax().y() && boundingVolume1.getMax().y() >= boundingVolume2.getMin().y())
		&& (boundingVolume1.getMin().z() <= boundingVolume2.getMax().z() && boundingVolume1.getMax().z() >= boundingVolume2.getMin().z());
}

bool MeshCollider::intersectBoundingSpheres(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2, vmml::Vector3f *minimumTranslationVector)
{
	float radius1 = boundingVolume1.getMax().x() - boundingVolume1.getCenter().x();
	float radius2 = boundingVolume2.getMax().x() - boundingVolume2.getCenter().x();
	vmml::Vector3f distVector = (boundingVolume1.getCenter() - boundingVolume2.getCenter());
	float distance = distVector.norm();

	if (distance <= (radius1 + radius2)) {
		distVector.normalize();
		minimumTranslationVector = &(distVector*(radius1 + radius2) - distance);
		return true;
	}

	return false;
}

//bool MeshCollider::intersectBoundingBoxWithSphere(const vmml::AABBf &box, const vmml::AABBf &sphere)
//{
//	//float radius = sphere.getMax().x() - sphere.getCenter().x();
//
//	//float distance = (sphere.getCenter() - box.getCenter()).norm();
//	//float radiusBox = box.getDimension().norm()*0.5;
//	//float radiusSphere = sphere.getDimension().norm()*0.5;
//
//	return intersectBoundingSpheres(box, sphere);
//}

bool MeshCollider::doesIntersect(MeshTriangle &colliderTriangleWorld1, MeshTriangle &colliderTriangleWorld2)
{
	if (NoDivTriTriIsect(colliderTriangleWorld1[0], colliderTriangleWorld1[1], colliderTriangleWorld1[2],
		colliderTriangleWorld2[0], colliderTriangleWorld2[1], colliderTriangleWorld2[2]))
	{
		//std::cout << "triangle intersect" << std::endl;
		return true;
	}
	return false;
}

//bool MeshCollider::doesIntersectBoundingSphere(const vmml::AABBf &boundingSphere, const MeshTriangle &triangleWorld, bool isPerfectSphere)
//{
//	// TODO
//	return false;
//}

float MeshCollider::getMaxAbsVectorValue(const vmml::Vector3f &vector)
{
	float max = abs(vector.x());
	if (abs(vector.y()) > max) max = abs(vector.y());
	if (abs(vector.z()) > max) max = abs(vector.z());
	return max;
}


bool MeshCollider::GJK(MeshCollider *meshCollider, vmml::Vector3f *minimumTranslationVector)
{
	vmml::Vector3f direction = getBoundingVolumeWorldSpace().getCenter() - meshCollider->getBoundingVolumeWorldSpace().getCenter();
	direction.normalize();
	if (direction.norm() == 0) direction = { 1.f };

	std::vector<vmml::Vector3f> simplex;
	simplex.push_back(supportFunction(meshCollider, direction));
	if (simplex[0].dot(direction) <= 0)
		return false;

	direction = -direction;

	int n = 0;
	while (n < MAXITER)
	{
		simplex.push_back(supportFunction(meshCollider, direction));

		if (simplex.back().dot(direction) <= 0)
			return false;
		else if (checkSimplex(simplex, direction))
			return true;
		n++;
	}

}



vmml::Vector3f MeshCollider::supportFunction(MeshCollider *meshCollider, const vmml::Vector3f & direction)
{
	vmml::Vector3f a = farthestPointInDirection(direction);
	vmml::Vector3f b = meshCollider->farthestPointInDirection(-direction);
	return a - b;
}

vmml::Vector3f MeshCollider::farthestPointInDirection(const vmml::Vector3f & direction)
{
	// find vertex with with the highest dot product with the direction
	vmml::Vector3f farthest = _vertices.at(0);
	vmml::Vector3f directionObjSpace = _rigidBody->getInverseWorldMatrix() * direction;	// work in object space so we don't have to transform every triangle
	directionObjSpace.normalize();
	float max = directionObjSpace.dot(_vertices.at(0));

	for (int i = 1; i < _vertices.size(); i++)
	{
		float projection = directionObjSpace.dot(_vertices.at(i));
		if (projection > max)
		{
			farthest = _vertices.at(i);
			max = projection;
		}
	}

	farthest = _rigidBody->getWorldMatrix() * farthest; // transform vertex

	return farthest;
}

bool MeshCollider::checkSimplex(std::vector<vmml::Vector3f> &simplex, vmml::Vector3f & direction)
{
	int simplexSize = simplex.size();

	vmml::Vector3f a = simplex[simplexSize - 1]; // a needs to be newest point for algorithm to work!
	vmml::Vector3f b = simplex[simplexSize  -2];
	vmml::Vector3f aToOrigin = -a;
	vmml::Vector3f ab = b - a;

	if (simplexSize == 4)
	{
		vmml::Vector3f c = simplex[simplexSize - 3];
		vmml::Vector3f d = simplex[simplexSize - 4];
		vmml::Vector3f ac = c - a;
		vmml::Vector3f ad = d - a;


		if ((ab.cross(ac)).dot(aToOrigin) > 0)
		{
			// do triangle stuff
		}
		else if ((ac.cross(ad)).dot(aToOrigin) > 0)
		{
			// origin is in front of triangle acd
			simplex = { d,c,a };
		}
		else if ((ad.cross(ab)).dot(aToOrigin) > 0)
		{
			// origin is in front of triangle adb
			simplex = { b,d,a };
		}
		else {
			// origin within
			return true;
		}
	}

	// here not else if since we may just have set a 3 point simplex above
	if (simplexSize >= 3) // triangle
	{
		vmml::Vector3f c = simplex[simplexSize - 3];
		vmml::Vector3f ac = c - a;
		vmml::Vector3f abc = ab.cross(ac);
		if ((abc.cross(ac)).dot(aToOrigin) > 0) // origin near ac edge outside trianle
		{
			if (ac.dot(aToOrigin) > 0) {
				simplex = { c, a };
				direction = ac.cross(aToOrigin).cross(ac);
			}
			else if (ab.dot(aToOrigin) > 0) {
				simplex = { b, a };
				direction = ab.cross(aToOrigin).cross(ab);
			}
			else {
				simplex = { a };
				direction = aToOrigin;
			}
		}
		else if ((ab.cross(abc)).dot(aToOrigin) > 0) // origin near ab edge outside trianle
		{
			if (ab.dot(aToOrigin) > 0) {
				simplex = { b, a };
				direction = ab.cross(aToOrigin).cross(ab);
			}
			else {
				simplex = { a };
				direction = aToOrigin;
			}
			
		}
		else if (abc.dot(aToOrigin)> 0) // origin above triangle
		{
			direction = abc;
		}
		else { // origin below triangle
			simplex = { b,c,a };
			direction = -abc;
		}
	}
	else // line
	{		
		if (ab.dot(aToOrigin) > 0) // edge closest to origin
			direction = ab.cross(aToOrigin).cross(ab);
		else { // point closest to origin
			simplex = { a };
			direction = aToOrigin;
		}
	}
	direction.normalize();
	return false;

}