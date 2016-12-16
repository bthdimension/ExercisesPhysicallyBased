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



bool MeshCollider::doesIntersect(MeshCollider *meshCollider, CollisionInformation *collisionInformation)
{
		if (_isPerfectSphere && meshCollider->isPerfectSphere())
		{
			return intersectBoundingVolumes(meshCollider->getBoundingVolumeWorldSpace(), collisionInformation, true);
		}
		else if (_isPerfectSphere)
		{
			// go through all polygons of other mesh collider -> the other collider should do that itself
			return meshCollider->doesIntersect(this, collisionInformation);
		}
		else {
			// we are no perfect sphere, so we go through our polygons to find intersections
			if (intersectBoundingBoxes(getBoundingVolumeWorldSpace(), meshCollider->getBoundingVolumeWorldSpace())) {
				return GJK(meshCollider, collisionInformation);
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

bool MeshCollider::intersectBoundingVolumes(const vmml::AABBf &boundingVolumeWorld, CollisionInformation *collisionInformation, bool isSphere)
{
	if (_isPerfectSphere && isSphere) {
		return intersectBoundingSpheres(getBoundingVolumeWorldSpace(), boundingVolumeWorld, collisionInformation);
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

bool MeshCollider::intersectBoundingSpheres(const vmml::AABBf &boundingVolume1, const vmml::AABBf &boundingVolume2, CollisionInformation *collisionInformation)
{
	float radius1 = boundingVolume1.getMax().x() - boundingVolume1.getCenter().x();
	float radius2 = boundingVolume2.getMax().x() - boundingVolume2.getCenter().x();
	vmml::Vector3f distVector = (boundingVolume2.getCenter() - boundingVolume1.getCenter());
	float distance = distVector.norm();

	if (distance <= (radius1 + radius2)) {
		distVector.normalize();
		collisionInformation->colPoint = distVector*(distance*0.5);
		collisionInformation->penetratonDepth = (radius1 + radius2) - distance;
		collisionInformation->colNormal = distVector;
		collisionInformation->collisionOccured = true;
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


bool MeshCollider::GJK(MeshCollider *meshCollider, CollisionInformation *collisionInformation)
{
	vmml::Vector3f direction = getBoundingVolumeWorldSpace().getCenter() - meshCollider->getBoundingVolumeWorldSpace().getCenter();
	direction.normalize();
	if (direction.norm() == 0) direction = { 1.f };

	std::vector<SupportPoint> simplex;
	simplex.push_back(supportFunction(meshCollider, direction));
	if (simplex[0].diff.dot(direction) <= 0)
		return false;

	direction = -direction;

	int n = 0;
	while (n < MAXITER)
	{
		simplex.push_back(supportFunction(meshCollider, direction));

		if (simplex.back().diff.dot(direction) <= 0)
			return false;
		else if (checkSimplex(simplex, direction)) {
			collisionInformation->collisionOccured = true;
			EPA(meshCollider, collisionInformation, simplex); // find minimum translation vector
			return true;
		}
		n++;
	}

}

void vectorBasis(const vmml::Vector3f &normal, vmml::Vector3f &tangent, vmml::Vector3f &bitangent)
{
	if (normal.x() >= 0.57735f)
		tangent = { normal.y(), -normal.x(), 0.0f };
	else
		tangent = {0.0f, normal.z(), -normal.y()};

	tangent.normalize();
	bitangent = normal.cross(tangent);
}

bool MeshCollider::EPA(MeshCollider *meshCollider, CollisionInformation *collisionInformation, std::vector<SupportPoint> &simplex)
{
	const float THRESH = 0.001f;
	SupportPointEdgesList edges;
	SupportPointTrianglesList triangles;

	// fourth entry [3] is normal!
	triangles.push_back({ simplex[3],simplex[2],simplex[1] });
	triangles.push_back({ simplex[3],simplex[1],simplex[0] });
	triangles.push_back({ simplex[3],simplex[0],simplex[2] });
	triangles.push_back({ simplex[2],simplex[0],simplex[1] });

	int n = 0;
	while (n < MAXITER)
	{
		// find closest face to origin
		auto currentTriangleIt = triangles.begin();
		float distance = FLT_MAX;
		for (auto it = triangles.begin(); it != triangles.end(); it++) {
			float dst = fabs(it->normal.dot( it->atDiff(0) ));
			if (dst < distance) {
				distance = dst;
				currentTriangleIt = it;
			}
		}
		vmml::Vector3f n = currentTriangleIt._Ptr->normal;

		SupportPoint currentSupport = supportFunction(meshCollider,n);
		if (((n.dot( currentSupport.diff )) - distance < THRESH)) {			
			return contactInformation(currentTriangleIt._Ptr, collisionInformation);
		}

		
		for (auto it = triangles.begin(); it != triangles.end(); ) {
			// can face be seen by currentSupport?			
			if (n.dot(currentSupport.diff - it->atDiff(0)) > 0) {
				addEdge(edges, it->at(0) , it->at(1));
				addEdge(edges, it->at(1) , it->at(2));
				addEdge(edges, it->at(2) , it->at(0));
				it = triangles.erase(it);
				continue;
			}
			it++;
		}

		for (auto it = edges.begin(); it != edges.end(); it++) {
			triangles.push_back({ currentSupport,it->at(0),it->at(1) });
		}
		edges.clear();
	}
	return true;
}


SupportPoint MeshCollider::supportFunction(MeshCollider *meshCollider, const vmml::Vector3f & direction)
{
	SupportPoint sup;
	sup.a = farthestPointInDirection(direction);
	sup.b = meshCollider->farthestPointInDirection(-direction);
	sup.diff = sup.a - sup.b;
	return sup;
}

vmml::Vector3f MeshCollider::farthestPointInDirection(const vmml::Vector3f & direction)
{
	// find vertex with with the highest dot product with the direction
	vmml::Vector3f farthest = _rigidBody->getWorldMatrix()*_vertices.at(0);
	vmml::Vector3f directionObjSpace = direction;	// work in object space so we don't have to transform every triangle
	directionObjSpace.normalize();
	float max = directionObjSpace.dot(farthest);

	vmml::Vector3f temp;

	for (int i = 1; i < _vertices.size(); i++)
	{
		temp = _rigidBody->getWorldMatrix()*_vertices.at(i);
		float projection = directionObjSpace.dot(temp);
		if (projection > max)
		{
			farthest = temp;
			max = projection;
		}
	}

	return farthest;
}

bool MeshCollider::checkSimplex(std::vector<SupportPoint> &simplex, vmml::Vector3f & direction)
{
	int simplexSize = simplex.size();

	SupportPoint a = simplex[simplexSize - 1]; // a needs to be newest point for algorithm to work!
	SupportPoint b = simplex[simplexSize  -2];
	vmml::Vector3f aToOrigin = -a.diff;
	vmml::Vector3f ab = b.diff - a.diff;

	if (simplexSize == 4)
	{
		SupportPoint c = simplex[simplexSize - 3];
		SupportPoint d = simplex[simplexSize - 4];
		vmml::Vector3f ac = c.diff - a.diff;
		vmml::Vector3f ad = d.diff - a.diff;


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
		SupportPoint c = simplex[simplexSize - 3];
		vmml::Vector3f ac = c.diff - a.diff;
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

void MeshCollider::addEdge(SupportPointEdgesList &edges, SupportPoint &a, SupportPoint &b)
{
	for (auto it = edges.begin(); it != edges.end(); it++) {
		if (it->at(0) == b && it->at(1) == a) {
			//opposite edge found, remove it and do not add new one
			edges.erase(it);
			return;
		}
	}
	SupportPointEdge edge = { a, b };
	edges.emplace_back(edge);
}

void barycentric(const vmml::Vector3f &p, const vmml::Vector3f &a, const vmml::Vector3f &b, const vmml::Vector3f &c, float *u, float *v, float *w) {
	// code from Crister Erickson's Real-Time Collision Detection
	vmml::Vector3f v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);
	float denom = d00 * d11 - d01 * d01;
	*v = (d11 * d20 - d01 * d21) / denom;
	*w = (d00 * d21 - d01 * d20) / denom;
	*u = 1.0f - *v - *w;
}

bool MeshCollider::contactInformation(SupportPointTriangle *triangle, CollisionInformation *collisionInformation)
{
	float distanceFromOrigin = triangle->normal.dot(triangle->atDiff(0));

	float bary_u, bary_v, bary_w;

	barycentric(triangle->normal * distanceFromOrigin,
		triangle->atDiff(0),
		triangle->atDiff(1),
		triangle->atDiff(2),
		&bary_u,
		&bary_v,
		&bary_w);

	// validation
	if (bary_u != bary_u || bary_v != bary_v || bary_w != bary_w
		|| abs(bary_u) > 1.0f || abs(bary_v) > 1.0f || abs(bary_w) > 1.0f)
		return false;

	collisionInformation->colPoint =			bary_u * triangle->at(0).a +
												bary_v * triangle->at(1).a +
												bary_w * triangle->at(2).a;
	collisionInformation->colNormal =			-triangle->normal;
	collisionInformation->penetratonDepth =		distanceFromOrigin;
	collisionInformation->collisionOccured =	true;
	return true;
}


std::vector< vmml::Vector3f > MeshCollider::getVertices() {
	return _vertices;
}