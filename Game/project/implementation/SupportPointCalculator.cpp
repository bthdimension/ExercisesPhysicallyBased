#include "SupportPointCalculator.h"


ConstraintInformation SupportPointCalculator::getContraintInformation(ARigidBodyOctree* a, ARigidBodyOctree* b) {
	
	ConstraintInformation info;

	info.penetrationDepth = 0.0f;
	info.n = Vector3f(1.0, 0.0, 0.0);
	info.rA = Vector3f(0.0, 0.0, 0.0);
	info.rB = Vector3f(0.0, 0.0, 0.0);

	ARigidBodyOctree::Type typeA = a->getType();
	ARigidBodyOctree::Type typeB = b->getType();


	if (typeA == ARigidBodyOctree::Type::SPHERE && typeB == ARigidBodyOctree::Type::SPHERE) {

		Vector3f posA = Utils::vec3fVmmlToEigen(a->getPosition());
		Vector3f posB = Utils::vec3fVmmlToEigen(b->getPosition());

		info.n = posB - posA;
		info.n.normalize();

		info.rA = info.n * 1.0; // ((SphereRigidBody*)a)->getRadius();
		info.rB = -info.n * 1.0; // ((SphereRigidBody*)b)->getRadius();

		Vector3f rAtoRB = (posB + info.rB) - (posA + info.rA);

		info.penetrationDepth = -rAtoRB.dot(info.n);


	} else if (typeA == ARigidBodyOctree::Type::VERTICES && typeB == ARigidBodyOctree::Type::VERTICES) {

		Vector3f midA = Utils::vec3fVmmlToEigen(a->getPosition());
		Vector3f midB = Utils::vec3fVmmlToEigen(b->getPosition());
		std::vector<Vector3f> verticesA = a->getVertices();
		std::vector<Vector3f> verticesB = b->getVertices();
		std::vector<SimplexP> simplex = std::vector<SimplexP>();

		bool hasPenetration = gjk(midA, midB, verticesA, verticesB, simplex);
		if (hasPenetration) {
			epa(midA, midB, verticesA, verticesB, simplex, info);

		} else {
			info.penetrationDepth = -1.0; // no penetration
		}
		

	}
	
	return info;
}


Vector3f SupportPointCalculator::getFarthestPointInDirection(const Vector3f & direction, std::vector<Vector3f> & points) {

	Vector3f farthest = points.at(0);
	float max = direction.dot(farthest);

	Vector3f temp;
	for (int i = 1; i < points.size(); i++) {
		temp = points.at(i);
		float projection = direction.dot(temp);
		if (projection > max) {
			farthest = temp;
			max = projection;
		}
	}
	return farthest;
}


SimplexP SupportPointCalculator::getSupportAminusB(Vector3f & midA, Vector3f & midB, const Vector3f & direction, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB) {
	Vector3f supportA = getFarthestPointInDirection(direction, verticesA);
	Vector3f supportB = getFarthestPointInDirection(-direction, verticesB);
	return SimplexP { supportA - supportB, supportA - midA, supportB - midB};
}


bool SupportPointCalculator::gjk(Vector3f & midA, Vector3f & midB, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB, std::vector<SimplexP> & simplex) {

	Vector3f direction = Vector3f(1.0, 0.0, 0.0);

	simplex.push_back(getSupportAminusB(midA, midB, direction, verticesA, verticesB));
	direction = -direction;

	while (true) {
		simplex.push_back(getSupportAminusB(midA, midB, direction, verticesA, verticesB));

		if (simplex.back().s.dot(direction) <= 0) {
			return false;

		} else if (containsOrigin(simplex, direction)) {
			return true;
		}
	}
}


bool SupportPointCalculator::containsOrigin(std::vector<SimplexP> & simplex, Vector3f & direction) {

	int size = simplex.size();

	SimplexP a = simplex[size - 1];
	Vector3f aToOrigin = -a.s;

	if (size == 4) {

		SimplexP b = simplex[size - 2];
		SimplexP c = simplex[size - 3];
		SimplexP d = simplex[size - 4];

		Vector3f ab = b.s - a.s;
		Vector3f ac = c.s - a.s;
		Vector3f ad = d.s - a.s;


		if ((ab.cross(ac)).dot(aToOrigin) > 0) {
			simplex = { c, b, a };

		} else if ((ac.cross(ad)).dot(aToOrigin) > 0) {
			simplex = { d, c, a };
		
		} else if ((ad.cross(ab)).dot(aToOrigin) > 0) {
			simplex = { b, d, a };

		} else {
			return true;
		}
	}

	// triangle
	if (size == 3) {

		SimplexP b = simplex[size - 2];
		SimplexP c = simplex[size - 3];

		Vector3f ab = b.s - a.s;
		Vector3f ac = c.s - a.s;

		Vector3f abPerp = tripleProduct(ac, ab, ab);
		Vector3f acPerp = tripleProduct(ab, ac, ac);

		if (abPerp.dot(aToOrigin) > 0) { // origin near ab edge outside triangle 
			simplex = { b, a };
			direction = abPerp;

		} else if (acPerp.dot(aToOrigin) > 0) { // origin near ac edge outside triangle
			simplex = { c, a };
			direction = acPerp;

		} else if (ab.cross(ac).dot(aToOrigin) > 0) { // origin above triangle
			simplex = { c, b, a };
			direction = ab.cross(ac);

		} else { // origin below triangle
			simplex = { b, c, a };
			direction = -ab.cross(ac);
		}

	// line
	} else {
		SimplexP b = simplex[size - 2];

		Vector3f ab = b.s - a.s;

		direction = tripleProduct(ab, aToOrigin, ab);
	}

	return false;
}


Vector3f SupportPointCalculator::tripleProduct(Vector3f a, Vector3f b, Vector3f c) {
	return (b * c.dot(a)) - (a * c.dot(b));
}


void SupportPointCalculator::epa(Vector3f & midA, Vector3f & midB, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB, std::vector<SimplexP> & simplex, ConstraintInformation & info) {

	const float THRESH = 0.001f;
	const int MAXITER = 50;

	std::vector<Triangle> triangles = std::vector<Triangle>();
	triangles.push_back(Triangle { simplex[3], simplex[2], simplex[1] }); // could be the wrong order
	triangles.push_back(Triangle { simplex[3], simplex[1], simplex[0] });
	triangles.push_back(Triangle { simplex[3], simplex[0], simplex[2] });
	triangles.push_back(Triangle { simplex[2], simplex[0], simplex[1] });
	
	Triangle closestTriangle;
	float distance;
	Vector3f closestN;

	int n = 0;
	while (n < MAXITER) {

		distance = FLT_MAX;
		findClosestTriangle(triangles, closestTriangle, distance, closestN);

		SimplexP currentSupport = getSupportAminusB(midA, midB, closestN, verticesA, verticesB);

		if (abs((abs(closestN.dot(currentSupport.s)) - distance) < THRESH)) {
			produceConstraintInformation(closestTriangle, info);
			return;
		}

		std::vector<Edge> edges = std::vector<Edge>();
		for (auto it = triangles.begin(); it != triangles.end(); ) {
			if (closestN.dot(currentSupport.s - it->a.s) > 0) {
				edges.push_back(Edge { it->a, it->b });
				edges.push_back(Edge { it->b, it->c });
				edges.push_back(Edge { it->c, it->a });
				it = triangles.erase(it);
				continue;
			}
			it++;
		}

		for (auto it = edges.begin(); it != edges.end(); it++) {
			triangles.push_back(Triangle { currentSupport, it->a, it->b });
		}
	}

	findClosestTriangle(triangles, closestTriangle, distance, closestN);
	produceConstraintInformation(closestTriangle, info);
}


void SupportPointCalculator::findClosestTriangle(std::vector<Triangle> & triangles, Triangle & closestTriangle, float & distance, Vector3f & closestN) {
	for (std::vector<Triangle>::size_type i = 0; i != triangles.size(); i++) {
		Triangle triangle = triangles[i];
		Vector3f triNormal = triangle.getNormal();
		float triDist = triNormal.dot(triangle.a.s);
		if (triDist < 0) {
			triNormal = -triNormal;
			triDist = -triDist;
		}
		if (triDist < distance) {
			distance = triDist;
			closestTriangle = triangle;
			closestN = triNormal;
		}
	}
}


void SupportPointCalculator::produceConstraintInformation(Triangle & triangle, ConstraintInformation & info) {

	info.n = triangle.getNormal();
	if (info.n.dot(triangle.a.s) < 0) {
		info.n = -info.n;
	}

	info.penetrationDepth = triangle.a.s.dot(info.n);
	Vector3f collisionPoint = info.n * info.penetrationDepth;

	float u;
	float v;
	float w;

	barycentric(collisionPoint, triangle.a.s, triangle.b.s, triangle.c.s, u, v, w);

	info.rA = triangle.a.a * u + triangle.b.a * v + triangle.c.a * w;
	info.rB = triangle.a.b * u + triangle.b.b * v + triangle.c.b * w;	
}


void SupportPointCalculator::barycentric(const Vector3f & p, const Vector3f & a, const Vector3f & b, const Vector3f & c, float & u, float & v, float & w) {
	// code from Crister Erickson's Real-Time Collision Detection
	Vector3f v0 = b - a;
	Vector3f v1 = c - a;
	Vector3f v2 = p - a;
	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);
	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}