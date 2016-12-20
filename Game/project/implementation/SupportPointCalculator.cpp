#include "SupportPointCalculator.h"
#include <limits>

template<typename T>
bool is_infinite( const T &value ){
    // Since we're a template, it's wise to use std::numeric_limits<T>
    //
    // Note: std::numeric_limits<T>::min() behaves like DBL_MIN, and is the smallest absolute value possible.
    //
    
    T max_value = std::numeric_limits<T>::max();
    T min_value = - max_value;
    
    return ! ( min_value <= value && value <= max_value );
}

template<typename T>
bool is_nan( const T &value ){
    // True if NAN
    return value != value;
}

template<typename T>
bool is_valid( const T &value ){
    return ! is_infinite(value) && ! is_nan(value);
}

void addEdge(const SimplexP & a, const SimplexP & b, std::vector<Edge> & edges ){
    for (auto it = edges.begin(); it != edges.end(); it++) {
        if(it->a == b && it->b == a){
            edges.erase(it);
            return;
        }
    }
    edges.push_back(Edge {a, b});
}

ConstraintInformation SupportPointCalculator::getContraintInformation(ARigidBodyOctree* a, ARigidBodyOctree* b) {
	
	ConstraintInformation info;

	info.penetrationDepth = 0.0f;
	info.n = Vector3f(0.0, 1.0, 0.0);
	info.rA = Vector3f(0.0, 0.0, 0.0);
	info.rB = Vector3f(0.0, 0.0, 0.0);

	ARigidBodyOctree::Type typeA = a->getType();
	ARigidBodyOctree::Type typeB = b->getType();
    
    vmml::Matrix4f invworldMatrixA = a->getInverseWorldMatrix();
    vmml::Matrix4f invworldMatrixB = b->getInverseWorldMatrix();


	if (typeA == ARigidBodyOctree::Type::SPHERE && typeB == ARigidBodyOctree::Type::SPHERE) {

		Vector3f posA = Utils::vec3fVmmlToEigen(a->getPosition());
		Vector3f posB = Utils::vec3fVmmlToEigen(b->getPosition());

		info.n = posB - posA;
		info.n.normalize();

		info.rA = info.n * ((SphereRigidBody*)a)->getRadius();
		info.rB = -info.n * ((SphereRigidBody*)b)->getRadius();

		Vector3f rAtoRB = (posB + info.rB) - (posA + info.rA);

		info.penetrationDepth = -rAtoRB.dot(info.n);


	} else if (typeA == ARigidBodyOctree::Type::VERTICES && typeB == ARigidBodyOctree::Type::VERTICES) {
        
        
		Vector3f midA = Utils::vec3fVmmlToEigen(a->getPosition());
		Vector3f midB = Utils::vec3fVmmlToEigen(b->getPosition());
        
        // vertices in worldcoordinates
		std::vector<Vector3f> verticesA = a->getWorldVertices();
		std::vector<Vector3f> verticesB = b->getWorldVertices();
		std::vector<SimplexP> simplex = std::vector<SimplexP>();

		bool hasPenetration = gjk(midA, midB, verticesA, verticesB, simplex);
		if (hasPenetration) {
			epa(midA, midB, verticesA, verticesB, simplex, info);
            if(info.col){
//                Vector3f conPointBodyA = Utils::vec3fVmmlToEigen((invworldMatrixA * Utils::EigenTovec3fVmml(info.contactPointa)));
//                Vector3f conPointBodyB = Utils::vec3fVmmlToEigen((invworldMatrixB * Utils::EigenTovec3fVmml(info.contactPointb)));
                
                info.rA = info.contactPointa - midA;
                info.rB = info.contactPointb - midB;
//                std::cout << "contactaWorld " << std::endl << conPointBodyA << std::endl << std::endl;
//                std::cout << "contactbWorld " << std::endl << conPointBodyB << std::endl << std::endl;
//                std::cout << "contactPointa " << std::endl << info.contactPointa << std::endl << std::endl;
//                std::cout << "contactPointb " << std::endl << info.contactPointb << std::endl << std::endl;
//                std::cout << "rA " << std::endl << info.rA << std::endl << std::endl;
//                std::cout << "rB " << std::endl << info.rB << std::endl << std::endl;
//                std::cout << midB << std::endl << std::endl;
            } else {
                info.penetrationDepth = -1.0; // no penetration
            }
            
            
//            std::cout << info.penetrationDepth<< std::endl << std::endl;
		} else {
			info.penetrationDepth = -1.0; // no penetration
		}
		

	} else if ((typeA == ARigidBodyOctree::Type::SPHERE && typeB == ARigidBodyOctree::Type::VERTICES) ||
		(typeA == ARigidBodyOctree::Type::VERTICES && typeB == ARigidBodyOctree::Type::SPHERE)) {

		bool flip = false;
		if (typeA == ARigidBodyOctree::Type::VERTICES && typeB == ARigidBodyOctree::Type::SPHERE) {
			ARigidBodyOctree* temp = a;
			a = b;
			b = temp;
			flip = true;
		}

		Vector3f sphereMid = Utils::vec3fVmmlToEigen(a->getPosition());
		std::vector<std::vector<Vector3f>> triangles = b->getTriangles();
		std::vector<Vector3f> vertices = b->getWorldVertices();

		Vector3f closestN;
		float distance = FLT_MAX;

		bool gotTriangle = findClosestTriangleToSphereMid(sphereMid, triangles, distance, closestN);
		if (!gotTriangle) {
			findClosestEdgeToSphereMid(sphereMid, triangles, distance, closestN);
			findClosestVertexToSphereMid(sphereMid, vertices, distance, closestN);
		}

		if (flip) {
			info.n = -closestN;
			info.rA = (sphereMid - (info.n * distance)) - Utils::vec3fVmmlToEigen(b->getPosition());
			info.rB = -info.n * ((SphereRigidBody*)a)->getRadius();
			info.penetrationDepth = ((SphereRigidBody*)a)->getRadius() - distance;
		} else {
			info.n = closestN;
			info.rA = info.n * ((SphereRigidBody*)a)->getRadius();
			info.rB = (sphereMid + (info.n * distance)) - Utils::vec3fVmmlToEigen(b->getPosition());
			info.penetrationDepth = ((SphereRigidBody*)a)->getRadius() - distance;
		}

	}

	//std::cout << "PENEINFO: depth=" << info.penetrationDepth << "           normal=" << info.n.transpose() << "           rA=" << info.rA.transpose() << "           rB=" << info.rB.transpose() << std::endl;
	
	return info;
}


bool SupportPointCalculator::findClosestTriangleToSphereMid(Vector3f & sphereMid, std::vector<std::vector<Vector3f>> & triangles, float & distance, Vector3f & closestN) {
	bool foundTriangle = false;
	for (std::vector<std::vector<Vector3f>>::size_type i = 0; i != triangles.size(); i++) {
		std::vector<Vector3f> triangle = triangles[i];
		if (isPointInTriangle(sphereMid, triangle)) {
			Vector3f triNormal = ((triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]));
			triNormal.normalize();
			float triDist = triNormal.dot(triangle[0] - sphereMid);
			if (triDist < 0) {
				triNormal = -triNormal;
				triDist = -triDist;
			}
			if (triDist < distance) {
				distance = triDist;
				closestN = triNormal;
				foundTriangle = true;
			}
		}
	}
	return foundTriangle;
}


bool SupportPointCalculator::isPointInTriangle(Vector3f & point, std::vector<Vector3f> & triangle) {
	Vector3f a = triangle[0];
	Vector3f b = triangle[1];
	Vector3f c = triangle[2];

	Vector3f ab = b - a;
	Vector3f bc = c - b;
	Vector3f ca = a - c;

	Vector3f abPerp = tripleProduct(ca, ab, ab);
	Vector3f bcPerp = tripleProduct(ab, bc, bc);
	Vector3f caPerp = tripleProduct(bc, ca, ca);

	return abPerp.dot(point - a) >= 0 && caPerp.dot(point - a) >= 0 && bcPerp.dot(point - b) >= 0;
}


void SupportPointCalculator::findClosestEdgeToSphereMid(Vector3f & sphereMid, std::vector<std::vector<Vector3f>> & triangles, float & distance, Vector3f & closestN) {
	for (std::vector<std::vector<Vector3f>>::size_type i = 0; i != triangles.size(); i++) {
		std::vector<Vector3f> triangle = triangles[i];
		for (std::vector<Vector3f>::size_type j = 0; j != triangle.size(); j++) {
			Vector3f edgeStart = triangle[j];
			Vector3f edge = triangle[(j + 1) % 3] - edgeStart;
			float edgeLen = edge.norm();
			edge.normalize();
			Vector3f edgeStartToMid = sphereMid - edgeStart;
			float proj = edge.dot(edgeStartToMid) / edgeLen;
			if (proj >= 0.0f && proj <= 1.0f) {
				Vector3f normal = tripleProduct(edge, edgeStartToMid, edge);
				normal.normalize();
				float midDist = edgeStartToMid.dot(normal);
				if (midDist < distance) {
					distance = midDist;
					closestN = normal;
				}
			}
		}
	}
}


void SupportPointCalculator::findClosestVertexToSphereMid(Vector3f & sphereMid, std::vector<Vector3f> & vertices, float & distance, Vector3f & closestN) {
	for (std::vector<Vector3f>::size_type i = 0; i != vertices.size(); i++) {
		Vector3f vertex = vertices[i];
		Vector3f vertexToMid = sphereMid - vertex;
		float midDist = vertexToMid.norm();
		vertexToMid.normalize();
		if (midDist < distance) {
			distance = midDist;
			closestN = -vertexToMid;
		}
	}
}


Vector3f SupportPointCalculator::getFarthestPointInDirection(const Vector3f & direction, std::vector<Vector3f> & points) {
    
	int farthestInd = 0;
	float max = direction.dot(points.at(0));
	for (int i = 0; i < points.size(); i++) {
		float projection = direction.dot(points.at(i));
		if (projection > max) {
			farthestInd = i;
			max = projection;
		}
	}
	return points.at(farthestInd);
}


SimplexP SupportPointCalculator::getSupportAminusB(Vector3f & midA, Vector3f & midB, const Vector3f & direction, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB) {
	Vector3f supportA = getFarthestPointInDirection(direction, verticesA);
	Vector3f supportB = getFarthestPointInDirection(-direction, verticesB);

//    std::cout << supportA - supportB << std::endl;
	return SimplexP { supportA - supportB, supportA, supportB};

	// return SimplexP { supportA - supportB, supportA - midA, supportB - midB, supportA, supportB};

}


bool SupportPointCalculator::gjk(Vector3f & midA, Vector3f & midB, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB, std::vector<SimplexP> & simplex) {

	Vector3f direction = Vector3f(1.0, 0.0, 0.0);
    
    SimplexP sup = getSupportAminusB(midA, midB, direction, verticesA, verticesB);
    if(std::fabs(sup.s.dot(direction)) >= sup.s.norm() * 0.8f){
        direction = Vector3f(0.0, 1.0, 0.0);
        sup = getSupportAminusB(midA, midB, direction, verticesA, verticesB);
    }
    
	simplex.push_back(sup);
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
	/*std::cout << "SimplexSize " << size << " Simplex: " << std::endl;

	for (int i = 0; i < size; i++) {
		std::cout << "   " << simplex.at(i).s.transpose() << "/" << std::endl;
		std::cout << "      A: " << simplex.at(i).a.transpose() << "/" << std::endl;
		std::cout << "      B: " << simplex.at(i).b.transpose() << "/" << std::endl;
		std::cout << "      worldA: " << simplex.at(i).worldA.transpose() << "/" << std::endl;
		std::cout << "      worldB:" << simplex.at(i).worldB.transpose() << "/" << std::endl;
	}
	std::cout << std::endl;*/

	SimplexP a = simplex.back();
	Vector3f aToOrigin = -a.s;

	if (size == 4) {

		SimplexP b = simplex[size - 2];
		SimplexP c = simplex[size - 3];
		SimplexP d = simplex[size - 4];

		Vector3f ab = b.s - a.s;
		Vector3f ac = c.s - a.s;
		Vector3f ad = d.s - a.s;
        
        Vector3f abc = ab.cross(ac);
        Vector3f acd = ac.cross(ad);
        Vector3f adb = ad.cross(ab);


		if (abc.dot(aToOrigin) > 0) {
			simplex = { c, b, a };

		} else if (acd.dot(aToOrigin) > 0) {
			simplex = { d, c, a };
		
		} else if (adb.dot(aToOrigin) > 0) {
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
        
        Vector3f abc = ab.cross(ac);

		Vector3f abPerp = tripleProduct(ac, ab, ab);
		Vector3f acPerp = tripleProduct(ab, ac, ac);

		if (abPerp.dot(aToOrigin) > 0) { // origin near ab edge outside triangle
			simplex = { b, a };
			direction = tripleProduct(ab, aToOrigin, ab);

		} else if (acPerp.dot(aToOrigin) > 0) { // origin near ac edge outside triangle
			simplex = { c, a };
			direction = tripleProduct(ac, aToOrigin, ac);

		} else if (abc.dot(aToOrigin) > 0) { // origin above triangle
			simplex = { c, b, a };
			direction = abc;

		} else { // origin below triangle
			simplex = { b, c, a };
			direction = -abc;
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
	return (a.cross(b)).cross(c); // (b * c.dot(a)) - (a * c.dot(b));
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
        n++;
		findClosestTriangle(triangles, closestTriangle, distance, closestN);
		SimplexP currentSupport = getSupportAminusB(midA, midB, closestN, verticesA, verticesB);
        
        if(closestTriangle.a == closestTriangle.b || closestTriangle.a == closestTriangle.c){
            std::cout << "something happend" << std::endl;
            findClosestTriangle(triangles, closestTriangle, distance, closestN);
        }
        
		if ( closestN.dot(currentSupport.s) - distance < THRESH) {
			produceConstraintInformation(closestTriangle, info);
			return;
		}

		std::vector<Edge> edges = std::vector<Edge>();
		for (auto it = triangles.begin(); it != triangles.end(); ) {
			if (closestN.dot(currentSupport.s - it->a.s) > 0) {
                addEdge(it->a, it->b, edges);
                addEdge(it->b, it->c, edges);
                addEdge(it->c, it->a, edges);
				it = triangles.erase(it);
				continue;
			}
			it++;
		}

		for (auto it = edges.begin(); it != edges.end(); it++) {
            triangles.push_back(Triangle { currentSupport, it->a, it->b });
		}
        
	}
    
    info.col = false;

//	findClosestTriangle(triangles, closestTriangle, distance, closestN);
//	produceConstraintInformation(closestTriangle, info);
}


void SupportPointCalculator::findClosestTriangle(std::vector<Triangle> & triangles, Triangle & closestTriangle, float & distance, Vector3f & closestN) {
    distance = FLT_MAX;
	for (std::vector<Triangle>::size_type i = 0; i != triangles.size(); i++) {
		Triangle triangle = triangles[i];
		Vector3f triNormal = triangle.getNormal();
        float triDist = std::fabs(triNormal.dot(triangle.a.s));
        
		if (triDist < distance) {
			distance = triDist;
			closestTriangle = triangle;
			closestN = triNormal;
		}
	}
    
}


void SupportPointCalculator::produceConstraintInformation(Triangle & triangle, ConstraintInformation & info) {

	info.penetrationDepth = triangle.a.s.dot(triangle.getNormal());
    
	Vector3f collisionPoint = info.n * info.penetrationDepth;

	float u;
	float v;
	float w;

	barycentric(collisionPoint, triangle.a.s, triangle.b.s, triangle.c.s, &u, &v, &w);
    
    if(!is_valid(u) || !is_valid(v) || !is_valid(w)){
        info.col = false;
        std::cout << "nan created! " << std::endl;
        std::cout << "a " << std::endl << triangle.a.s << std::endl << std::endl;
        std::cout << "b " << std::endl << triangle.b.s << std::endl << std::endl;
        std::cout << "c " << std::endl << triangle.c.s << std::endl << std::endl;
        
//        return;
    }
    
    info.contactPointa = triangle.a.a * u + triangle.b.a * v + triangle.c.a * w;
    info.contactPointb = triangle.a.b * u + triangle.b.b * v + triangle.c.b * w;
    
//    info.contactPointa = {0.f,0.f,0.f};
//    info.contactPointb = {0.f,0.f,0.f};
    // from body a to b
    info.n = triangle.getNormal();
    info.col = true;
    
    return;
    
}


void SupportPointCalculator::barycentric(const Vector3f & p, const Vector3f & a, const Vector3f & b, const Vector3f & c, float * u, float * v, float * w) {
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
	*v = (d11 * d20 - d01 * d21) / denom;
	*w = (d00 * d21 - d01 * d20) / denom;
	*u = 1.0f - *v - *w;
}
