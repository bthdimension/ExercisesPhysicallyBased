#ifndef SUPPORT_POINT_CALCULATOR_H
#define SUPPORT_POINT_CALCULATOR_H

#include "bRenderer.h"
#include "SphereRigidBody.h"
#include "ARigidBodyOctree.h"
#include "Utils.h"
#include <Dense>

using namespace Eigen;


struct SimplexP {
	Vector3f s;
	Vector3f a;
	Vector3f b;
    
	Vector3f worldA;
	Vector3f worldB;

    bool operator==(const SimplexP &r) const { return s == r.s; }

};


struct ConstraintInformation {
    bool col;
	float penetrationDepth;
	Vector3f n;
	Vector3f rA;
	Vector3f rB;
    Vector3f contactPointa;
    Vector3f contactPointb;
    
    float conVal; //constraint value
};


struct Edge {
	SimplexP a;
	SimplexP b;
};


struct Triangle {
	SimplexP a;
	SimplexP b;
	SimplexP c;

	Vector3f getNormal() {
		Vector3f n = ((b.s - a.s).cross(c.s - a.s));
		n.normalize();
		return n;
	}
};



class SupportPointCalculator {

public:

	static ConstraintInformation getContraintInformation(ARigidBodyOctree* a, ARigidBodyOctree* b);

	static bool findClosestTriangleToSphereMid(Vector3f & sphereMid, std::vector<std::vector<Vector3f>> & triangles, float & distance, Vector3f & closestN);
	static bool isPointInTriangle(Vector3f & point, std::vector<Vector3f> & triangle);
	static void findClosestEdgeToSphereMid(Vector3f & sphereMid, std::vector<std::vector<Vector3f>> & triangles, float & distance, Vector3f & closestN);
	static void findClosestVertexToSphereMid(Vector3f & sphereMid, std::vector<Vector3f> & vertices, float & distance, Vector3f & closestN);

	static Vector3f getFarthestPointInDirection(const Vector3f & direction, std::vector<Vector3f> & points);
	static SimplexP getSupportAminusB(Vector3f & midA, Vector3f & midB, const Vector3f & direction, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB);

	static bool gjk(Vector3f & midA, Vector3f & midB, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB, std::vector<SimplexP> & simplex);
	static bool containsOrigin(std::vector<SimplexP> & simplex, Vector3f & direction);

	static Vector3f tripleProduct(Vector3f a, Vector3f b, Vector3f c);

	static void epa(Vector3f & midA, Vector3f & midB, std::vector<Vector3f> & verticesA, std::vector<Vector3f> & verticesB, std::vector<SimplexP> & simplex, ConstraintInformation & info);
	static void findClosestTriangle(std::vector<Triangle> & triangles, Triangle & closestTriangle, float & distance, Vector3f & closestN);
	static void produceConstraintInformation(Triangle & triangle, ConstraintInformation & info);

	static void barycentric(const Vector3f & p, const Vector3f & a, const Vector3f & b, const Vector3f & c, float * u, float * v, float * w);

};


#endif
