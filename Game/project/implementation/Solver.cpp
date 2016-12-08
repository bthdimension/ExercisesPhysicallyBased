//
//  Solver.cpp
//  bRenderer_osx
//
//  Created by Jacob Jacob on 06.12.16.
//  Copyright © 2016 b-dimension.com. All rights reserved.
//

#include "Solver.h"


Solver::Solver() {
}


void Solver::assembleMatrices(std::vector<ARigidBodyOctree*> bodies) {

	int massMatrixSize = bodies.size() * 6;

	_M = ArrayXXf::Zero(massMatrixSize, massMatrixSize);
	_v = ArrayXf(massMatrixSize);

	int offset;

	float m;
	Matrix3f I;

	Vector3f v;
	Vector3f omega;

	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != bodies.size(); i++) {
		
		offset = (int)i * 6;

		m = bodies[i]->getMass();
		_M(offset, offset) = m;
		_M(offset + 1, offset + 1) = m;
		_M(offset + 2, offset + 2) = m;

		I = mat3fVmmlToEigen(bodies[i]->getMomentsOfInertia());
		_M.block<3,3>(offset + 3, offset + 3) = I;

		v = vec3fVmmlToEigen(bodies[i]->getVelocity());
		omega = vec3fVmmlToEigen(bodies[i]->getAngularVelocity());

		_v.segment(offset, 3) = v;
		_v.segment(offset + 3, 3) = omega;
	}



	std::cout << _M << std::endl << std::endl << _v << std::endl << std::endl;
}


Matrix3f Solver::mat3fVmmlToEigen(vmml::Matrix3f input) {
	Matrix3f output = Matrix3f();
	for (int x = 0; x < 3; x++) {
		for (int y = 0; y < 3; y++) {
			output(x, y) = input(x, y);
		}
	}
	return output;
};


Vector3f Solver::vec3fVmmlToEigen(vmml::Vector3f input) {
	Vector3f output = Vector3f();
	for (int i = 0; i < 3; i++) {
		output(i) = input(i);
	}
	return output;
}