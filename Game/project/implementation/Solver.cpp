//
//  Solver.cpp
//  bRenderer_osx
//
//  Created by Jacob Jacob on 06.12.16.
//  Copyright © 2016 b-dimension.com. All rights reserved.
//

#include "Solver.h"
//#include "Utils.h"


Solver::Solver() {
	_constraintStack = std::vector<ConstraintStackElement>();
	//_lambda = VectorXf::Zero(_s);
}


void Solver::setRididBodyIndices(std::vector<ARigidBodyOctree*> bodies) {
	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != bodies.size(); i++) {
		bodies[i]->setIndex((int)i);
	}
}


void Solver::createConstraintCheckMatrix(int size) {
	_constraintCheckMatrix = ArrayXXi::Zero(size, size);
}


void Solver::registerConstraint(ARigidBodyOctree* a, ARigidBodyOctree* b, ConstraintInformation info) {
	int aI = a->getIndex();
	int bI = b->getIndex();
	if (_constraintCheckMatrix(aI, bI) == 0) {
		_constraintStack.push_back(ConstraintStackElement { a, b, info });
		_constraintCheckMatrix(aI, bI) = 1;
		_constraintCheckMatrix(bI, aI) = 1;
	}
}


void Solver::assembleMatrices(std::vector<ARigidBodyOctree*> bodies) {

	_6n = (int) bodies.size() * 6;
	_s = (int) _constraintStack.size();


	// Mass, veloctiy and external force matrices

	_M = ArrayXXf::Zero(_6n, _6n);
	_v = ArrayXf::Zero(_6n);
	_Fext = ArrayXf::Zero(_6n);
    
    _constraint = VectorXf::Zero(_s);
    _zeta = VectorXf::Zero(_s);

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

		I = Utils::mat3fVmmlToEigen(bodies[i]->getMomentsOfInertia());
		_M.block<3,3>(offset + 3, offset + 3) = I;

		v = Utils::vec3fVmmlToEigen(bodies[i]->getVelocity());
		omega = Utils::vec3fVmmlToEigen(bodies[i]->getAngularVelocity());

		_v.segment<3>(offset) = v;
		_v.segment<3>(offset + 3) = omega;

		if (bodies[i]->isFixed()) {
			_Fext.segment<3>(offset) = Vector3f::Zero();
		} else {
			_Fext.segment<3>(offset) = Utils::vec3fVmmlToEigen(bodies[i]->getForce());
		}

	}

	//std::cout << _M << std::endl << std::endl << _v << std::endl << std::endl;



	// Constraints

	_J = ArrayXXf::Zero(_s, _6n);
	_Jmap = MatrixXi(_s, 2);

	ARigidBodyOctree* body1;
	ARigidBodyOctree* body2;

	int body1offset;
	int body2offset;
	int row;

	for (std::vector<ConstraintStackElement>::size_type i = 0; i != _constraintStack.size(); i++) {

		body1 = _constraintStack[i].a;
		body2 = _constraintStack[i].b;

        Vector3f posA = Utils::vec3fVmmlToEigen(body1->getPosition());
        Vector3f posB = Utils::vec3fVmmlToEigen(body2->getPosition());
        
        
		body1offset = body1->getIndex() * 6;
		body2offset = body2->getIndex() * 6;
		row = (int)i;

		ConstraintInformation constraintInfo = _constraintStack[i].info;

		if (body1->isFixed()) {
			_J.block<1, 6>(i, body1offset) = ArrayXXf::Zero(1, 6);
		} else {
			_J.block<1, 3>(i, body1offset) = -constraintInfo.n.transpose();
			_J.block<1, 3>(i, body1offset + 3) = -(constraintInfo.rA.cross(constraintInfo.n).transpose());

		}
		if (body2->isFixed()) {
			_J.block<1, 6>(i, body2offset) = ArrayXXf::Zero(1, 6);

		} else {
			_J.block<1, 3>(i, body2offset) = constraintInfo.n.transpose();
			_J.block<1, 3>(i, body2offset + 3) = constraintInfo.rB.cross(constraintInfo.n).transpose();
		}

		_Jmap(i, 0) = body1->getIndex();
		_Jmap(i, 1) = body2->getIndex();
        
        _constraint(row) = (posB + constraintInfo.rB - (posA + constraintInfo.rA)).dot( constraintInfo.n );

//        _bias(i) += alpha * (Utils::vec3fVmmlToEigen(body2->getVelocity()) +
//                           Utils::vec3fVmmlToEigen(body2->getAngularVelocity()).cross(constraintInfo.rB) -
//                           Utils::vec3fVmmlToEigen(body1->getVelocity()) -
//                           Utils::vec3fVmmlToEigen(body1->getAngularVelocity()).cross(constraintInfo.rA)).dot( constraintInfo.n );

        
	}
	_constraintStack.clear();

	_Minv = _M.inverse();
	_Jtrans = _J.transpose();
	_B = _Minv * _Jtrans;
    
    
	//std::cout << _J << std::endl << std::endl;
}


void Solver::solveForLambda(float dt, int iterations) {

	// We set lambda0 to zero, maybe consider warm starting
	VectorXf lambda0 = VectorXf::Zero(_s); // save lambda from last step in lambda0
    
//    if(dt > 1.){
//        lambda0 = _lambda;
//    }
    
	//std::cout << "B = " << std::endl << _B << std::endl << std::endl;
	//std::cout << "lamdba0 = " << std::endl << lambda0 << std::endl << std::endl;

    

	_lambda = lambda0;
	VectorXf a = _B * _lambda;


	VectorXf d = VectorXf(_s); // J * B
	VectorXf lambdaMin = VectorXf(_s);
	VectorXf lambdaMax = VectorXf(_s);

	for (int i = 0; i < _s; i++) {
		d(i) = (Jsp(i, 0) * Bsp(i, 0) +
			Jsp(i, 1) * Bsp(i, 1))
			(0); // get float

		lambdaMin(i) = 0.0f;
		lambdaMax(i) = 9999.9f; // TODO: what is best upper bound
	}

    float beta = 0.3f;
    
//    std::cout << _constraint << std::endl << std::endl;
    
	VectorXf eta = - beta * _constraint / dt  -_J * (((1.f / dt) * _v) + (_Minv * _Fext));
//    eta -= _zeta /dt;

	for (int iter = 0; iter < iterations; iter++) {

		for (int i = 0; i < _s; i++) {
			int b0 = _Jmap(i, 0);
			int b1 = _Jmap(i, 1);
			float deltaLambdaI = (
				eta(i)
				- (Jsp(i, 0) * a.segment<6>(b0 * 6))(0)
				- (Jsp(i, 1) * a.segment<6>(b1 * 6))(0)
				) / d(i);
			lambda0(i) = _lambda(i);
			_lambda(i) = std::max(lambdaMin(i), std::min(lambdaMax(i), lambda0(i) + deltaLambdaI));
			deltaLambdaI = _lambda(i) - lambda0(i);
			a.segment<6>(b0 * 6) += deltaLambdaI * Bsp(i, 0);
			a.segment<6>(b1 * 6) += deltaLambdaI * Bsp(i, 1);
		}

	}

	//std::cout << "_lamdba = " << std::endl << _lambda << std::endl << std::endl;
};


void Solver::computeNewVelocity(float dt, std::vector<ARigidBodyOctree*> bodies) {
	//VectorXf v2 = _v + (dt * _Minv * _Fext);
	_v2 = _v + (dt * _Minv * ((_Jtrans * _lambda) + _Fext));
	//std::cout << "_Minv = " << std::endl << _Minv << std::endl << std::endl;
	//std::cout << "_Jtrans = " << std::endl << _Jtrans << std::endl << std::endl;
	//std::cout << "_lamdba = " << std::endl << _lambda << std::endl << std::endl;
	//std::cout << "_Fext = " << std::endl << _Fext << std::endl << std::endl;
    
//    _zeta = _J * _v2;
    
	int index;
	int offset;

	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != bodies.size(); i++) {

		index = bodies[i]->getIndex();
		offset = index * 6;

		vmml::Vector3f vel = vmml::Vector3f(_v2(offset), _v2(offset + 1), _v2(offset + 2));
		vmml::Vector3f pos = bodies[i]->getPosition();
		pos += vel * dt;
        
        vmml::Vector3f angularvel = vmml::Vector3f(_v2(offset + 3), _v2(offset + 4), _v2(offset + 5));
        vmml::Vector3f angles = bodies[i]->getAxesRotation();
        angles += angularvel * dt;
        
		bodies[i]->setVelocity(vel);
		bodies[i]->setPosition(pos);
        bodies[i]->setAxesRotation(angles);
        
        bodies[i]->updateMatrices();
	}
}


MatrixXf Solver::Jsp(int i, int j) {
	//std::cout << "Jsp" << i << "/" << j << " (" << _J << std::endl;
	return _J.block<1, 6>(i, _Jmap(i, j) * 6);
}


MatrixXf Solver::Bsp(int i, int j) {
	//std::cout << "Bsp" << i << "/" << j << " (" << _B << std::endl;
	//std::cout << "Jmap" << i << "/" << j << " (" << _Jmap << std::endl;
	return _B.block<6, 1>(_Jmap(i, j) * 6, i);
}
