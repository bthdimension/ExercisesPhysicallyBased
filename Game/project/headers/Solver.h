//
//  Solver.hpp
//  bRenderer_osx
//
//  Created by Jacob Jacob on 06.12.16.
//  Copyright © 2016 b-dimension.com. All rights reserved.
//

#ifndef Solver_h
#define Solver_h

#include "bRenderer.h"
#include "ARigidBodyOctree.h"
#include "SupportPointCalculator.h"
#include <Dense>


using namespace Eigen;


struct ConstraintStackElement {
	ARigidBodyOctree* a;
	ARigidBodyOctree* b;
	ConstraintInformation info;
};


class Solver{
public:
    Solver();

	void setRididBodyIndices(std::vector<ARigidBodyOctree*> bodies);
	void createConstraintCheckMatrix(int size);
	void registerConstraint(ARigidBodyOctree* a, ARigidBodyOctree* b, ConstraintInformation info);
	void assembleMatrices(std::vector<ARigidBodyOctree*> bodies);
	void solveForLambda(float dt, int iterations);
	void computeNewVelocity(float dt, std::vector<ARigidBodyOctree*> bodies);

private :

	MatrixXf Jsp(int i, int j);
	MatrixXf Bsp(int i, int j);

    
private:

	MatrixXi _constraintCheckMatrix;
	std::vector<ConstraintStackElement> _constraintStack;
	
	int _6n; // 6 * num of objects
	int _s; // num of constraints
	MatrixXf _M; // mass and inertia of all objects
	MatrixXf _Minv; // inverse M
	VectorXf _v; // linear and angular velocity
	VectorXf _Fext; // external forces
	MatrixXf _J; // jacobian
	MatrixXf _Jtrans; // transposed jacobian
	MatrixXf _B; // M^-1 * J^T
	MatrixXi _Jmap; // sparse jacobian map
	VectorXf _lambda; // Lambda of last step
    
};


typedef std::shared_ptr< Solver >  SolverPtr;

#endif /* Solver_h */
