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
#include <Dense>

using namespace Eigen;

class Solver{
public:
    Solver();

	void assembleMatrices(std::vector<ARigidBodyOctree*> bodies);

	Matrix3f mat3fVmmlToEigen(vmml::Matrix3f input);
	Vector3f vec3fVmmlToEigen(vmml::Vector3f input);
    
private:

	MatrixXf _M;
	VectorXf _v;    
    
};


typedef std::shared_ptr< Solver >  SolverPtr;

#endif /* Solver_h */
