//
//  Solver.hpp
//  bRenderer_osx
//
//  Created by Jacob Jacob on 06.12.16.
//  Copyright © 2016 b-dimension.com. All rights reserved.
//

#ifndef Solver_hpp
#define Solver_hpp

#include <vector>
#include "vmmlib/matrix.hpp"
#include "vmmlib/vector.hpp"

class Solver{
public:
    Solver(){};
    
private:
    // number of constraints
    size_t s;
    // number of rigid bodies
    size_t n;
    // len = 6*n
    size_t len;
    
    // velocity vector
//    vmml::Vector<len> V;
    // Jacobian of Constraints
//    vmml::Matrix<s,len> Jacobian;
    //
    
    
};



#endif /* Solver_hpp */
