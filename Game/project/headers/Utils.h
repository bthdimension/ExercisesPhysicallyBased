#ifndef UTILS_H
#define UTILS_H

#include "bRenderer.h"
#include <eigen3/Eigen/Dense>


using namespace Eigen;


class Utils {

public:

	static Matrix3f mat3fVmmlToEigen(vmml::Matrix3f input) {
		Matrix3f output = Matrix3f();
		for (int x = 0; x < 3; x++) {
			for (int y = 0; y < 3; y++) {
				output(x, y) = input(x, y);
			}
		}
		return output;
	}

	static Vector3f vec3fVmmlToEigen(vmml::Vector3f input) {
		Vector3f output = Vector3f();
		for (int i = 0; i < 3; i++) {
			output(i) = input(i);
		}
		return output;
	}
    
    static vmml::Vector3f EigenTovec3fVmml(Vector3f input) {
        vmml::Vector3f output = vmml::Vector3f();
        for (int i = 0; i < 3; i++) {
            output(i) = input(i);
        }
        return output;
    }
    

};


#endif
