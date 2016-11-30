#ifndef SPHERE_H
#define SPHERE_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class Sphere : public ARigidBodyOctree {

public:

	Sphere(ModelPtr model);
	
	Sphere(ModelPtr model, vmml::Vector3f position);

	void update(const double &deltaTime);
	void draw(ModelRendererPtr modelRenderer, int id);

private:

};


#endif
