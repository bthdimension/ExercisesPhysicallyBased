#ifndef SPHERE_H
#define SPHERE_H

#include "bRenderer.h"
#include "ARigidBody.h"


class Sphere : public ARigidBody {

public:

	Sphere(vmml::Vector3f position, double radius);

	int getType();

	bool isInAABB(double x0, double x1, double y0, double y1, double z0, double z1);

	void update(const double &deltaTime);
	void draw(ModelRendererPtr modelRenderer, int id);

	void setVelocity(vmml::Vector3f velocity);
	vmml::Vector3f getVelocity();


private:

	vmml::Vector3f _position;
	double _radius;

	vmml::Vector3f _velocity;

};


#endif
