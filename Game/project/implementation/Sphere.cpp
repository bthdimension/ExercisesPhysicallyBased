#include "Sphere.h"


Sphere::Sphere(vmml::Vector3f position, double radius) {
	_position = position;
	_radius = radius;

	_velocity = vmml::Vector3f(0.0, 0.0, 0.0);
}


int Sphere::getType() {
	return ARigidBody::TYPE_SPHERE;
}


bool Sphere::isInAABB(double x0, double x1, double y0, double y1, double z0, double z1) {

	// TODO: There might be a more efficient way to do this
	double bX0 = _position.x() - _radius;
	double bX1 = _position.x() + _radius;
	double bY0 = _position.y() - _radius;
	double bY1 = _position.y() + _radius;
	double bZ0 = _position.z() - _radius;
	double bZ1 = _position.z() + _radius;
	
	return ((x0 > bX0 && x0 < bX1) || (x1 > bX0 && x1 < bX1) || (x0 < bX0 && x1 > bX1)) &&
		((y0 > bY0 && y0 < bY1) || (y1 > bY0 && y1 < bY1) || (y0 < bY0 && y1 > bY1)) &&
		((z0 > bZ0 && z0 < bZ1) || (z1 > bZ0 && z1 < bZ1) || (z0 < bZ0 && z1 > bZ1));
}


void Sphere::update(const double &deltaTime) {


	// TODO do physics of block here

	// just as example:
	_position += _velocity * deltaTime;

	// only do this if there was a change in position/orientation
	//registerInOctTree();
}


void Sphere::draw(ModelRendererPtr modelRenderer, int id) {

	vmml::Matrix4f modelMatrix = vmml::create_translation(_position);
	modelMatrix *= vmml::create_scaling(vmml::Vector3f(_radius));

	modelRenderer->queueModelInstance(
		"sphere",
		"sphere_" + std::to_string(id),
		"camera", 
		modelMatrix,
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}


void Sphere::setVelocity(vmml::Vector3f velocity) {
	_velocity = velocity;
}


vmml::Vector3f Sphere::getVelocity() {
	return _velocity;
}