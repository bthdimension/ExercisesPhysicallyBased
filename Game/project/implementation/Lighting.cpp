#include "Lighting.h"


Lighting::Lighting(ObjectManagerPtr objectManager) {
	_objectManager = objectManager;
}


void Lighting::setup() {

	_objectManager->setAmbientColor(vmml::Vector3f(0.05f, 0.05f, 0.05f));

	_objectManager->createLight(
		"sun", // name
		vmml::Vector3f(-50.0f, 200.0f, -100.0f), // position
		vmml::Vector3f(1.0f, 1.0f, 1.0f), // diffuse color
		vmml::Vector3f(1.0f, 1.0f, 1.0f), // specular color
		400.0f, // intensity
		0.01f, // attenuation
		1000.0f // radius
	);

	_objectManager->createLight(
		"moon", // name
		vmml::Vector3f(50.0f, 100.0f, 100.0f), // position
		vmml::Vector3f(0.4f, 0.4f, 0.4f), // diffuse color
		vmml::Vector3f(0.4f, 0.4f, 0.4f), // specular color
		200.0f, // intensity
		0.01f, // attenuation
		1000.0f // radius
	);
}