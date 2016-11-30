#include "Scene.h"


Scene::Scene(Renderer* bRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera) {

	_bRenderer = bRenderer;
	_modelRenderer = bRenderer->getModelRenderer();
	_inputController = inputController;
	_freeCamera = freeCamera;

	_sceneEditor = SceneEditorPtr(new SceneEditor(this, _modelRenderer, inputController, freeCamera));

	_lighting = LightingPtr(new Lighting(bRenderer->getObjects()));
	_lighting->setup();

	_rigidBodies = std::vector<ARigidBodyOctree*>();

	_octTree = OctTreeNodePtr(new OctTreeNode(
		0, // make root
		5, // depth,
		{{ -25.f, -10.f, -25.f},{25.f, 40.f, 25.f}}
	));

	generateSpheres();
}


Scene::~Scene() {
	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != _rigidBodies.size(); i++) {
		delete _rigidBodies[i];
	}
}


void Scene::generateSpheres() {
	srand(time(NULL));
	for (int i = 0; i < 32; i++) {
		vmml::Vector3d position = vmml::Vector3d(15.0 + ((rand() % 100) * 0.1), 0.5, -25.0 + ((rand() % 500) * 0.1));
		Sphere* sphere = new Sphere(_modelRenderer->getObjectManager()->getModel("sphere"), position);

		vmml::Vector3d velocity = vmml::Vector3d(-6.0 + ((rand() % 300) * 0.01), 0.0, -1.0 + ((rand() % 200) * 0.01));
		sphere->setVelocity(velocity);

		addRigidBody((ARigidBodyOctree*)sphere);
	}
}


void Scene::loop(const double &deltaTime, bool* running) {
	_sceneEditor->update();
	if (*running) {
		update(deltaTime);
	}
	draw();
}


void Scene::addRigidBody(ARigidBodyOctree* rigidBody) {
	_rigidBodies.push_back(rigidBody);
	rigidBody->setOctTree(_octTree);
	rigidBody->registerInOctTree();
}


void Scene::update(const double &deltaTime) {
	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != _rigidBodies.size(); i++) {
		_rigidBodies[i]->update(deltaTime);
	}
	_octTree->collide();
}


void Scene::draw() {
	_modelRenderer->drawQueue(/*GL_LINES*/);
	_modelRenderer->clearQueue();

	vmml::Matrix4f modelMatrix = vmml::create_translation(vmml::Vector3f(0.0f, 0.0f, 0.0f)) * vmml::create_scaling(vmml::Vector3f(1.0f));
	_modelRenderer->queueModelInstance("base", "base", "camera", modelMatrix, std::vector<std::string>({ "sun", "moon" }), true, true);

	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != _rigidBodies.size(); i++) {
		_rigidBodies[i]->draw(_modelRenderer, (int) i);
	}
}