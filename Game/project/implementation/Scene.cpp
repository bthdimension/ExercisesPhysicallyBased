#include "Scene.h"


Scene::Scene(Renderer* bRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera) {

	_bRenderer = bRenderer;
	_modelRenderer = bRenderer->getModelRenderer();
	_inputController = inputController;
	_freeCamera = freeCamera;	

	_lighting = LightingPtr(new Lighting(bRenderer->getObjects()));
	_lighting->setup();

	_rigidBodies = std::vector<ARigidBodyOctree*>();

	_octTree = OctTreeNodePtr(new OctTreeNode(
		this, //scene
		0, // make root
		4, // depth,
		{{ -25.f, -5.f, -25.f},{25.f, 40.f, 25.f}} // size of octree
	));

	_sceneEditor = SceneEditorPtr(new SceneEditor(this, _modelRenderer, inputController, freeCamera));
	_sceneEditor->createDebugScene();

	_solver = SolverPtr(new Solver());

	//addRigidBody(new FloorRigidBody(_modelRenderer->getObjectManager()->getModel("base")));

	//generateSpheres();
}


Scene::~Scene() {
	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != _rigidBodies.size(); i++) {
		delete _rigidBodies[i];
	}
}


void Scene::generateSpheres() {
	srand(time(NULL));
	for (int i = 0; i < NUM_SPHERES; i++) {
		vmml::Vector3f position = vmml::Vector3f(15.0 + ((rand() % 100) * 0.1), 1.f, -25.0 + ((rand() % 500) * 0.1));
		SphereRigidBody* sphere = new SphereRigidBody(_modelRenderer->getObjectManager()->getModel("sphere"), position);

		vmml::Vector3f velocity = vmml::Vector3f(-6.0 + ((rand() % 300) * 0.01), 0.0, -1.0 + ((rand() % 200) * 0.01));
		sphere->setVelocity(velocity);

		addRigidBody(sphere);
	}
}


void Scene::loop(const double &deltaTime, bool* running) {
	if (*running) {
		update(deltaTime);
	}
	draw();
}


void Scene::addRigidBody(ARigidBodyOctree* rigidBody) {
	rigidBody->updateMatrices();
	_rigidBodies.push_back(rigidBody);
	rigidBody->setOctTree(_octTree);
	rigidBody->registerInOctTree();
}


void Scene::update(const double &deltaTime) {
    
    //_sceneEditor->update(deltaTime);
    
	_solver->createConstraintCheckMatrix((int)_rigidBodies.size());
	_octTree->collide();
	_solver->assembleMatrices(_rigidBodies);
	_solver->solveForLambda((float) deltaTime, 8);
	_solver->computeNewVelocity((float) deltaTime, _rigidBodies);

	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != _rigidBodies.size(); i++) {
		_rigidBodies[i]->update(deltaTime);
	}
    
	
}


void Scene::draw() {
	_modelRenderer->drawQueue(/*GL_LINES*/);
	_modelRenderer->clearQueue();

	vmml::Matrix4f modelMatrix = vmml::create_translation(vmml::Vector3f(0.0f, 0.f, 0.0f)) * vmml::create_scaling(vmml::Vector3f(1.0f));
	_modelRenderer->queueModelInstance("base", "base", "camera", modelMatrix, std::vector<std::string>({ "sun", "moon" }), true, true);

	for (std::vector<ARigidBodyOctree*>::size_type i = 0; i != _rigidBodies.size(); i++) {
		_rigidBodies[i]->draw(_modelRenderer, (int) i);
	}
}


void Scene::registerSolverConstraint(ARigidBodyOctree* a, ARigidBodyOctree* b) {
	_solver->registerConstraint(a, b);
}
