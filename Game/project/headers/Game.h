#ifndef GAME_H
#define GAME_H

#include "bRenderer.h"
#include "SettingsController.h"
#include "ResourceManager.h"
#include "FreeCamera.h"
#include "Scene.h"

class Game : public IRenderProject {

public:

	Game() : IRenderProject() {}
    virtual ~Game(){ bRenderer::log("Game deleted"); }

	void init();

	void initFunction();

	void loopFunction(const double &deltaTime, const double &elapsedTime);

	void terminateFunction();
 
	void showFPS(const double &deltaTime);
    
private:

	SettingsControllerPtr _settingsController;
	ResourceManagerPtr _resourceManager;
	InputControllerPtr _inputController;

	FreeCameraPtr _freeCamera;
	ScenePtr _scene;


private:

	bool _running = false; 
	int framesPerSec=0;
	double fpsTime=0;
};

#endif
