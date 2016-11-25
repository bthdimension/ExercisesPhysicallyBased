#ifndef SETTINGS_CONTROLLER_H
#define SETTINGS_CONTROLLER_H

#include "bRenderer.h"

class SettingsController {

public:

	SettingsController(Renderer* bRenderer);

	void setup();


private:

	Renderer* _bRenderer;

};


typedef std::shared_ptr< SettingsController >  SettingsControllerPtr;


#endif