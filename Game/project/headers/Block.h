#ifndef BLOCK_H
#define BLOCK_H

#include "bRenderer.h"
#include "ARigidBody.h"


class Block : public ARigidBody {

public:

	Block(vmml::Vector3f position);

	int getType();

	bool isInAABB(double x0, double x1, double y0, double y1, double z0, double z1);

	void update(const double &deltaTime);
	void draw(ModelRendererPtr modelRenderer, int id);


private:

	vmml::Vector3f _position;

};


#endif
