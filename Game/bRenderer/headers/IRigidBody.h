#ifndef B_IRIGIDBODY_H
#define B_IRIGIDBODY_H

#include "headers/Model.h"
#include "vmmlib/util.hpp"
#include "headers/MeshCollider.h"
/** @brief Rigid Body based on a model
*	@author Benjamin Buergisser
*/
class IRigidBody
{
public:

	/* Functions */

	/**	@brief Constructor creating a rigid body from a model
	*	@param[in] model
	*/
	IRigidBody(ModelPtr model) {
		_meshCollider = MeshColliderPtr(new MeshCollider(model, this));
	}

	virtual void update(const double &deltaTime) = 0;

	virtual bool doesIntersect(IRigidBody* rigidBody)
	{
		return _meshCollider->doesIntersect(rigidBody->getMeshCollider().get());
	}

	void setPosition(vmml::Vector3f position) { _position = position; }
	void setRotationAxes(vmml::Vector3f rotationAxes) { _rotationAxes = rotationAxes; }
	void setScale(vmml::Vector3f scale) { _scale = scale; }
	void setVelocity(vmml::Vector3f velocity) { _velocity = velocity; }

	vmml::Vector3f getPosition() { return _position; }
	vmml::Vector3f getRotationAxes() { return _rotationAxes; }
	vmml::Vector3f getScale() { return _scale; }
	vmml::Vector3f getVelocity() { return _velocity; }

	vmml::Matrix4f getRotation() { return _rotationMatrix; }
	vmml::Matrix4f getInverseRotation()	{ return _inverseRotationMatrix; }

	vmml::Matrix4f getWorldMatrix() { return _worldMatrix; }
	vmml::Matrix4f getInverseWorldMatrix() { return _inverseWorldMatrix; }

	MeshColliderPtr getMeshCollider() { return _meshCollider; }

	/**	@brief Updates all matrices of this actor based on the current values
	*/
	void updateMatrices() {
		updateRotationMatrix();
		updateInverseRotationMatrix();
		updateWorldMatrix();
		updateInverseWorldMatrix();
	}

	void updateWorldMatrix()
	{
		_worldMatrix = vmml::create_scaling(getScale()) * getRotation() * vmml::create_translation(getPosition());
	}

	void updateInverseWorldMatrix()
	{
		_inverseWorldMatrix = vmml::create_translation(-getPosition()) * getInverseRotation() * vmml::create_scaling(vmml::Vector3f(1.f / _scale.x(), 1.f / _scale.y(), 1.f / _scale.z()));
	}

	void updateRotationMatrix()
	{
		_rotationMatrix = vmml::create_rotation(_rotationAxes.x(), vmml::Vector3f::UNIT_X) * vmml::create_rotation(_rotationAxes.y(), vmml::Vector3f::UNIT_Y) * vmml::create_rotation(_rotationAxes.z(), vmml::Vector3f::UNIT_Z);
	}

	void updateInverseRotationMatrix()
	{
		getRotation().transpose_to(_inverseRotationMatrix);
	}

private:

	/* Variables */
	MeshColliderPtr _meshCollider;
	vmml::Vector3f _position = {};
	vmml::Vector3f _rotationAxes = {};
	vmml::Vector3f _scale = {1.f,1.f,1.f};

	vmml::Vector3f _velocity = {0.f,0.f,0.f};

	vmml::Matrix4f _worldMatrix = {};
	vmml::Matrix4f _inverseWorldMatrix = {};
	vmml::Matrix4f _rotationMatrix = {};
	vmml::Matrix4f _inverseRotationMatrix = {};
};

typedef std::shared_ptr<IRigidBody> IRigidBodyPtr;

#endif /* defined(B_IRIGIDBODY_H) */

