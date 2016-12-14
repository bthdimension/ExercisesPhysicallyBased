#ifndef B_IRIGIDBODY_H
#define B_IRIGIDBODY_H

#include "headers/Model.h"
#include "vmmlib/util.hpp"
#include "headers/MeshCollider.h"

static const float m_G = 9.81;

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
    IRigidBody(ModelPtr model);
    
    /* Member functions */
    
    // this function depends on the RB
	virtual void update(const double &deltaTime) = 0;
    
    virtual bool doesIntersect(IRigidBody * rigidBody, vmml::Vector3f *minimumTranslationVector);
	
    // GET FUNCTIONS
	vmml::Vector3f getScale();

    float getMass();
	vmml::Matrix3f getMomentsOfInertia();
    
    vmml::Vector3f getPosition();
	vmml::Vector3f getAxesRotation();

    vmml::Vector3f getVelocity();
	vmml::Vector3f getAngularVelocity();
    
    vmml::Vector3f getForce();
    vmml::Vector3f getTorque();
    
    vmml::Matrix4f getRotation();
    vmml::Matrix4f getInverseRotation();
    
    vmml::Matrix4f getWorldMatrix();
    vmml::Matrix4f getInverseWorldMatrix();

    MeshColliderPtr getMeshCollider();
    
    // SET FUNCTIONS
	void setScale(vmml::Vector3f scale);

    void setPosition(vmml::Vector3f position);
	void setAxesRotation(vmml::Vector3f axesRotation);

    void setVelocity(vmml::Vector3f velocity);
    void setAngularVelocity(vmml::Vector3f angularVelocity);
    

	/**	@brief Updates all matrices of this actor based on the current values
	*/
    void updateMatrices();

    void updateWorldMatrix();
    void updateInverseWorldMatrix();

    void updateRotationMatrix();
    void updateInverseRotationMatrix();

	void updateMomentsOfInertia();
	

private:

	/* Variables */
	vmml::Vector3f _scale;

    float _mass;
	vmml::Matrix3f _momentsOfInertiaObject;
	vmml::Matrix3f _momentsOfInertiaWorld;

	vmml::Vector3f _position;
	vmml::Vector3f _axesRotation;
    
    vmml::Vector3f _velocity;
    vmml::Vector3f _angularVelocity;
    
    vmml::Vector3f _force;
    vmml::Vector3f _torque;
    
    MeshColliderPtr _meshCollider;
    
    vmml::Matrix4f _worldMatrix;
    vmml::Matrix4f _inverseWorldMatrix;
    vmml::Matrix4f _rotationMatrix;
    vmml::Matrix4f _inverseRotationMatrix;

};

typedef std::shared_ptr<IRigidBody> IRigidBodyPtr;


inline IRigidBody::IRigidBody(ModelPtr model) {
    _meshCollider = MeshColliderPtr(new MeshCollider(model, this));

	_scale = { 1.f, 1.f, 1.f };
    
    _mass    = 1.f;
    
    _velocity        = {0.f, 0.f, 0.f};
    _angularVelocity = {0.f, 0.f, 0.f};
    
    _force  = {0.f, 0.f, 0.f};
    _torque = {0.f, 0.f, 0.f};

	_momentsOfInertiaObject = vmml::Matrix3f(); // ONLY FOR BLOCKS UNTIL NOW, TODOD: Cover spheres too
	for (int i = 0; i < 2; i++) {
		_momentsOfInertiaObject(i, i) = _mass * 8.0f / 12.0f; // (m/12) * 2 * s^2
	}
	_momentsOfInertiaObject(2, 2) = _mass * 20.0f / 12.0f; // (m/12) * (s^2 + h^2)
    
}


inline bool IRigidBody::doesIntersect(IRigidBody* rigidBody, vmml::Vector3f *minimumTranslationVector) {
    return _meshCollider->doesIntersect(rigidBody->getMeshCollider().get(), minimumTranslationVector);
}


// SET FUNCTIONS

inline void IRigidBody::setScale(vmml::Vector3f scale) {
	_scale = scale;
}


inline void IRigidBody::setPosition(vmml::Vector3f position) {
    _position = position;
}


inline void IRigidBody::setAxesRotation(vmml::Vector3f axesRotation) {
	_axesRotation = axesRotation;
}


inline void IRigidBody::setVelocity(vmml::Vector3f velocity) {
    _velocity = velocity;
}


inline void IRigidBody::setAngularVelocity(vmml::Vector3f angularVelocity) {
    _angularVelocity = angularVelocity;
}


// GET FUNCTIONS
inline float IRigidBody::getMass() {
    return _mass;
}


inline vmml::Matrix3f IRigidBody::getMomentsOfInertia() {
	return _momentsOfInertiaWorld;
}


inline vmml::Vector3f IRigidBody::getPosition() {
    return _position;
}


inline vmml::Vector3f IRigidBody::getAxesRotation() {
	return _axesRotation;
}


inline vmml::Vector3f IRigidBody::getVelocity() {
    return _velocity;
}


inline vmml::Vector3f IRigidBody::getAngularVelocity() {
	return _angularVelocity;
}


inline vmml::Vector3f IRigidBody::getForce() {
    return _force;
}


inline vmml::Vector3f IRigidBody::getTorque() {
    return _torque;
}


inline vmml::Vector3f IRigidBody::getScale() {
    return _scale;
}


inline vmml::Matrix4f IRigidBody::getRotation() {
    return _rotationMatrix;
}


inline vmml::Matrix4f IRigidBody::getInverseRotation()	{
    return _inverseRotationMatrix;
}


inline vmml::Matrix4f IRigidBody::getWorldMatrix() {
    return _worldMatrix;
}


inline vmml::Matrix4f IRigidBody::getInverseWorldMatrix() {
    return _inverseWorldMatrix;
}


inline MeshColliderPtr IRigidBody::getMeshCollider() {
    return _meshCollider;
}

// UPDATE FUNCTIONS

inline void IRigidBody::updateMatrices() {
    updateRotationMatrix();
    updateInverseRotationMatrix();
    updateWorldMatrix();
    updateInverseWorldMatrix();
	updateMomentsOfInertia();
}


inline void IRigidBody::updateWorldMatrix(){
    _worldMatrix =  vmml::create_translation(getPosition()) * vmml::create_scaling(getScale()) * getRotation();
}


inline void IRigidBody::updateInverseWorldMatrix(){
	vmml::Matrix4f inverse; inverse.inverse(getWorldMatrix()); inverse.transpose_to(inverse);
	_inverseWorldMatrix = inverse;//getInverseRotation() * vmml::create_scaling(vmml::Vector3f(1.f / _scale.x(), 1.f / _scale.y(), 1.f / _scale.z())) * vmml::create_translation(-getPosition());
}


inline void IRigidBody::updateRotationMatrix(){
    _rotationMatrix = vmml::create_rotation(_axesRotation.x(), vmml::Vector3f::UNIT_X) * vmml::create_rotation(_axesRotation.y(), vmml::Vector3f::UNIT_Y) * vmml::create_rotation(_axesRotation.z(), vmml::Vector3f::UNIT_Z);
}


inline void IRigidBody::updateInverseRotationMatrix(){
	_inverseRotationMatrix = getRotation();
	_inverseRotationMatrix.transpose_to(_inverseRotationMatrix);
}


inline void IRigidBody::updateMomentsOfInertia() {
	_momentsOfInertiaWorld = vmml::Matrix3f(_inverseRotationMatrix) * _momentsOfInertiaObject * vmml::Matrix3f(_rotationMatrix); // = R^T * I(object) * R (world to body)
}


#endif /* defined(B_IRIGIDBODY_H) */

