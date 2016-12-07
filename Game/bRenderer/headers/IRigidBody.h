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
    
    virtual bool doesIntersect(IRigidBody * rigidBody);
	
    // GET FUNCTIONS
    float getMass();
    vmml::Matrix3f getInertia();
    
    vmml::Vector3f getPosition();
    vmml::Vector3f getVelocity();
    
    vmml::Vector3f getRotationAxes();
    vmml::Vector3f getScale();
    
    
    vmml::Vector3f getForce();
    vmml::Vector3f getTorque();
    
    vmml::Matrix4f getRotation();
    vmml::Matrix4f getInverseRotation();
    
    vmml::Matrix4f getWorldMatrix();
    vmml::Matrix4f getInverseWorldMatrix();
    
    MeshColliderPtr getMeshCollider();
    
    // SET FUNCTIONS
    void setPosition(vmml::Vector3f position);
    void setVelocity(vmml::Vector3f velocity);
    void setAngularVelocity(vmml::Vector3f angularVelocity); // angular velocity
    
    void setRotationAxes(vmml::Vector3f rotationAxes);
    void setScale(vmml::Vector3f scale);

	/**	@brief Updates all matrices of this actor based on the current values
	*/
    void updateMatrices();

    void updateWorldMatrix();
    void updateInverseWorldMatrix();

    void updateRotationMatrix();
	
    void updateInverseRotationMatrix();
	

private:

	/* Variables */
    float _mass;
    float _invMass;
    
    vmml::Matrix3f _inertia;
    vmml::Matrix3f _invInertia;
    
    vmml::Vector3f _position;
    // linear velocity
    vmml::Vector3f _velocity;
    // angular velocity
    vmml::Vector3f _angularVelocity;
    
    vmml::Vector3f _force;
    vmml::Vector3f _torque;
    
    MeshColliderPtr _meshCollider;
    vmml::Vector3f _rotationAxes;
    vmml::Vector3f _scale;

    vmml::Matrix4f _worldMatrix;
    vmml::Matrix4f _inverseWorldMatrix;
    vmml::Matrix4f _rotationMatrix;
    vmml::Matrix4f _inverseRotationMatrix;
};

typedef std::shared_ptr<IRigidBody> IRigidBodyPtr;


inline IRigidBody::IRigidBody(ModelPtr model) {
    _meshCollider = MeshColliderPtr(new MeshCollider(model, this));
    
    _mass    = 0.f;
    _invMass = 0.f;
    
    _inertia.zero();
    _invInertia.zero();
    
    _velocity        = {0.f, 0.f, 0.f};
    _angularVelocity = {0.f, 0.f, 0.f};
    
    _force  = {0.f, 0.f, 0.f};
    _torque = {0.f, 0.f, 0.f};
    
    _scale  = {1.f, 1.f, 1.f};
    
    
}

inline bool IRigidBody::doesIntersect(IRigidBody* rigidBody) {
    return _meshCollider->doesIntersect(rigidBody->getMeshCollider().get());
}

// SET FUNCTIONS
inline void IRigidBody::setPosition(vmml::Vector3f position) {
    _position = position;
}

inline void IRigidBody::setVelocity(vmml::Vector3f velocity) {
    _velocity = velocity;
}

inline void IRigidBody::setAngularVelocity(vmml::Vector3f angularVelocity) {
    _angularVelocity = angularVelocity;
}


inline void IRigidBody::setRotationAxes(vmml::Vector3f rotationAxes) {
    _rotationAxes = rotationAxes;
}

inline void IRigidBody::setScale(vmml::Vector3f scale) {
    _scale = scale;
}


// GET FUNCTIONS
inline float IRigidBody::getMass() {
    return _mass;
}

inline vmml::Matrix3f IRigidBody::getInertia() {
    return _inertia;
}

inline vmml::Vector3f IRigidBody::getPosition() {
    return _position;
}

inline vmml::Vector3f IRigidBody::getVelocity() {
    return _velocity;
}


inline vmml::Vector3f IRigidBody::getForce() {
    return _force;
}

inline vmml::Vector3f IRigidBody::getTorque() {
    return _torque;
}

inline vmml::Vector3f IRigidBody::getRotationAxes() {
    return _rotationAxes;
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
}

inline void IRigidBody::updateWorldMatrix(){
    _worldMatrix =  vmml::create_translation(getPosition()) * vmml::create_scaling(getScale()) * getRotation();
}

inline void IRigidBody::updateInverseWorldMatrix(){
    _inverseWorldMatrix = getInverseRotation() * vmml::create_scaling(vmml::Vector3f(1.f / _scale.x(), 1.f / _scale.y(), 1.f / _scale.z())) * vmml::create_translation(-getPosition());
}

inline void IRigidBody::updateRotationMatrix(){
    _rotationMatrix = vmml::create_rotation(_rotationAxes.x(), vmml::Vector3f::UNIT_X) * vmml::create_rotation(_rotationAxes.y(), vmml::Vector3f::UNIT_Y) * vmml::create_rotation(_rotationAxes.z(), vmml::Vector3f::UNIT_Z);
}

inline void IRigidBody::updateInverseRotationMatrix(){
    getRotation().transpose_to(_inverseRotationMatrix);
}


#endif /* defined(B_IRIGIDBODY_H) */

