#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Math/MathTypes.h>

#include <Jolt/Physics/Character/CharacterBase.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>

/// Contains the configuration of a character
class UNREALJOLT_API MyCharacterSettings : public JPH::CharacterBaseSettings
{
public:
	JPH_OVERRIDE_NEW_DELETE
	/// Constructor
	MyCharacterSettings() = default;
	MyCharacterSettings(const MyCharacterSettings&) = default;
	MyCharacterSettings& operator =(const MyCharacterSettings&) = default;

	/// Layer that this character will be added to
	JPH::ObjectLayer mLayer = 0;

	/// Mass of the character
	float mMass = 80.0f;

	/// Friction for the character
	float mFriction = 0.2f;

	/// Value to multiply gravity with for this character
	float mGravityFactor = 1.0f;

	/// Allowed degrees of freedom for this character
	JPH::EAllowedDOFs mAllowedDOFs = JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY |
		JPH::EAllowedDOFs::TranslationZ;

	JPH::Ref<JPH::Shape> mCrouchingShape;
};

/// Runtime character object.
/// This object usually represents the player or a humanoid AI. It uses a single rigid body,
/// usually with a capsule shape to simulate movement and collision for the character.
/// The character is a keyframed object, the application controls it by setting the velocity.
class UNREALJOLT_API MyCharacter : public JPH::CharacterBase
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
	/// @param inSettings The settings for the character
	/// @param inPosition Initial position for the character
	/// @param inRotation Initial rotation for the character (usually only around Y)
	/// @param inUserData Application specific value
	/// @param inSystem Physics system that this character will be added to later
	MyCharacter(
		const MyCharacterSettings* inSettings, JPH::RVec3Arg inPosition,
		JPH::QuatArg               inRotation, uint64        inUserData,
		JPH::PhysicsSystem*        inSystem);

	/// Destructor
	virtual ~MyCharacter() override;

	/// Add bodies and constraints to the system and optionally activate the bodies
	void AddToPhysicsSystem(JPH::EActivation inActivationMode = JPH::EActivation::Activate, bool inLockBodies = true);

	/// Remove bodies and constraints from the system
	void RemoveFromPhysicsSystem(bool inLockBodies = true);

	/// Wake up the character
	void Activate(bool inLockBodies = true);

	/// Control the velocity of the character
	void SetLinearAndAngularVelocity(
		JPH::Vec3Arg inLinearVelocity, JPH::Vec3Arg inAngularVelocity, bool inLockBodies = true);

	/// Get the linear velocity of the character (m / s)
	JPH::Vec3 GetLinearVelocity(bool inLockBodies = true) const;

	/// Set the linear velocity of the character (m / s)
	void SetLinearVelocity(JPH::Vec3Arg inLinearVelocity, bool inLockBodies = true);

	/// Add world space linear velocity to current velocity (m / s)
	void AddLinearVelocity(JPH::Vec3Arg inLinearVelocity, bool inLockBodies = true);

	/// Add impulse to the center of mass of the character
	void AddImpulse(JPH::Vec3Arg inImpulse, bool inLockBodies = true);

	/// Get the body associated with this character
	JPH::BodyID GetBodyID() const { return mBodyID; }

	/// Get position / rotation of the body
	void GetPositionAndRotation(JPH::RVec3& outPosition, JPH::Quat& outRotation, bool inLockBodies = true) const;

	/// Set the position / rotation of the body, optionally activating it.
	void SetPositionAndRotation(
		JPH::RVec3Arg    inPosition, JPH::QuatArg inRotation,
		JPH::EActivation inActivationMode = JPH::EActivation::Activate,
		bool             inLockBodies = true) const;

	/// Get the position of the character
	JPH::RVec3 GetPosition(bool inLockBodies = true) const;

	/// Set the position of the character, optionally activating it.
	void SetPosition(JPH::RVec3Arg inPosition, JPH::EActivation inActivationMode = JPH::EActivation::Activate,
		bool                       inLockBodies = true);

	/// Get the rotation of the character
	JPH::Quat GetRotation(bool inLockBodies = true) const;

	/// Set the rotation of the character, optionally activating it.
	void SetRotation(
		JPH::QuatArg inRotation, JPH::EActivation inActivationMode = JPH::EActivation::Activate,
		bool         inLockBodies = true);

	/// Position of the center of mass of the underlying rigid body
	JPH::RVec3 GetCenterOfMassPosition(bool inLockBodies = true) const;

	/// Calculate the world transform of the character
	JPH::RMat44 GetWorldTransform(bool inLockBodies = true) const;

	/// Get the layer of the character
	JPH::ObjectLayer GetLayer() const { return mLayer; }

	/// Update the layer of the character
	void SetLayer(JPH::ObjectLayer inLayer, bool inLockBodies = true);

	/// Switch the shape of the character (e.g. for stance). When inMaxPenetrationDepth is not FLT_MAX, it checks
	/// if the new shape collides before switching shape. Returns true if the switch succeeded.
	bool SetShape(const JPH::Shape* inShape, float inMaxPenetrationDepth, bool inLockBodies = true);

	/// Get the transformed shape that represents the volume of the character, can be used for collision checks.
	JPH::TransformedShape GetTransformedShape(bool inLockBodies = true) const;

	/// @brief Get all contacts for the character at a particular location
	/// @param inPosition Position to test.
	/// @param inRotation Rotation at which to test the shape.
	/// @param inMovementDirection A hint in which direction the character is moving, will be used to calculate a proper normal.
	/// @param inMaxSeparationDistance How much distance around the character you want to report contacts in (can be 0 to match the character exactly).
	/// @param inShape Shape to test collision with.
	/// @param inBaseOffset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
	/// @param ioCollector Collision collector that receives the collision results.
	/// @param inLockBodies If the collision query should use the locking body interface (true) or the non locking body interface (false)
	void CheckCollision(
		JPH::RVec3Arg               inPosition, JPH::QuatArg                   inRotation, JPH::Vec3Arg inMovementDirection,
		float                       inMaxSeparationDistance, const JPH::Shape* inShape, JPH::RVec3Arg   inBaseOffset,
		JPH::CollideShapeCollector& ioCollector, bool                          inLockBodies = true) const;

	/// Get the character settings that can recreate this character
	MyCharacterSettings GetCharacterSettings(bool inLockBodies = true) const;


	float GetGroundPenetrationDepth() const { return mGroundPenetrationDepth; }

	JPH::Vec3 GetStepDown() const { return mStepDown; }

	bool StickToGround();

	void UpdateGravity();

	void DetachFromGround();

	JPH::Vec3 mStepDown = JPH::Vec3(0, -.5f, 0); // Step down distance when checking for ground contact

private:
	/// Check collisions between inShape and the world using the center of mass transform
	void CheckCollision(
		JPH::RMat44Arg    inCenterOfMassTransform, JPH::Vec3Arg inMovementDirection,
		float             inMaxSeparationDistance,
		const JPH::Shape* inShape, JPH::RVec3Arg inBaseOffset, JPH::CollideShapeCollector& ioCollector,
		bool              inLockBodies) const;

	/// Check collisions between inShape and the world using the current position / rotation of the character
	void CheckCollision(
		const JPH::Shape*           inShape, float inMaxSeparationDistance,
		JPH::RVec3Arg               inBaseOffset,
		JPH::CollideShapeCollector& ioCollector, bool inLockBodies) const;

	/// The body of this character
	JPH::BodyID mBodyID;

	/// The layer the body is in
	JPH::ObjectLayer mLayer;
	float            mGroundPenetrationDepth = 0.0f;
	float            mPlaceAboveGroundDist = .05f;
};