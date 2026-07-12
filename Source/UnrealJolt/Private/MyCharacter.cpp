#include "MyCharacter.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Math/MathTypes.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>

#include "Jolt/Renderer/DebugRenderer.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

// Collector that finds the hit with the normal that is the most 'up'
class MyCollector : public JPH::CollideShapeCollector
{
public:
	// Constructor
	explicit MyCollector(JPH::Vec3Arg inUp, JPH::RVec3 inBaseOffset)
		: mBaseOffset(inBaseOffset)
		, mUp(inUp) {}

	// See: CollectorType::AddHit
	virtual void AddHit(const JPH::CollideShapeResult& inResult) override
	{
		JPH::Vec3 normal = -inResult.mPenetrationAxis.Normalized();
		float     dot = normal.Dot(mUp);
		auto      mBaseOffsetXZ = JPH::Vec3(mBaseOffset.GetX(), 0, mBaseOffset.GetZ());
		auto      contactPointXZ = JPH::Vec3(inResult.mContactPointOn2.GetX(), 0, inResult.mContactPointOn2.GetZ());
		float     distFromPosition = (mBaseOffsetXZ - contactPointXZ).Length();
		if (dot > mBestDot) // Find the hit that is most aligned with the up vector
		{
			mGroundBodyID = inResult.mBodyID2;
			mGroundBodySubShapeID = inResult.mSubShapeID2;
			mGroundPosition = mBaseOffset + inResult.mContactPointOn2;
			mGroundNormal = normal;
			mBestDot = dot;
			mPenetrationDepth = inResult.mPenetrationDepth;
		}
	}

	JPH::BodyID     mGroundBodyID;
	JPH::SubShapeID mGroundBodySubShapeID;
	JPH::RVec3      mGroundPosition = JPH::RVec3::sZero();
	JPH::Vec3       mGroundNormal = JPH::Vec3::sZero();
	float           mPenetrationDepth = 0.0f;

private:
	JPH::RVec3 mBaseOffset;
	JPH::Vec3  mUp;
	float      mBestDot = -FLT_MAX;
};

class CharacterBodyFilter : public JPH::IgnoreSingleBodyFilter
{
public:
	using IgnoreSingleBodyFilter::IgnoreSingleBodyFilter;

	virtual bool ShouldCollideLocked(const JPH::Body& inBody) const override
	{
		return !inBody.IsSensor();
	}
};

static inline const JPH::BodyLockInterface& sCharacterGetBodyLockInterface(
	const JPH::PhysicsSystem* inSystem, bool inLockBodies)
{
	return inLockBodies
		? static_cast<const JPH::BodyLockInterface&>(inSystem->GetBodyLockInterface())
		: static_cast<const JPH::BodyLockInterface&>(inSystem->GetBodyLockInterfaceNoLock());
}

static inline JPH::BodyInterface& sCharacterGetBodyInterface(JPH::PhysicsSystem* inSystem, bool inLockBodies)
{
	return inLockBodies ? inSystem->GetBodyInterface() : inSystem->GetBodyInterfaceNoLock();
}

static inline const JPH::NarrowPhaseQuery& sCharacterGetNarrowPhaseQuery(const JPH::PhysicsSystem* inSystem,
	bool                                                                                           inLockBodies)
{
	return inLockBodies ? inSystem->GetNarrowPhaseQuery() : inSystem->GetNarrowPhaseQueryNoLock();
}

MyCharacter::MyCharacter(
	const MyCharacterSettings* inSettings, JPH::RVec3Arg       inPosition, JPH::QuatArg inRotation,
	uint64                     inUserData, JPH::PhysicsSystem* inSystem)
	: CharacterBase(inSettings, inSystem)
	, mLayer(inSettings->mLayer)
{
	// Construct rigid body
	mShape = inSettings->mShape;
	JPH::BodyCreationSettings settings(
		mShape, inPosition, inRotation, JPH::EMotionType::Dynamic, mLayer);
	settings.mAllowedDOFs = inSettings->mAllowedDOFs;
	settings.mEnhancedInternalEdgeRemoval = inSettings->mEnhancedInternalEdgeRemoval;
	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
	settings.mMassPropertiesOverride.mMass = inSettings->mMass;
	settings.mFriction = inSettings->mFriction;
	settings.mGravityFactor = inSettings->mGravityFactor;
	settings.mUserData = inUserData;
	const JPH::Body* body = mSystem->GetBodyInterface().CreateBody(settings);
	if (body != nullptr)
		mBodyID = body->GetID();
}

MyCharacter::~MyCharacter()
{
	// Destroy the body
	mSystem->GetBodyInterface().DestroyBody(mBodyID);
}

void MyCharacter::AddToPhysicsSystem(JPH::EActivation inActivationMode, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).AddBody(mBodyID, inActivationMode);
}

void MyCharacter::RemoveFromPhysicsSystem(bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).RemoveBody(mBodyID);
}

void MyCharacter::Activate(bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).ActivateBody(mBodyID);
}

bool MyCharacter::StickToGround()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(MyCharacter::StickToGround);

	// Get character position and rotation for trace checks
	JPH::RVec3 char_pos;
	JPH::Vec3  char_vel;
	JPH::Quat  char_rot;
	{
		JPH::BodyLockRead lock(sCharacterGetBodyLockInterface(mSystem, false), mBodyID);
		if (!lock.Succeeded())
			return false;
		const JPH::Body& body = lock.GetBody();
		char_pos = body.GetPosition();
		char_vel = body.GetLinearVelocity();
		char_rot = body.GetRotation();
	}

	// Create query filters
	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = mSystem->GetDefaultBroadPhaseLayerFilter(mLayer);
	JPH::DefaultObjectLayerFilter     object_layer_filter = mSystem->GetDefaultLayerFilter(mLayer);

	auto PrevGroundState = GetGroundState();
	if (GetGroundState() == EGroundState::OnGround || GetGroundState() == EGroundState::OnSteepGround)
	{
		// Ray cast from above the character feet position to below feet position
		// to keep the character grounded and resolve any step up/downs
		JPH::RayCastResult hit;
		JPH::RRayCast      ray(char_pos + mStepDown * .5f, -mStepDown);
		sCharacterGetNarrowPhaseQuery(mSystem, false).CastRay(
			ray, hit, broadphase_layer_filter, object_layer_filter, CharacterBodyFilter(mBodyID));

		mGroundBodyID = hit.mBodyID;
		mGroundBodySubShapeID = hit.mSubShapeID2;
		mGroundPosition = ray.GetPointOnRay(hit.mFraction);
		mGroundPenetrationDepth = 0.0f;
	}
	else
	{
		// Don't use the step up/down ray trace here to keep the character on the ground,
		// as we've either jumped or are falling, 
		// so collide capsule until we are back on the ground.
		MyCollector collector(mUp, char_pos);
		CheckCollision(
			char_pos, char_rot, char_vel, mPlaceAboveGroundDist, mShape,
			char_pos, collector, false);

		mGroundBodyID = collector.mGroundBodyID;
		mGroundBodySubShapeID = collector.mGroundBodySubShapeID;
		mGroundPosition = collector.mGroundPosition;
		mGroundNormal = collector.mGroundNormal;
		mGroundPenetrationDepth = collector.mPenetrationDepth;
	}

	JPH::BodyLockRead groundLock(
		sCharacterGetBodyLockInterface(mSystem, false), mGroundBodyID);
	if (!groundLock.Succeeded())
	{
		// Hit result is not valid, so we are falling
		DetachFromGround();
		return false;
	}

	const JPH::Body& groundBody = groundLock.GetBody();
	mGroundNormal = groundBody.GetWorldSpaceSurfaceNormal(mGroundBodySubShapeID, mGroundPosition);
	mGroundMaterial = groundBody.GetShape()->GetMaterial(mGroundBodySubShapeID);
	mGroundVelocity = groundBody.GetPointVelocity(mGroundPosition);
	mGroundUserData = groundBody.GetUserData();

	// Update ground state
	JPH::RMat44 inv_transform = JPH::RMat44::sInverseRotationTranslation(char_rot, char_pos);
	if (mSupportingVolume.SignedDistance(JPH::Vec3(inv_transform * mGroundPosition)) > 0.0f)
		mGroundState = EGroundState::NotSupported;
	else if (IsSlopeTooSteep(mGroundNormal))
		mGroundState = EGroundState::OnSteepGround;
	else
		mGroundState = EGroundState::OnGround;

	// Update gravity after ground state
	UpdateGravity();

	// Set position to ground contact point with a bit of buffer so the physics doesn't
	// try to push the character out of the ground on slopes
	if (mGroundState == EGroundState::OnGround || mGroundState == EGroundState::OnSteepGround)
		SetPosition(mGroundPosition + JPH::Vec3(0, mPlaceAboveGroundDist, 0));

	return true;
}

void MyCharacter::UpdateGravity()
{
	// Turn on/off gravity if supported
	auto&             BodyLockInterface = sCharacterGetBodyLockInterface(mSystem, false);
	JPH::BodyLockRead selfLock(BodyLockInterface, GetBodyID());
	if (selfLock.Succeeded())
	{
		JPH::Body& selfBody = const_cast<JPH::Body&>(selfLock.GetBody());

		if (auto MotionProperties = selfBody.GetMotionProperties())
		{
			bool  bApplyGravity = mGroundState != EGroundState::OnGround;
			float GravityFactor = MotionProperties->GetGravityFactor();
			if (!bApplyGravity && GravityFactor != 0.f)
				MotionProperties->SetGravityFactor(0.f);
			else if (bApplyGravity && GravityFactor == 0.f)
				MotionProperties->SetGravityFactor(1.f);
		}
	}
}

void MyCharacter::DetachFromGround()
{
	mGroundState = EGroundState::InAir;
	mGroundMaterial = JPH::PhysicsMaterial::sDefault;
	mGroundVelocity = JPH::Vec3::sZero();
	mGroundUserData = 0;
	UpdateGravity();
}

void MyCharacter::CheckCollision(
	JPH::RMat44Arg              inCenterOfMassTransform, JPH::Vec3Arg      inMovementDirection,
	float                       inMaxSeparationDistance, const JPH::Shape* inShape, JPH::RVec3Arg inBaseOffset,
	JPH::CollideShapeCollector& ioCollector, bool                          inLockBodies) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(MyCharacter::CheckCollision);
	// Create query broadphase layer filter
	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = mSystem->GetDefaultBroadPhaseLayerFilter(mLayer);

	// Create query object layer filter
	JPH::DefaultObjectLayerFilter object_layer_filter = mSystem->GetDefaultLayerFilter(mLayer);

	// Ignore sensors and my own body
	class CharacterBodyFilter : public JPH::IgnoreSingleBodyFilter
	{
	public:
		using IgnoreSingleBodyFilter::IgnoreSingleBodyFilter;

		virtual bool ShouldCollideLocked(const JPH::Body& inBody) const override
		{
			return !inBody.IsSensor();
		}
	};
	CharacterBodyFilter body_filter(mBodyID);

	// Settings for collide shape
	JPH::CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inMaxSeparationDistance;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = inMovementDirection;
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;

	sCharacterGetNarrowPhaseQuery(mSystem, inLockBodies).CollideShape(
		inShape, JPH::Vec3::sOne(), inCenterOfMassTransform,
		settings, inBaseOffset, ioCollector,
		broadphase_layer_filter, object_layer_filter, body_filter);
}

void MyCharacter::CheckCollision(
	JPH::RVec3Arg               inPosition, JPH::QuatArg                   inRotation, JPH::Vec3Arg inMovementDirection,
	float                       inMaxSeparationDistance, const JPH::Shape* inShape, JPH::RVec3Arg   inBaseOffset,
	JPH::CollideShapeCollector& ioCollector, bool                          inLockBodies) const
{
	// Calculate center of mass transform
	JPH::RMat44 center_of_mass = JPH::RMat44::sRotationTranslation(inRotation, inPosition).PreTranslated(
		inShape->GetCenterOfMass());

	CheckCollision(
		center_of_mass, inMovementDirection, inMaxSeparationDistance, inShape,
		inBaseOffset, ioCollector, inLockBodies);
}

void MyCharacter::CheckCollision(
	const JPH::Shape*           inShape, float    inMaxSeparationDistance, JPH::RVec3Arg inBaseOffset,
	JPH::CollideShapeCollector& ioCollector, bool inLockBodies) const
{
	// Determine position and velocity of body
	JPH::RMat44 query_transform;
	JPH::Vec3   velocity;
	{
		JPH::BodyLockRead lock(sCharacterGetBodyLockInterface(mSystem, inLockBodies), mBodyID);
		if (!lock.Succeeded())
			return;

		const JPH::Body& body = lock.GetBody();

		// Correct the center of mass transform for the difference between the old and new center of mass shape
		query_transform = body.GetCenterOfMassTransform().PreTranslated(
			inShape->GetCenterOfMass() - mShape->GetCenterOfMass());
		velocity = body.GetLinearVelocity();
	}

	CheckCollision(
		query_transform, velocity, inMaxSeparationDistance, inShape, inBaseOffset,
		ioCollector, inLockBodies);
}

void MyCharacter::SetLinearAndAngularVelocity(
	JPH::Vec3Arg inLinearVelocity, JPH::Vec3Arg inAngularVelocity, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).SetLinearAndAngularVelocity(
		mBodyID, inLinearVelocity, inAngularVelocity);
}

JPH::Vec3 MyCharacter::GetLinearVelocity(bool inLockBodies) const
{
	return sCharacterGetBodyInterface(mSystem, inLockBodies).GetLinearVelocity(mBodyID);
}

void MyCharacter::SetLinearVelocity(JPH::Vec3Arg inLinearVelocity, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).SetLinearVelocity(mBodyID, inLinearVelocity);
}

void MyCharacter::AddLinearVelocity(JPH::Vec3Arg inLinearVelocity, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).AddLinearVelocity(mBodyID, inLinearVelocity);
}

void MyCharacter::AddImpulse(JPH::Vec3Arg inImpulse, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).AddImpulse(mBodyID, inImpulse);
}

void MyCharacter::GetPositionAndRotation(
	JPH::RVec3& outPosition, JPH::Quat& outRotation, bool inLockBodies) const
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).GetPositionAndRotation(mBodyID, outPosition, outRotation);
}

void MyCharacter::SetPositionAndRotation(
	JPH::RVec3Arg inPosition, JPH::QuatArg inRotation, JPH::EActivation inActivationMode,
	bool          inLockBodies) const
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).SetPositionAndRotation(
		mBodyID, inPosition, inRotation, inActivationMode);
}

JPH::RVec3 MyCharacter::GetPosition(bool inLockBodies) const
{
	return sCharacterGetBodyInterface(mSystem, inLockBodies).GetPosition(mBodyID);
}

void MyCharacter::SetPosition(
	JPH::RVec3Arg inPosition, JPH::EActivation inActivationMode, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).SetPosition(mBodyID, inPosition, inActivationMode);
}

JPH::Quat MyCharacter::GetRotation(bool inLockBodies) const
{
	return sCharacterGetBodyInterface(mSystem, inLockBodies).GetRotation(mBodyID);
}

void MyCharacter::SetRotation(JPH::QuatArg inRotation, JPH::EActivation inActivationMode, bool inLockBodies)
{
	sCharacterGetBodyInterface(mSystem, inLockBodies).SetRotation(mBodyID, inRotation, inActivationMode);
}

JPH::RVec3 MyCharacter::GetCenterOfMassPosition(bool inLockBodies) const
{
	return sCharacterGetBodyInterface(mSystem, inLockBodies).GetCenterOfMassPosition(mBodyID);
}

JPH::RMat44 MyCharacter::GetWorldTransform(bool inLockBodies) const
{
	return sCharacterGetBodyInterface(mSystem, inLockBodies).GetWorldTransform(mBodyID);
}

void MyCharacter::SetLayer(JPH::ObjectLayer inLayer, bool inLockBodies)
{
	mLayer = inLayer;

	sCharacterGetBodyInterface(mSystem, inLockBodies).SetObjectLayer(mBodyID, inLayer);
}

bool MyCharacter::SetShape(const JPH::Shape* inShape, float inMaxPenetrationDepth, bool inLockBodies)
{
	if (inMaxPenetrationDepth < FLT_MAX)
	{
		// Collector that checks if there is anything in the way while switching to inShape
		class MyCollector : public JPH::CollideShapeCollector
		{
		public:
			// Constructor
			explicit MyCollector(float inMaxPenetrationDepth)
				: mMaxPenetrationDepth(inMaxPenetrationDepth) {}

			// See: CollectorType::AddHit
			virtual void AddHit(const JPH::CollideShapeResult& inResult) override
			{
				if (inResult.mPenetrationDepth > mMaxPenetrationDepth)
				{
					mHadCollision = true;
					ForceEarlyOut();
				}
			}

			float mMaxPenetrationDepth;
			bool  mHadCollision = false;
		};

		// Test if anything is in the way of switching
		JPH::RVec3  char_pos = GetPosition(inLockBodies);
		MyCollector collector(inMaxPenetrationDepth);
		CheckCollision(inShape, 0.01f, char_pos, collector, inLockBodies);
		if (collector.mHadCollision)
			return false;
	}

	// Switch the shape
	mShape = inShape;
	sCharacterGetBodyInterface(mSystem, inLockBodies).SetShape(mBodyID, mShape, false, JPH::EActivation::Activate);
	return true;
}

JPH::TransformedShape MyCharacter::GetTransformedShape(bool inLockBodies) const
{
	return sCharacterGetBodyInterface(mSystem, inLockBodies).GetTransformedShape(mBodyID);
}

MyCharacterSettings MyCharacter::GetCharacterSettings(bool inLockBodies) const
{
	JPH::BodyLockRead lock(sCharacterGetBodyLockInterface(mSystem, inLockBodies), mBodyID);
	JPH_ASSERT(lock.Succeeded());
	const JPH::Body& body = lock.GetBody();

	MyCharacterSettings settings;
	settings.mUp = mUp;
	settings.mSupportingVolume = mSupportingVolume;
	settings.mMaxSlopeAngle = JPH::ACos(mCosMaxSlopeAngle);
	settings.mEnhancedInternalEdgeRemoval = body.GetEnhancedInternalEdgeRemoval();
	settings.mShape = mShape;
	settings.mLayer = mLayer;
	const JPH::MotionProperties* mp = body.GetMotionProperties();
	settings.mMass = 1.0f / mp->GetInverseMass();
	settings.mFriction = body.GetFriction();
	settings.mGravityFactor = mp->GetGravityFactor();
	settings.mAllowedDOFs = mp->GetAllowedDOFs();
	return settings;
}