#include "JoltContactListener.h"
#include "JoltPhysicsMaterial.h"
#include "UnrealJolt/Helpers.h"

JPH::ValidateResult UEJoltCallBackContactListener::OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult)
{
	return ContactListener::OnContactValidate(inBody1, inBody2, inBaseOffset, inCollisionResult);
}

void UEJoltCallBackContactListener::OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
{

	JPH::CollisionEstimationResult result;
	EstimateCollisionResponse(inBody1, inBody2, inManifold, result, ioSettings.mCombinedFriction, ioSettings.mCombinedRestitution);

	// Resolve EPhysicalSurface for each body at the contact sub-shape. The shape
	// stores a JoltPhysicsMaterial (subclass of JPH::PhysicsMaterial) whose
	// SurfaceType field mirrors the source UPhysicalMaterial — populated when
	// UJoltSubsystem built the shape from the body setup.
	const JPH::Shape*			  shape1 = inBody1.GetShape();
	const JPH::Shape*			  shape2 = inBody2.GetShape();
	const JPH::PhysicsMaterial*	  mat1 = shape1 ? shape1->GetMaterial(inManifold.mSubShapeID1) : nullptr;
	const JPH::PhysicsMaterial*	  mat2 = shape2 ? shape2->GetMaterial(inManifold.mSubShapeID2) : nullptr;
	const EPhysicalSurface surface1Raw = mat1 ? static_cast<const JoltPhysicsMaterial*>(mat1)->SurfaceType : EPhysicalSurface::SurfaceType_Default;
	const EPhysicalSurface surface2Raw = mat2 ? static_cast<const JoltPhysicsMaterial*>(mat2)->SurfaceType : EPhysicalSurface::SurfaceType_Default;
	const TEnumAsByte<EPhysicalSurface> surface1(surface1Raw);
	const TEnumAsByte<EPhysicalSurface> surface2(surface2Raw);

	// Snapshot linear velocities at contact time. Static bodies are zero, so
	// vehicle-vs-world vRel collapses to the vehicle's own velocity.
	const FVector linVel1 = JoltHelpers::ToUESize(inBody1.GetLinearVelocity());
	const FVector linVel2 = JoltHelpers::ToUESize(inBody2.GetLinearVelocity());

	for (uint8 i = 0; const float& impulse : result.mContactImpulse)
	{
		Queue.Enqueue(
			FContactInfo(
				inBody1.GetID().GetIndexAndSequenceNumber(),
				inBody2.GetID().GetIndexAndSequenceNumber(),
				JoltHelpers::ToUEPos(inManifold.GetWorldSpaceContactPointOn1(i)),
				JoltHelpers::ToUEPos(inManifold.GetWorldSpaceContactPointOn2(i)),
				JoltHelpers::ToUESize(impulse),
				JoltHelpers::ToUESize(inManifold.mWorldSpaceNormal, false),
				surface1,
				surface2,
				linVel1,
				linVel2)

		);

		i++;
	}
}

void UEJoltCallBackContactListener::OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
{
	// return ContactListener::OnContactPersisted(inBody1, inBody2, inManifold, ioSettings);
}

void UEJoltCallBackContactListener::OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) {
};
