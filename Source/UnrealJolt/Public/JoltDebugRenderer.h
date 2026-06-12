#pragma once

#include "UnrealJolt/Helpers.h"
#include "UnrealJolt/JoltMain.h"
#include "JoltSettings.h"

#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>

class UEJoltBodyDrawFilter : public JPH::BodyDrawFilter
{
public:
	struct FBodyCandidate
	{
		uint32 BodyID;
		float DistanceSquared;
	};

	struct FPendingCapsuleDraw
	{
		FVector Center;
		FQuat Rotation;
		float HalfHeight;
		float Radius;
		FColor Color;
	};

	bool bDrawStatic = true;
	bool bDrawDynamic = true;
	bool bDrawKinematic = true;
	bool bDrawHeightField = true;
	FVector CameraPosition = FVector::ZeroVector;
	float MaxDrawDistanceSquared = 0.0f;
	int32 MaxBodiesToDraw = 0;
	bool bCollectClosestBodies = false;
	mutable TArray<FBodyCandidate> ClosestBodies;
	mutable TSet<uint32> SelectedBodyIDs;
	mutable TArray<FPendingCapsuleDraw> PendingCapsuleDraws;

	void BeginFrame(const FVector& InCameraPosition, float InMaxDrawDistance, int32 InMaxBodies);

	void BeginCollectClosestBodies();

	void FinalizeClosestBodies();

	void AddClosestBodyCandidate(uint32 BodyID, float DistanceSquared) const;

	bool ShouldDrawByType(const JPH::Body& inBody) const;

	FColor GetBodyDebugColor(const JPH::Body& inBody) const;

	void QueueCapsuleDraw(const JPH::Body& inBody) const;

	virtual bool ShouldDraw(const JPH::Body& inBody) const override;
};

class UEJoltDebugRenderer final : public JPH::DebugRendererSimple
{
private:
	UWorld* World;
	bool bLoggedMissingWorld = false;

	UEJoltBodyDrawFilter DrawFilter;

	bool EnsureWorld();
	bool UpdateCameraPosition(FVector& OutCameraPosition);
	void DrawConfiguredBodies(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings);
	void DrawPendingCapsules();

public:
	UEJoltDebugRenderer(UWorld* world);

	void DrawBodiesFiltered(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings);

	virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override;
	virtual void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow) override;
	virtual void DrawText3D(JPH::RVec3Arg inPosition, const JPH::string_view& inString, JPH::ColorArg inColor, float inHeight) override;
};
