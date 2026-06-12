#include "JoltDebugRenderer.h"

#include "DrawDebugHelpers.h"
#include "GameFramework/PlayerController.h"

void UEJoltBodyDrawFilter::BeginFrame(const FVector& InCameraPosition, float InMaxDrawDistance, int32 InMaxBodies)
{
	CameraPosition = InCameraPosition;
	MaxDrawDistanceSquared = InMaxDrawDistance > 0.0f ? InMaxDrawDistance * InMaxDrawDistance : 0.0f;
	MaxBodiesToDraw = InMaxBodies;
	bCollectClosestBodies = false;
	ClosestBodies.Reset();
	SelectedBodyIDs.Reset();
	PendingCapsuleDraws.Reset();
}

void UEJoltBodyDrawFilter::BeginCollectClosestBodies()
{
	bCollectClosestBodies = true;
	ClosestBodies.Reset();
	SelectedBodyIDs.Reset();
}

void UEJoltBodyDrawFilter::FinalizeClosestBodies()
{
	bCollectClosestBodies = false;
	SelectedBodyIDs.Reset();
	for (auto& Candidate : ClosestBodies)
	{
		SelectedBodyIDs.Add(Candidate.BodyID);
	}
}

void UEJoltBodyDrawFilter::AddClosestBodyCandidate(uint32 BodyID, float DistanceSquared) const
{
	if (MaxBodiesToDraw <= 0)
	{
		return;
	}

	if (ClosestBodies.Num() < MaxBodiesToDraw)
	{
		ClosestBodies.Add(FBodyCandidate{ BodyID, DistanceSquared });
		return;
	}

	int32 FurthestIndex = 0;
	for (int32 i = 1; i < ClosestBodies.Num(); ++i)
	{
		if (ClosestBodies[i].DistanceSquared > ClosestBodies[FurthestIndex].DistanceSquared)
		{
			FurthestIndex = i;
		}
	}

	if (DistanceSquared < ClosestBodies[FurthestIndex].DistanceSquared)
	{
		ClosestBodies[FurthestIndex] = FBodyCandidate{ BodyID, DistanceSquared };
	}
}

bool UEJoltBodyDrawFilter::ShouldDrawByType(const JPH::Body& inBody) const
{
	if (inBody.GetShape()->GetType() == JPH::EShapeType::HeightField)
	{
		return bDrawHeightField;
	}

	switch (inBody.GetMotionType())
	{
		case JPH::EMotionType::Static:
			return bDrawStatic;
		case JPH::EMotionType::Dynamic:
			return bDrawDynamic;
		case JPH::EMotionType::Kinematic:
			return bDrawKinematic;
		default:
			return true;
	}
}

FColor UEJoltBodyDrawFilter::GetBodyDebugColor(const JPH::Body& inBody) const
{
	if (inBody.IsSensor())
	{
		return JoltHelpers::ToUEColor(JPH::Color::sYellow);
	}

	switch (inBody.GetMotionType())
	{
		case JPH::EMotionType::Static:
			return JoltHelpers::ToUEColor(JPH::Color::sGrey);
		case JPH::EMotionType::Kinematic:
			return JoltHelpers::ToUEColor(JPH::Color::sGreen);
		case JPH::EMotionType::Dynamic:
			return JoltHelpers::ToUEColor(JPH::Color::sGetDistinctColor(inBody.GetID().GetIndex()));
		default:
			return JoltHelpers::ToUEColor(JPH::Color::sWhite);
	}
}

void UEJoltBodyDrawFilter::QueueCapsuleDraw(const JPH::Body& inBody) const
{
	if (inBody.GetShape()->GetSubType() != JPH::EShapeSubType::Capsule)
	{
		return;
	}

	auto* CapsuleShape = static_cast<const JPH::CapsuleShape*>(inBody.GetShape());

	FPendingCapsuleDraw Capsule;
	Capsule.Center = JoltHelpers::ToUEPos(inBody.GetCenterOfMassPosition());
	Capsule.Radius = JoltHelpers::ToUESize(CapsuleShape->GetRadius());
	Capsule.HalfHeight = JoltHelpers::ToUESize(CapsuleShape->GetHalfHeightOfCylinder() + CapsuleShape->GetRadius());
	Capsule.Rotation = JoltHelpers::ToUERot(inBody.GetRotation());
	Capsule.Color = GetBodyDebugColor(inBody);
	PendingCapsuleDraws.Add(Capsule);
}

bool UEJoltBodyDrawFilter::ShouldDraw(const JPH::Body& inBody) const
{
	auto  bodyPosition = JoltHelpers::ToUEPos(inBody.GetCenterOfMassPosition());
	float DistanceSquared = FVector::DistSquared(bodyPosition, CameraPosition);

	if (MaxDrawDistanceSquared > 0.0f)
	{
		if (DistanceSquared > MaxDrawDistanceSquared)
		{
			return false;
		}
	}

	if (!ShouldDrawByType(inBody))
	{
		return false;
	}

	if (MaxBodiesToDraw <= 0)
	{
		if (inBody.GetShape()->GetSubType() == JPH::EShapeSubType::Capsule)
		{
			QueueCapsuleDraw(inBody);
			return false;
		}

		return true;
	}

	auto bodyID = inBody.GetID().GetIndexAndSequenceNumber();
	if (bCollectClosestBodies)
	{
		AddClosestBodyCandidate(bodyID, DistanceSquared);
		return false;
	}

	if (!SelectedBodyIDs.Contains(bodyID))
	{
		return false;
	}

	if (inBody.GetShape()->GetSubType() == JPH::EShapeSubType::Capsule)
	{
		QueueCapsuleDraw(inBody);
		return false;
	}

	return true;
}

bool UEJoltDebugRenderer::EnsureWorld()
{
	if (World)
	{
		bLoggedMissingWorld = false;
		return true;
	}

	if (!bLoggedMissingWorld)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("JoltPhysicsSubSystem::DebugRenderer World is null."));
		bLoggedMissingWorld = true;
	}

	return false;
}

bool UEJoltDebugRenderer::UpdateCameraPosition(FVector& OutCameraPosition)
{
	if (!EnsureWorld())
	{
		return false;
	}

	auto* PlayerController = World->GetFirstPlayerController();
	if (!PlayerController)
	{
		return false;
	}

	FVector  ViewLocation;
	FRotator ViewRotation;
	PlayerController->GetPlayerViewPoint(ViewLocation, ViewRotation);
	SetCameraPos(JoltHelpers::ToJoltPos(ViewLocation));
	OutCameraPosition = ViewLocation;
	return true;
}

UEJoltDebugRenderer::UEJoltDebugRenderer(UWorld* world)
	: World(world) {}

void UEJoltDebugRenderer::DrawBodiesFiltered(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings)
{
	if (!EnsureWorld())
	{
		return;
	}

	FVector CameraPosition;
	if (!UpdateCameraPosition(CameraPosition))
	{
		return;
	}

	DrawFilter.BeginFrame(CameraPosition, JoltSettings->DebugDrawMaxDistance, JoltSettings->DebugDrawMaxBodies);
	if (JoltSettings->DebugDrawMaxBodies > 0)
	{
		DrawFilter.BeginCollectClosestBodies();
		DrawConfiguredBodies(PhysicsSystem, Settings, JoltSettings);
		DrawFilter.FinalizeClosestBodies();
	}

	DrawConfiguredBodies(PhysicsSystem, Settings, JoltSettings);
	DrawPendingCapsules();

	NextFrame();
}

void UEJoltDebugRenderer::DrawConfiguredBodies(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings)
{
	bool bWantStatics = JoltSettings->bDebugDrawStaticBodies || JoltSettings->bDebugDrawHeightFields;

	// Draw all static body shapes (including heightfields if enabled).
	if (bWantStatics)
	{
		DrawFilter.bDrawStatic = JoltSettings->bDebugDrawStaticBodies;
		DrawFilter.bDrawDynamic = false;
		DrawFilter.bDrawKinematic = false;
		DrawFilter.bDrawHeightField = JoltSettings->bDebugDrawHeightFields;

		PhysicsSystem->DrawBodies(Settings, this, &DrawFilter);
	}

	// Draw non-static bodies in a separate pass to keep motion-type toggles independent.
	if (JoltSettings->bDebugDrawDynamicBodies || JoltSettings->bDebugDrawKinematicBodies)
	{
		DrawFilter.bDrawStatic = false;
		DrawFilter.bDrawDynamic = JoltSettings->bDebugDrawDynamicBodies;
		DrawFilter.bDrawKinematic = JoltSettings->bDebugDrawKinematicBodies;
		DrawFilter.bDrawHeightField = false;

		PhysicsSystem->DrawBodies(Settings, this, &DrawFilter);
	}
}

void UEJoltDebugRenderer::DrawPendingCapsules()
{
	for (auto& Capsule : DrawFilter.PendingCapsuleDraws)
	{
		DrawDebugCapsule(World, Capsule.Center, Capsule.HalfHeight, Capsule.Radius, Capsule.Rotation, Capsule.Color, false, -1.0f);
	}
}

void UEJoltDebugRenderer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor)
{
	if (!EnsureWorld())
	{
		return;
	}

	DrawDebugLine(World,
		JoltHelpers::ToUEPos(inFrom),
		JoltHelpers::ToUEPos(inTo),
		JoltHelpers::ToUEColor(inColor),
		false,
		-1.0f);
}

void UEJoltDebugRenderer::DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow)
{
	if (!EnsureWorld())
	{
		return;
	}

	FVector V1 = JoltHelpers::ToUEPos(inV1);
	FVector V2 = JoltHelpers::ToUEPos(inV2);
	FVector V3 = JoltHelpers::ToUEPos(inV3);
	FColor  Color = JoltHelpers::ToUEColor(inColor);

	DrawDebugLine(World, V1, V2, Color, false, -1.0f);
	DrawDebugLine(World, V2, V3, Color, false, -1.0f);
	DrawDebugLine(World, V3, V1, Color, false, -1.0f);
}

void UEJoltDebugRenderer::DrawText3D(JPH::RVec3Arg inPosition, const JPH::string_view& inString, JPH::ColorArg inColor, float inHeight)
{
	if (!EnsureWorld())
	{
		return;
	}

	FVector Position = JoltHelpers::ToUEPos(inPosition);
	FColor  Color = JoltHelpers::ToUEColor(inColor);
	FString TextString(inString.data(), static_cast<int32>(inString.size()));

	DrawDebugString(World, Position, TextString, nullptr, Color, -1.0f, false, inHeight);
}