// Fill out your copyright notice in the Description page of Project Settings.

#define IS_UE_56_OR_LATER (ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION >= 6)

#include "JoltSkeletalMeshComponent.h"
#include "Engine/World.h"
#include "UnrealJolt/Helpers.h"
#include "Misc/AssertionMacros.h"
#include "PhysicsEngine/PhysicsAsset.h"
#if IS_UE_56_OR_LATER
	#include "PhysicsEngine/SkeletalBodySetup.h"
#endif

void UJoltSkeletalMeshComponent::AddOwnPhysicsAsset()
{
	if (IsSimulatingPhysics()) // Chaos physics, disable it
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: 'Simulate Physics'"));
		SetSimulatePhysics(false);
	}

	if (JoltSubSystem == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: jolt subsystem empty"));
		return;
	}

	/*
	 * We are assigning a an ID regardless of the body being in the physics simulation or not
	 * Might not be a good idea to waste body ids if we need to simulate lots of bodies
	 * This will do for now.
	 * Also, we are assuming that every body is either dynamic or kinematic
	 */
	if (OwnBodyID.IsInvalid())
	{
		JoltSubSystem->DynamicBodyIDX++;
		OwnBodyID = JPH::BodyID(JoltSubSystem->DynamicBodyIDX);
		UE_LOG(JoltSubSystemLogs, Log, TEXT("Using dynamically generated BodyID: %d"), OwnBodyID.GetIndexAndSequenceNumber());
	}

	JoltSubSystem->SkeletalMeshBodyIDLocalTransformMap.Add(&OwnBodyID, VisualOffset);

	if (!bSimulatePhysics)
	{
		BodyFilter = new JPH::IgnoreSingleBodyFilter(OwnBodyID);
		return;
	}

	const JPH::Shape* shape = ExtractJoltShape(JoltSubSystem);
	if (!shape)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Invalid or unsupported shape configured for skeletal mesh: %s, owner: %s"), *GetName(), *GetOwner()->GetName());
		BodyFilter = new JPH::IgnoreSingleBodyFilter(OwnBodyID);
		return;
	}

	bIsKinematicBody ? JoltSubSystem->AddKinematicBodyCollision(OwnBodyID, shape, GetOwner()->GetActorTransform(), Friction, Restitution, Mass)
					 : JoltSubSystem->AddDynamicBodyCollision(OwnBodyID, shape, GetOwner()->GetActorTransform(), Friction, Restitution, Mass);

	if (bIncludeInSnapshot)
	{
		StateFilter = new SaveStateFilter();
		StateFilter->AddToBodyIDAllowList(OwnBodyID);
	}

	JoltSubSystem->JoltBodyActors.Emplace(OwnBodyID, GetOwner());
	BodyFilter = new JPH::IgnoreSingleBodyFilter(OwnBodyID);
	UE_LOG(JoltSubSystemLogs, Log, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
}

const JPH::Shape* UJoltSkeletalMeshComponent::ExtractJoltShape(UJoltSubsystem* jolt) const
{
	UPhysicsAsset* PhysicsAsset = GetPhysicsAsset();
	if (!PhysicsAsset)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("UJoltSkeletalMeshComponent::ExtractJoltShape: no physics asset on %s"), *GetName());
		if (GetSkeletalMeshAsset())
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("  SkeletalMesh IS set but has no physics asset"));
		}
		else
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("  SkeletalMesh is NULL — BP defaults not applied yet?"));
		}
		if (PhysicsAssetOverride)
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("  PhysAssetOverride is set"));
		}
		else
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("  PhysAssetOverride is NULL"));
		}
		return nullptr;
	}

	UE_LOG(JoltSubSystemLogs, Log,
		TEXT("UJoltSkeletalMeshComponent::ExtractJoltShape: PhysicsAsset=%s, BodySetups=%d"),
		*PhysicsAsset->GetName(), PhysicsAsset->SkeletalBodySetups.Num());

	const JPH::Shape* extractedShape = nullptr;

	for (const USkeletalBodySetup* skeletalBodySetup : PhysicsAsset->SkeletalBodySetups)
	{
		TArray<FExtractedShape> ExtractedShapes;
		jolt->ExtractPhysicsGeometry(GetOwner()->GetActorTransform(), skeletalBodySetup, ExtractedShapes);
		for (auto& ExtractedShape : ExtractedShapes)
		{
			if (ExtractedShape.Shape && !extractedShape)
				extractedShape = ExtractedShape.Shape;
		}
	}

	if (extractedShape && !CentreOfMassOffset.IsZero())
	{
		extractedShape = new JPH::OffsetCenterOfMassShape(
			extractedShape, JoltHelpers::ToJoltVec3(CentreOfMassOffset));
	}

	if (!extractedShape)
	{
		UE_LOG(JoltSubSystemLogs, Warning,
			TEXT("UJoltSkeletalMeshComponent::ExtractJoltShape: ExtractPhysicsGeometry produced no shapes from %s"),
			*PhysicsAsset->GetName());
	}

	return extractedShape;
}

void UJoltSkeletalMeshComponent::SaveState(TArray<uint8>& serverPhysicsState)
{
	if (StateFilter == nullptr)
	{
		return;
	}
	JoltSubSystem->SaveState(serverPhysicsState, StateFilter);
}

void UJoltSkeletalMeshComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UJoltSkeletalMeshComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bManualInitialization)
	{
		UE_LOG(JoltSubSystemLogs, Log, TEXT("Using manual initialization. Set BodyID and call LoadJoltSubsytem() manually"));
		return;
	}

	UJoltSubsystem* joltSubSystem = GetWorld()->GetSubsystem<UJoltSubsystem>();
	if (joltSubSystem == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("joltSubSystem null, please manually call LoadJoltSubsytem()"));
		return;
	}
	LoadJoltSubsystem(joltSubSystem);
}

void UJoltSkeletalMeshComponent::OnComponentDestroyed(bool bDestroyingHierarchy)
{
	Super::OnComponentDestroyed(bDestroyingHierarchy);
	delete BodyFilter;
	delete StateFilter;
}

void UJoltSkeletalMeshComponent::LoadJoltSubsystem(UJoltSubsystem* joltSubsystem)
{
	JoltSubSystem = joltSubsystem;
	AddOwnPhysicsAsset();
}

FRaycastResult UJoltSkeletalMeshComponent::RayCastNarrowPhaseIgnoreSelf(const FVector& start, const FVector& end) const
{
	if (!ensureMsgf(JoltSubSystem != nullptr && BodyFilter != nullptr,
			TEXT("%hs: JoltSubSystem or BodyFilter not set on %s"),
			__FUNCTION__, *GetName()))
	{
		return FRaycastResult{};
	}
	return JoltSubSystem->RayCastNarrowPhase(start, end, {}, {}, *BodyFilter);
}

void UJoltSkeletalMeshComponent::JoltSetLinearAndAngularVelocity(const FVector& velocity, const FVector& angularVelocity) const
{
	JoltSubSystem->JoltSetLinearAndAngularVelocity(OwnBodyID, velocity, angularVelocity);
}

void UJoltSkeletalMeshComponent::JoltSetLinearVelocity(const FVector& velocity) const
{
	JoltSubSystem->JoltSetLinearVelocity(OwnBodyID, velocity);
}

void UJoltSkeletalMeshComponent::JoltSetPhysicsLocationAndRotation(const FVector& locationWS, const FQuat& rotationWS) const
{
	JoltSubSystem->JoltSetPhysicsLocationAndRotation(OwnBodyID, locationWS, rotationWS);
}

void UJoltSkeletalMeshComponent::JoltSetPhysicsLocation(const FVector& locationWS) const
{
	JoltSubSystem->JoltSetPhysicsLocation(OwnBodyID, locationWS);
}

void UJoltSkeletalMeshComponent::JoltSetPhysicsRotation(const FQuat& rotationWS) const
{
	JoltSubSystem->JoltSetPhysicsRotation(OwnBodyID, rotationWS);
}

FVector UJoltSkeletalMeshComponent::JoltGetVelocityAt(const FVector& locationWS) const
{
	return JoltSubSystem->JoltGetVelocityAt(OwnBodyID, locationWS);
}

void UJoltSkeletalMeshComponent::JoltAddForceAtLocation(const FVector& force, const FVector& locationWS) const
{
	JoltSubSystem->JoltAddForceAtLocation(OwnBodyID, force, locationWS);
}

void UJoltSkeletalMeshComponent::JoltAddImpulseLocation(const FVector& force, const FVector& locationWS) const
{
	JoltSubSystem->JoltAddImpulseAtLocation(OwnBodyID, force, locationWS);
}

void UJoltSkeletalMeshComponent::JoltAddCentralForce(const FVector& force) const
{
	JoltSubSystem->JoltAddForce(OwnBodyID, force);
}

void UJoltSkeletalMeshComponent::JoltGetPhysicsState(FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const
{
	JoltSubSystem->JoltGetPhysicsState(OwnBodyID, transform, transformCOM, velocity, angularVelocity);
}

void UJoltSkeletalMeshComponent::JoltAddCentralImpulse(const FVector& impulse) const
{
	JoltSubSystem->JoltAddCentralImpulse(OwnBodyID, impulse);
}

void UJoltSkeletalMeshComponent::JoltAddTorque(const FVector& torque) const
{
	JoltSubSystem->JoltAddTorque(OwnBodyID, torque);
}

void UJoltSkeletalMeshComponent::JoltReadPhysicsTransform(FTransform& outTransform) const
{
	JoltSubSystem->JoltGetPhysicsTransform(OwnBodyID, outTransform);
}

void UJoltSkeletalMeshComponent::JoltSetVisualTransform(FTransform& joltTransformWS)
{
	JoltSubSystem->ApplyLocalTxIfAny(&OwnBodyID, joltTransformWS);
	SetWorldTransform(joltTransformWS);
}

int32 UJoltSkeletalMeshComponent::GetJoltBodyID() const
{
	return OwnBodyID.GetIndexAndSequenceNumber();
}
