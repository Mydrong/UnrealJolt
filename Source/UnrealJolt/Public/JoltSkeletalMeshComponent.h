// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/SkeletalMeshComponent.h"
#include "Containers/Array.h"
#include "CoreMinimal.h"
#include "JoltSubsystem.h"

#include "JoltSkeletalMeshComponent.generated.h"

/**
 *
 */
UCLASS(Blueprintable, meta = (BlueprintSpawnableComponent))
class UNREALJOLT_API UJoltSkeletalMeshComponent : public USkeletalMeshComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	FVector CentreOfMassOffset = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	float Mass = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	float Friction = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	float Restitution = 0.5f;

	/*
	 * Change this value if you need to change the visual offset(alignment) of the
	 * mesh with the physics body
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	FTransform VisualOffset = FTransform::Identity;

	/*
	 * Set this value to true if this body needs to be included in the simulation
	 * snapshot
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	bool bIncludeInSnapshot = false;

	// JoltPhysics Toggle to disable simulating physics for ths skeletal mesh
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	bool bIsKinematicBody = false;

	// JoltPhysics Toggle to disable simulating physics for ths skeletal mesh
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	bool bSimulatePhysics = true;

	/*
	 * This is for use cases where you need control over the BodyID, and when the
	 * JoltSubsystem needs to be loaded
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jolt Physics")
	bool bManualInitialization = false;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void LoadJoltSubsystem(UJoltSubsystem* joltSubsystem);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	FVector JoltGetVelocityAt(const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltAddForceAtLocation(const FVector& force, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltAddImpulseLocation(const FVector& force, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltAddCentralForce(const FVector& force) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltGetPhysicsState(FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltAddCentralImpulse(const FVector& impulse) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltAddTorque(const FVector& torque) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	int32 GetJoltBodyID() const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltSetPhysicsLocationAndRotation(const FVector& locationWS, const FQuat& rotationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltSetLinearAndAngularVelocity(const FVector& velocity, const FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltSetLinearVelocity(const FVector& velocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltSetPhysicsLocation(const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltSetPhysicsRotation(const FQuat& rotationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltReadPhysicsTransform(FTransform& outTransform) const;

	/**
	 * Extracts the first JPH::Shape from this component's physics asset.
	 * Applies CentreOfMassOffset if set. Returns nullptr if no physics asset or no shapes found.
	 * Does NOT create a body — use this when you need the shape for external body creation.
	 */
	const JPH::Shape* ExtractJoltShape(UJoltSubsystem* jolt) const;

	FRaycastResult RayCastNarrowPhaseIgnoreSelf(const FVector& start, const FVector& end) const;

	/* Will simply set the location of the visual mesh only
	 * Has no interaction with the physics system
	 * Used to apply visual transform set in editor to the jolt transform
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void JoltSetVisualTransform(FTransform& joltTransformWS);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Skeletal Mesh", BlueprintPure = false)
	void SetBodyID(int32 bodyID) { OwnBodyID = JPH::BodyID(bodyID); };

	// For externally-managed bodies where bManualInitialization=true skips
	// LoadJoltSubsystem, so the JoltSubSystem pointer never gets set via the
	// normal AddOwnPhysicsAsset path. Call this alongside SetBodyID so
	// mesh-level Jolt* methods route through the real subsystem instead of
	// null-dereferencing.
	void SetJoltSubsystem(UJoltSubsystem* jolt) { JoltSubSystem = jolt; }

	virtual void
	TickComponent(float DeltaTime, enum ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	virtual void BeginPlay() override;

	virtual void OnComponentDestroyed(bool bDestroyingHierarchy);

	SaveStateFilter* StateFilter = nullptr;

	JPH::BodyFilter* BodyFilter = nullptr;

	void SaveState(TArray<uint8>& serverPhysicsState);

private:
	UPROPERTY()
	TObjectPtr<UJoltSubsystem> JoltSubSystem = nullptr;

	JPH::BodyID OwnBodyID;

	void AddOwnPhysicsAsset();
};
