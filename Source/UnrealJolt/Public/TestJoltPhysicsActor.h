#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TestJoltPhysicsActor.generated.h"

class UStaticMesh;

UCLASS()
class ATestJoltPhysicsActor : public AActor
{
	GENERATED_BODY()

public:
	ATestJoltPhysicsActor();

protected:
	virtual void BeginPlay() override;

private:
	void SpawnCapsules();

	FVector GetSpawnLocation(int32 Index) const;

	FQuat GetSpawnRotation(int32 Index) const;

private:
	UPROPERTY(EditAnywhere, Category="Spawn")
	int32 CapsuleCount = 25;

	UPROPERTY(EditAnywhere, Category="Spawn")
	float CapsuleSpacing = 175.f;

	UPROPERTY(EditAnywhere, Category="Spawn")
	float SpawnHeightOffset = 200.f;

	UPROPERTY(EditAnywhere, Category="Spawn")
	bool bRandomizeCapsuleRotation = true;

	UPROPERTY(EditAnywhere, Category="Spawn")
	int32 RotationSeed = 1337;

	UPROPERTY(EditAnywhere, Category="Spawn")
	TObjectPtr<UStaticMesh> CapsuleStaticMesh;

	UPROPERTY(EditAnywhere, Category="Jolt")
	float Mass = 60.f;

	UPROPERTY(EditAnywhere, Category="Jolt")
	float Restitution = 0.1f;

	UPROPERTY(EditAnywhere, Category="Jolt")
	float Friction = 0.7f;
};
