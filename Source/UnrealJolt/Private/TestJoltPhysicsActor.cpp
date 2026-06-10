#include "TestJoltPhysicsActor.h"

#include "JoltSubsystem.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

ATestJoltPhysicsActor::ATestJoltPhysicsActor()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
}

void ATestJoltPhysicsActor::BeginPlay()
{
	Super::BeginPlay();
	SpawnCapsules();
}

void ATestJoltPhysicsActor::SpawnCapsules()
{
	if (!GetWorld())
		return;
	if (!ensure(CapsuleStaticMesh))
		return;
	if (!ensure(CapsuleCount > 0))
		return;

	UJoltSubsystem* JoltSubsystem = GetWorld()->GetSubsystem<UJoltSubsystem>();
	if (!JoltSubsystem)
		return;

	for (int32 Index = 0; Index < CapsuleCount; ++Index)
	{
		AActor* CapsuleActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FActorSpawnParameters{});
		if (!CapsuleActor)
			continue;

		CapsuleActor->SetRootComponent(NewObject<USceneComponent>(CapsuleActor));

		FTransform Transform;
		Transform.SetLocation(GetSpawnLocation(Index));
		Transform.SetRotation(GetSpawnRotation(Index));
		CapsuleActor->SetActorTransform(Transform);

		UStaticMeshComponent* CapsuleComponent = NewObject<UStaticMeshComponent>(CapsuleActor);
		CapsuleComponent->SetupAttachment(CapsuleActor->GetRootComponent());
		CapsuleActor->AddInstanceComponent(CapsuleComponent);
		CapsuleComponent->SetStaticMesh(CapsuleStaticMesh);
		CapsuleComponent->SetGenerateOverlapEvents(false);
		CapsuleComponent->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		CapsuleComponent->SetMobility(EComponentMobility::Movable);
		CapsuleComponent->SetSimulatePhysics(false);
		CapsuleComponent->SetEnableGravity(false);
		CapsuleComponent->SetRelativeLocation(-CapsuleStaticMesh->GetBounds().Origin);
		CapsuleComponent->RegisterComponent();

		int64 BodyId = JoltSubsystem->AddDynamicBody(CapsuleActor, Friction, Restitution, Mass);
		if (BodyId == 0)
			CapsuleActor->Destroy();
	}
}

FVector ATestJoltPhysicsActor::GetSpawnLocation(int32 Index) const
{
	int32 SafeCapsuleCount = FMath::Max(1, CapsuleCount);

	// Derive a near-square grid from CapsuleCount.
	int32 DerivedGridWidth = FMath::Max(
		1, FMath::CeilToInt(FMath::Sqrt(static_cast<float>(SafeCapsuleCount))));
	int32 GridHeight = FMath::DivideAndRoundUp(SafeCapsuleCount, DerivedGridWidth);

	int32 X = Index % DerivedGridWidth;
	int32 Y = Index / DerivedGridWidth;
	int32 RowCapsuleCount = FMath::Min(DerivedGridWidth, SafeCapsuleCount - Y * DerivedGridWidth);

	FVector LocalOffset(
		(X - (RowCapsuleCount - 1) * 0.5f) * CapsuleSpacing,
		(Y - (GridHeight - 1) * 0.5f) * CapsuleSpacing,
		0.f);

	return GetActorTransform().TransformPosition(LocalOffset);
}

FQuat ATestJoltPhysicsActor::GetSpawnRotation(int32 Index) const
{
	FQuat SpawnRotation = GetActorQuat();
	if (bRandomizeCapsuleRotation)
	{
		FRandomStream RandomStream(RotationSeed + Index);
		SpawnRotation = FRotator(
				RandomStream.FRandRange(-180.f, 180.f),
				RandomStream.FRandRange(-180.f, 180.f),
				RandomStream.FRandRange(-180.f, 180.f))
			.Quaternion();
	}
	return SpawnRotation;
}