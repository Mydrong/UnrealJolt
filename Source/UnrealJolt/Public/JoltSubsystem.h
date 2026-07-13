#pragma once

#include "JoltBodyID.h"
#include "JoltCharacter.h"
#include "Containers/Array.h"
#include "Containers/Map.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "JoltContactListener.h"
#include "JoltDataAsset.h"
#include "HAL/LowLevelMemTracker.h"
#include "JoltWorker.h"
#include "JoltSettings.h"
#include "Landscape.h"
#include "Subsystems/WorldSubsystem.h"
#include "JoltLayerTable.h"
#include "JoltFilters.h"
#include "Delegates/DelegateCombinations.h"
#include "UObject/ObjectMacros.h"

LLM_DECLARE_TAG(Jolt_Physics);

#ifdef JPH_DEBUG_RENDERER
#include "JoltDebugRenderer.h"
#endif

#include "JoltSubsystem.generated.h"

/* Just used to make them friend classess of UJoltSubSystem class because it needs some private methods
 * that should not be exposed to avoid accidental usage in project code  */
class UJoltSkeletalMeshComponent;
class JoltAxisConstraint;
class JoltPhysicsMaterial;
class UBodySetup;
struct FKAggregateGeom;
struct FKConvexElem;

USTRUCT(BlueprintType)
struct FRaycastResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector HitLocation = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector HitNormal = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	bool bHasHit = false;

	FJoltBodyID HitBodyID;
};

struct FFrameHistory
{
	FVector  PrevLocation;
	FRotator PrevRotation;

	FVector  CurrentLocation;
	FRotator CurrentRotation;
};

USTRUCT(BlueprintType)
struct FCastShapeResult
{
	GENERATED_USTRUCT_BODY()

	FJoltBodyID ContactBodyID;
	
	FVector LocationShape;

	FVector ContactPointOn2;

	FVector ContactPointOn1;

	/** Fraction of the sweep where the hit occurred (0 = at start, 1 = at end). Negative or 0 indicates initial penetration. */
	float Fraction = 1.0f;
};

struct FJoltBodyActor
{
	FJoltBodyActor(const JPH::BodyID JoltBodyID, const TWeakObjectPtr<AActor>& Actor)
		: JoltBodyID(JoltBodyID)
		, Actor(Actor) {}

	const JPH::BodyID      JoltBodyID;
	TWeakObjectPtr<AActor> Actor;
	FFrameHistory          FrameHistory;
};

struct FExtractedShape
{
	JPH::RefConst<JPH::Shape> Shape;
	FTransform                WorldTransform;
};

typedef const std::function<void(const FVector&, const FVector&, const bool&, const uint32&, const UPhysicalMaterial*)> NarrowPhaseQueryCallback;

DECLARE_MULTICAST_DELEGATE(FOnJoltReady);

UCLASS()
class UNREALJOLT_API UJoltSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", meta = (AdvancedDisplay = "Layer"))
	FJoltBodyID AddDynamicBody(AActor* Actor, const float& friction, const float& restitution, const float& mass, FName Layer = NAME_None);

	/// Creates a new dynamic body from a UE FKAggregateGeom and returns its newly allocated BodyID.
	///
	/// Will update actor position/rotation if provided.
	FJoltBodyID AddDynamicShapes(
		AActor*                actor,
		const FKAggregateGeom& aggregateGeom,
		const FTransform&      initialWorldTransform,
		float                  friction, float restitution, float mass,
		FName                  layerName = NAME_None);

	/*
	 * Adds a static body to the jolt physics system.
	 * The other way is to just add 'jolt-static' tag to the actor
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", meta = (AdvancedDisplay = "Layer"))
	FJoltBodyID AddStaticBody(const AActor* body, const float& friction, const float& restitution, FName Layer = NAME_None);

	FJoltBodyID AddStaticShapes(const FKAggregateGeom& AggregateGeom, const FTransform& worldTransform, const float& friction, const float& restitution, FName Layer = NAME_None);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", meta = (AdvancedDisplay = "Layer"))
	FJoltBodyID AddMeshShape(
		const TArray<FVector>& vertices, const TArray<int32>& indices, const FTransform& worldTransform,
		bool                   bDynamic, float                friction, float            restitution, float Mass, FName Layer = NAME_None);

	TArray<JPH::Ref<FJoltCharacter>> JoltCharacters;
	FJoltCharacter*                  CreateCharacter(
		const FVector& Location, const FRotator&  Rotation, float      HalfHeight, float Radius,
		float          MaxPenetrationDepth, float MaxSlopeAngle, FName Layer);


	/*
	 * Layer lookup. Returns INDEX_NONE if the name isn't a configured object layer.
	 * Use when gameplay code needs the raw id for a custom body creation path.
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt|Layers")
	int32 GetObjectLayerByName(FName LayerName) const;

	// Returns the JPH::ObjectLayer for the given name, or Fallback when the name is unknown.
	// Default fallback is JPH::cObjectLayerInvalid so callers can detect & refuse to create a body
	// rather than silently dumping it into whichever layer happens to be at index 0.
	JPH::ObjectLayer ResolveObjectLayer(FName LayerName, JPH::ObjectLayer Fallback = JPH::cObjectLayerInvalid) const;

	JPH::ObjectLayer ResolveDynamicLayer(FName LayerName) const
	{
		return ResolveObjectLayer(LayerName.IsNone() ? JoltSettings->DefaultDynamicLayer : LayerName);
	}

	JPH::ObjectLayer ResolveStaticLayer(FName LayerName) const
	{
		return ResolveObjectLayer(LayerName.IsNone() ? JoltSettings->DefaultStaticLayer : LayerName);
	}

	FName GetLayerName(JPH::ObjectLayer Layer) const;

	// Read-only access to the runtime layer table (valid after Initialize / before Deinitialize).
	const FJoltLayerTable& GetLayerTable() const { return LayerTable; }

	/*
	 * Sweeps a ray from start to end.
	 * This will first perform a broadphase, then a narrow phase query
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	FRaycastResult RayCastNarrowPhase(const FVector& start, const FVector& end);

	/*
	 * Sweeps a ray and returns all body hits sorted from nearest to furthest.
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FRaycastResult> RayCastBroadPhase(const FVector& start, const FVector& end);

	/*
	 * Used to check a collision by placing the shape at a static location
	 * 
	 * Returns bodies whose layer is in the given sets of layers.
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<int32> CollideShape(
		const UShapeComponent* shape, const FTransform&             shapeCOM,
		const TSet<FName>&     broadPhaseLayers, const TSet<FName>& objectLayers);

	/// Used to check a collision by placing the shape at a static location
	/// 
	/// Returns bodies that collide with the given layers.
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<int32> CollideShape_ByLayers(
		const UShapeComponent* shape, const FTransform&               shapeCOM,
		const TArray<FName>&   broadPhaseLayers, const TArray<FName>& objectLayers);

	/*
	 * Sweep a shape to detect collision
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> CastShape(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& direction);

	// UE-style primitive sweeps that don't require a shape component.
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	bool SphereTraceSingle(
		const FVector&             start, const FVector& end, float radius, FCastShapeResult& outHit,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> SphereTraceMulti(
		const FVector&             start, const FVector& end, float radius,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> SphereTraceMulti_ByLayers(
		const FVector&             start, const FVector&                  end, float radius,
		const TArray<FName>&       broadPhaseLayers, const TArray<FName>& objectLayers,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	bool BoxTraceSingle(
		const FVector&             start, const FVector& end, const FVector& halfExtent, const FRotator& orientation, FCastShapeResult& outHit,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> BoxTraceMulti(
		const FVector&             start, const FVector& end, const FVector& halfExtent, const FRotator& orientation,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> BoxTraceMulti_ByLayers(
		const FVector&             start, const FVector&                  end, const FVector& halfExtent, const FRotator& orientation,
		const TArray<FName>&       broadPhaseLayers, const TArray<FName>& objectLayers,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	bool CapsuleTraceSingle(
		const FVector&             start, const FVector& end, float radius, float halfHeight, const FRotator& orientation, FCastShapeResult& outHit,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> CapsuleTraceMulti(
		const FVector&             start, const FVector& end, float radius, float halfHeight, const FRotator& orientation,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> CapsuleTraceMulti_ByLayers(
		const FVector&             start, const FVector&                  end, float radius, float halfHeight, const FRotator& orientation,
		const TArray<FName>&       broadPhaseLayers, const TArray<FName>& objectLayers,
		const TArray<FJoltBodyID>& ignoredBodyIDs);

	bool CastShapeSingle(
		const JPH::Shape*      inShape, const FVector&    inShapeScale,
		const FTransform&      inShapeCOM, const FVector& inDirection,
		FCastShapeResult&      outHit,
		const JPH::BodyFilter& inBodyFilter) const;

	/*
	 * Fetch the centre of mass of the body
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	FTransform GetBodyCOM(int32 bodyID);

	/*
	 * This fetches the current call interval beween each calls to update()
	 * This will be equal to the JoltSettings->FixedDeltaTime in most cases.
	 * This will only change if SetTimeScale() is called manually
	 * Useful for client-server configs where scaling time is desired
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	double GetTimeScale() const { return ConfiguredDeltaSeconds; };

	/*
	 * This is the value that is passed to the physics update() function
	 * Changing the TimeScale will not change this value
	 * The physics system is esseintailly goint to "pretent" like the deltatime has scaled
	 * Call interval can change, the physicsDeltaTime will not (at runtime only)
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	double GetDeltaTime() const { return JoltSettings->FixedDeltaTime; };

	/*
	 * Jolt physics tickrate. This is divided in inCollisionSteps iterations
	 * Number of physics ticks per second
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	int GetTickRate() { return JoltSettings->TickRate; };

	#if WITH_EDITOR
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void ExtractSplineMeshGeometry(const UBodySetup* splineMeshBodySetup, const FTransform& splineMeshTransform);
	#endif

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetLinearAndAngularVelocity(const FJoltBodyID& bodyID, const FVector& velocity, const FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltGetPhysicsState(const FJoltBodyID& bodyID, FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddCentralImpulse(const FJoltBodyID& bodyID, const FVector& impulse) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddTorque(const FJoltBodyID& bodyID, const FVector& torque) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddForce(const FJoltBodyID& bodyID, const FVector& force) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddImpulseAtLocation(const FJoltBodyID& bodyID, const FVector& impulse, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddForceAtLocation(const FJoltBodyID& bodyID, const FVector& force, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	FVector JoltGetVelocityAt(const FJoltBodyID& bodyID, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsLocationAndRotation(const FJoltBodyID& bodyID, const FVector& locationWS, const FQuat& rotationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsLocationRotationAndVelocity(const FJoltBodyID& bodyID, const FVector& locationWS, const FQuat& rotationWS, const FVector& linearVelocity, const FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetLinearVelocity(const FJoltBodyID& bodyID, const FVector& velocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsLocation(const FJoltBodyID& bodyID, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsRotation(const FJoltBodyID& bodyID, const FQuat& rotationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltMoveKinematic(const FJoltBodyID& bodyID, const FVector& locationWS, const FQuat& rotationWS, float DeltaTime) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltGetPhysicsTransform(const FJoltBodyID& bodyID, FTransform& transform) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void SetTimeScale(double deltaTime);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	bool GetContactInfo(FContactInfo& contactInfo) const { return ContactListener->Consume(contactInfo); };

	UEJoltCallBackContactListener* GetContactListener() { return ContactListener; };

	void JoltSetLinearAndAngularVelocity(const JPH::BodyID& bodyID, const FVector& velocity, const FVector& angularVelocity) const;

	void JoltGetPhysicsState(const JPH::BodyID& bodyID, FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const;

	void JoltAddCentralImpulse(const JPH::BodyID& bodyID, const FVector& impulse) const;

	void JoltAddTorque(const JPH::BodyID& bodyID, const FVector& torque) const;

	void JoltAddForce(const JPH::BodyID& bodyID, const FVector& torque) const;

	void JoltAddImpulseAtLocation(const JPH::BodyID& bodyID, const FVector& impulse, const FVector& locationWS) const;

	void JoltAddForceAtLocation(const JPH::BodyID& bodyID, const FVector& force, const FVector& locationWS) const;

	FVector JoltGetVelocityAt(const JPH::BodyID& bodyID, const FVector& locationWS) const;

	void JoltSetPhysicsLocationAndRotation(const JPH::BodyID& bodyID, const FVector& locationWS, const FQuat& rotationWS) const;

	void JoltSetLinearVelocity(const JPH::BodyID& bodyID, const FVector& velocity) const;

	void JoltSetPhysicsLocation(const JPH::BodyID& bodyID, const FVector& locationWS) const;

	void JoltSetPhysicsRotation(const JPH::BodyID& bodyID, const FQuat& rotationWS) const;

	void JoltGetPhysicsTransform(const JPH::BodyID& bodyID, FTransform& transform) const;

	// This will first perform a broadphase, and then a narrow phase query.
	FRaycastResult RayCastNarrowPhase(
		const FVector&          start, const FVector& end,
		const TSet<FName>&      broadPhaseLayers = {},
		const TSet<FName>&      objectLayers = {},
		const JPH::BodyFilter&  inBodyFilter = {},
		const JPH::ShapeFilter& inShapeFilter = {}) const;

	/// Same as RayCastNarrowPhase but returns all hits along the ray, sorted by distance.
	/// 
	/// Returns bodies with layers in the given sets of layers.
	TArray<FRaycastResult> RayCastBroadPhase_ForObjects(
		const FVector&          start, const FVector& end,
		const TSet<FName>&      broadPhaseLayers = {},
		const TSet<FName>&      objectLayers = {},
		const JPH::BodyFilter&  inBodyFilter = {},
		const JPH::ShapeFilter& inShapeFilter = {}) const;

	/// Same as RayCastNarrowPhase but returns all hits along the ray, sorted by distance.
	/// 
	/// Returns bodies that collide with the given layers.
	TArray<FRaycastResult> RayCastBroadPhase_ByLayers(
		const FVector&          start, const FVector& end,
		const TArray<FName>&    broadPhaseLayers = {},
		const TArray<FName>&    objectLayers = {},
		const JPH::BodyFilter&  inBodyFilter = {},
		const JPH::ShapeFilter& inShapeFilter = {}) const;

	void RayCastShapeNarrowPhase(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset, NarrowPhaseQueryCallback& hitCallback);

	uint16 AddPrePhysicsCallback(const TDelegate<void(float)>& callback) const;

	uint16 AddPostPhysicsCallback(const TDelegate<void(float)>& callback) const;

	/** Fired once per frame after InterpolatePhysicsFrame completes. dt = frame delta. */
	void AddPostInterpolationCallback(const TDelegate<void(float)>& callback);

	/** Physics-frame interpolation alpha (0..1) computed in the most recent Tick. */
	double GetPhysicsAlpha() const { return PhysicsAlpha_; }

	void RestoreState(const TArray<uint8>& serverPhysicsState) const;

	// Saves the current physics states of bodies (optionally filtered)
	void SaveState(TArray<uint8>& serverPhysicsState, JPH::StateRecorderFilter* saveFilterImpl = nullptr) const;

	/* This will automatically be called in the tick function
	 * for use cases where you just need to step the physics only
	 * set bWithCallbacks to false and disable all the binded callbacks
	 */
	void StepPhysics(bool bWithCallbacks = true);

	// Ad hoc debug draw wrappers routed through the Jolt debug renderer when available.
	void JoltDrawDebugSphere(const FVector& Center, float Radius, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugBox(const FVector& Center, const FVector& Extent, const FQuat& Rotation, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugTriangle(const FVector& V1, const FVector& V2, const FVector& V3, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugCapsule(const FVector& Center, float HalfHeight, float Radius, const FQuat& Rotation, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugCylinder(const FVector& Center, float HalfHeight, float Radius, const FQuat& Rotation, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugConvexHull(const TArray<FVector>& Points, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugMesh(const TArray<FVector>& Vertices, const TArray<uint32>& Indices, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;
	void JoltDrawDebugPlane(const FVector& Center, const FVector& Normal, float Size, const FColor& Color, bool bPersistent = false, float LifeTime = -1.0f) const;

	// --- Public external-owner API ---
	// Stable contract for plugins that drive their own physics state on top
	// of UJoltSubsystem. Not BlueprintCallable.
	JPH::BodyInterface* GetBodyInterface() const { return BodyInterface; }
	JPH::PhysicsSystem* GetPhysicsSystem() const { return MainPhysicsSystem; }
	const JPH::BodyID*  AddDynamicBodyForExternalOwner(
		const JPH::BodyID& bodyID,
		const JPH::Shape*  shape,
		const FTransform&  initialWorldTransform,
		float              friction, float restitution, float mass,
		FName              layerName = NAME_None);

	// Mirror of AddDynamicBodyForExternalOwner: removes and destroys the body
	// in Jolt AND drops the BodyIDBodyMap entry. 
	void RemoveBodyForExternalOwner(const FJoltBodyID& bodyID);

	void AddActorBodyMapping(const JPH::BodyID& bodyID, const TWeakObjectPtr<AActor>& actor);

protected:
	const JoltPhysicsMaterial* GetJoltPhysicsMaterial(const UPhysicalMaterial* UEPhysicsMat);

	const UPhysicalMaterial* GetUEPhysicsMaterial(const JoltPhysicsMaterial* JoltPhysicsMat) const;

	const JPH::BoxShape* GetBoxCollisionShape(const FVector& dimensions, const JoltPhysicsMaterial* material = nullptr);

	const JPH::SphereShape* GetSphereCollisionShape(const float& radius, const JoltPhysicsMaterial* material = nullptr);

	const JPH::CapsuleShape* GetCapsuleCollisionShape(const float& radius, const float& height, const JoltPhysicsMaterial* material = nullptr);

	const JPH::ConvexHullShape* GetConvexHullCollisionShape(const FKConvexElem& convexElem, const FVector& scale, const JoltPhysicsMaterial* material = nullptr);

	const JPH::BodyID* AddDynamicBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddDynamicBodyCollision(const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddStaticBodyCollision(const JPH::Shape* shape, const FTransform& transform, float friction, float restitution, FName layerName = NAME_None);

	const JPH::BodyID* AddStaticBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, FName layerName = NAME_None);

	const JPH::BodyID* AddKinematicBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddKinematicBodyCollision(const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddBodyToSimulation(const JPH::BodyID* bodyID, const JPH::BodyCreationSettings& shapeSettings, float friction, float restitution);

	JPH::Body* GetBody(const FJoltBodyID& bodyID) const { return BodyIDBodyMap[bodyID]; }

	UPROPERTY()
	UJoltDataAsset* JoltDataAsset = nullptr;

	UPROPERTY()
	const UJoltSettings* JoltSettings = nullptr;

	FJoltWorkerOptions* WorkerOptions = nullptr;

	FJoltWorker* JoltWorker = nullptr;

	UEJoltCallBackContactListener* ContactListener = nullptr;

	JPH::PhysicsSystem* MainPhysicsSystem = nullptr;

	JPH::BodyInterface* BodyInterface = nullptr;

	uint32 StaticBodyIDX;

	uint32 DynamicBodyIDX;

	// Runtime layer lookup table — built from UJoltSettings in InitPhysicsSystem and referenced by
	// all three filter implementations for the lifetime of MainPhysicsSystem.
	FJoltLayerTable LayerTable;

	BPLayerInterfaceImpl* BroadPhaseLayerInterface = nullptr;

	// Create class that filters object vs broadphase layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectVsBroadPhaseLayerFilterImpl* ObjectVsBroadphaseLayerFilter = nullptr;

	// Create class that filters object vs object layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectLayerPairFilterImpl* ObjectVsObjectLayerFilter = nullptr;

	TArray<const JPH::BoxShape*> BoxShapes;

	TArray<const JPH::SphereShape*> SphereShapes;

	TArray<const JPH::CapsuleShape*> CapsuleShapes;

	TArray<const JPH::HeightFieldShapeSettings*> HeightFieldShapes;

	TArray<JPH::Body*> SavedBodies;

	TMap<FJoltBodyID, JPH::Body*> BodyIDBodyMap;

	// JPH::Array<const JPH::Body*> HeightMapArray;

	// JPH::Array<const JPH::Body*> LandscapeSplines;

	TMap<EPhysicalSurface, const JoltPhysicsMaterial*> SurfaceJoltMaterialMap;

	TMap<EPhysicalSurface, TWeakObjectPtr<const UPhysicalMaterial>> SurfaceUEMaterialMap;

	TMap<const JPH::BodyID*, FTransform> SkeletalMeshBodyIDLocalTransformMap;

	TArray<const JPH::ConvexHullShape*> ConvexShapes;

	#ifdef JPH_DEBUG_RENDERER
	UEJoltDebugRenderer* JoltDebugRendererImpl = nullptr;

	JPH::BodyManager::DrawSettings* DrawSettings = nullptr;

	UEJoltDebugRenderer* GetDebugRendererForDraw() const;

	void DrawDebugLines() const;
	#endif

	void LoadLandscapeFromDataAsset();

	static ALandscape* FindSingleLandscape(const UWorld* world);

	#if WITH_EDITOR
	void GetAllLandscapeHeights(const ALandscape* landscapeActor);

	bool CookBodies() const;

	void HandleLandscapeMeshes(const ALandscape* LandscapeActor);
	#endif

	static TArray<ULandscapeComponent*> GetLandscapeComponents(const ALandscape* landscapeActor);

	TArray<FExtractedShape> ExtractPhysicsGeometryFromActor(const AActor* actor);

	void ExtractPhysicsGeometryFromComponent(const UPrimitiveComponent* Component, const FTransform& componentTransform, TArray<FExtractedShape>& OutShapes);

	void ExtractPhysicsGeometry(const FTransform& xformSoFar, const UBodySetup* bodySetup, TArray<FExtractedShape>& OutShapes);

	void ExtractPhysicsGeometry(const FTransform& xformSoFar, const FKAggregateGeom& aggregateGeom, const JoltPhysicsMaterial* physicsMaterial, TArray<FExtractedShape>& OutShapes);

	void ExtractAggGeomBoxes(const FTransform& xformSoFar, const FKAggregateGeom& aggregateGeom, const FVector& scale, const JoltPhysicsMaterial* physicsMaterial, JPH::CompoundShapeSettings* compoundShapeSettings, TArray<FExtractedShape>& OutShapes);

	void ExtractAggGeomSpheres(const FTransform& xformSoFar, const FKAggregateGeom& aggregateGeom, const FVector& scale, const JoltPhysicsMaterial* physicsMaterial, JPH::CompoundShapeSettings* compoundShapeSettings, TArray<FExtractedShape>& OutShapes);

	void ExtractAggGeomCapsules(const FTransform& xformSoFar, const FKAggregateGeom& aggregateGeom, const FVector& scale, const JoltPhysicsMaterial* physicsMaterial, JPH::CompoundShapeSettings* compoundShapeSettings, TArray<FExtractedShape>& OutShapes);

	void ExtractAggGeomConvex(const FTransform& xformSoFar, const FKAggregateGeom& aggregateGeom, const FVector& scale, const JoltPhysicsMaterial* physicsMaterial, JPH::CompoundShapeSettings* compoundShapeSettings, TArray<FExtractedShape>& OutShapes);

	void ExtractComplexPhysicsGeometry(const FTransform& xformSoFar, const UBodySetup* bodySetup, const FString& meshName, TArray<FExtractedShape>& OutShapes);

	void ExtractMeshShape(
		const TArray<FVector>& ueVertices, const TArray<int32>&     ueIndices,
		const FTransform&      xformSoFar, TArray<FExtractedShape>& OutShapes);

	const JPH::Shape* ProcessShapeElement(const UShapeComponent* shapeComponent);

	TArray<FCastShapeResult> CastShapeMultiInternal(const JPH::Shape* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& direction) const;

	TArray<FCastShapeResult> CastShapeMultiInternal_ByLayers(
		const JPH::Shape*      shape, const FVector&                  shapeScale, const FTransform& shapeCOM, const FVector& direction,
		const TArray<FName>&   broadPhaseLayers, const TArray<FName>& objectLayers,
		const JPH::BodyFilter& inBodyFilter = {}) const;

	bool CastShapeSingleInternal(
		const JPH::Shape*      shape, const FVector&        shapeScale, const FTransform& shapeCOM,
		const FVector&         direction, FCastShapeResult& outHit,
		const JPH::BodyFilter& inBodyFilter = {}) const;

	/*
	 * Fetch all the actors in UE world and add them to jolt simulation
	 * "jolt-static" tag should be added for static objects (from UE editor)
	 * "jolt-dynamic" tag should be added for dynamic objects (from UE editor)
	 */
	void AddAllJoltActors(const UWorld* World);

	void InterpolatePhysicsFrame(const double& alpha);

	void ApplyLocalTxIfAny(const JPH::BodyID* bodyID, FTransform& interpolatedTransform) const;

	virtual void Tick(float deltaSeconds) override;

	virtual TStatId GetStatId() const override;

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	virtual void Deinitialize() override;

	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	virtual void OnWorldComponentsUpdated(UWorld& InWorld) override;

	void InitPhysicsSystem(
		int cMaxBodies,
		int cNumBodyMutexes,
		int cMaxBodyPairs,
		int cMaxContactConstraints);

	double CurrentTime = FPlatformTime::Seconds();

	double Accumulator = 0.0f;

	double ConfiguredDeltaSeconds = 1.0f / 60.0f;

	double PhysicsAlpha_ = 0.0;

	void RecordFrames();

	TArray<FJoltBodyActor> JoltBodyActors;

	TArray<TDelegate<void(float)>> PostInterpolationCallbacks;

	bool bIsReady = false;
	bool bHasDrawnDebugLinesThisFrame = false;

public:
	FOnJoltReady OnReady;

	double GetPhysicsAlpha() { return PhysicsAlpha_; }

	/** True once OnWorldBeginPlay has finished initializing the worker — for listeners that bind after BeginPlay. */
	bool IsReady() const { return bIsReady; }

	JPH::Vec3 GetLinearVelocity(const JPH::BodyID& bodyID) const
	{
		return BodyInterface->GetLinearVelocity(bodyID);
	}

	JPH::Vec3 GetAngularVelocity(const JPH::BodyID& bodyID) const
	{
		return BodyInterface->GetAngularVelocity(bodyID);
	}

	uint32 GetNumBodies() const
	{
		return MainPhysicsSystem->GetNumBodies();
	}

	friend class UJoltSkeletalMeshComponent;
	friend class JoltAxisConstraint;

	void DrawDebugShapeComponent(
		const UShapeComponent* shapeComponent, const FTransform& Transform, FColor        Color,
		float                  LifeTime = -1, uint8              DepthPriority = 0, float Thickness = 1) const;
};