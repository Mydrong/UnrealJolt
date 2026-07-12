#pragma once

#include "CoreMinimal.h"

class UJoltSettings;

/**
 * Runtime lookup table built from UJoltSettings layer config. Consumed by the filter
 * implementations in JoltFilters.h and by UJoltSubsystem for FName -> JPH::ObjectLayer lookup.
 *
 * Indices in ObjectLayerNames and BroadphaseLayerNames become the Jolt layer ids the
 * filter classes and BodyCreationSettings consume, so ordering must be preserved for the
 * lifetime of a PhysicsSystem instance.
 */
struct UNREALJOLT_API FJoltLayerTable
{
	TArray<FName> ObjectLayerNames;     // index -> ObjectLayer id
	TArray<FName> BroadphaseLayerNames; // index -> BroadPhaseLayer id

	// ANSI cache used by BPLayerInterfaceImpl::GetBroadPhaseLayerName — Jolt's profiler hook
	// returns const char* and we need stable storage owned by the table.
	TArray<TArray<ANSICHAR>> BroadphaseLayerNamesAnsi;

	// For each object layer, the broadphase layer it maps into.
	TArray<uint8> ObjectToBroadphase;

	// Flattened N*N symmetric mask: ObjectCollisionMask[a * N + b] == true if layer a collides with b.
	TArray<uint8> ObjectCollisionMask;

	// N_object * N_broadphase mask indexed by object_layer * NumBroadphase + broadphase_layer.
	TArray<uint8> ObjectVsBroadphaseMask;

	TMap<FName, int32> NameToObjectLayer;
	TMap<FName, int32> NameToBroadphaseLayer;

	int32 NumObjectLayers() const { return ObjectLayerNames.Num(); }
	int32 NumBroadphaseLayers() const { return BroadphaseLayerNames.Num(); }

	bool ObjectPairCollides(int32 a, int32 b) const
	{
		const int32 n = NumObjectLayers();
		if (a < 0 || b < 0 || a >= n || b >= n)
		{
			return false;
		}
		bool bCollides = ObjectCollisionMask[a * n + b] != 0;
		return bCollides;
	}

	bool ObjectVsBroadphaseCollides(int32 objectLayer, int32 broadphase) const
	{
		const int32 nObj = NumObjectLayers();
		const int32 nBp = NumBroadphaseLayers();
		if (objectLayer < 0 || broadphase < 0 || objectLayer >= nObj || broadphase >= nBp)
		{
			return false;
		}
		bool bCollides = ObjectVsBroadphaseMask[objectLayer * nBp + broadphase] != 0;
		return bCollides;
	}

	// Builds the table from the given settings. Safe to call even if settings contain stale data:
	// unknown broadphase references are dropped.
	static FJoltLayerTable BuildFromSettings(const UJoltSettings& Settings);
};