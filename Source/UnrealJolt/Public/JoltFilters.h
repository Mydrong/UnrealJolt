#pragma once

#include "CoreMinimal.h"
#include "JoltLayerTable.h"
#include "UnrealJolt/JoltMain.h"

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
{
public:
	explicit ObjectLayerPairFilterImpl(const FJoltLayerTable& InTable)
		: Table(InTable) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
	{
		return Table.ObjectPairCollides(static_cast<int32>(inObject1), static_cast<int32>(inObject2));
	}

private:
	const FJoltLayerTable& Table;
};

// BroadPhaseLayerInterface implementation
// Data-driven: reads the object->broadphase mapping and layer count out of FJoltLayerTable.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
	explicit BPLayerInterfaceImpl(const FJoltLayerTable& InTable)
		: Table(InTable) {}

	virtual uint GetNumBroadPhaseLayers() const override
	{
		return static_cast<uint>(Table.NumBroadphaseLayers());
	}

	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
	{
		const int32 idx = static_cast<int32>(inLayer);
		JPH_ASSERT(Table.ObjectToBroadphase.IsValidIndex(idx));
		return JPH::BroadPhaseLayer(Table.ObjectToBroadphase[idx]);
	}

	#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
	{
		const int32 idx = static_cast<int32>(static_cast<JPH::BroadPhaseLayer::Type>(inLayer));
		if (Table.BroadphaseLayerNamesAnsi.IsValidIndex(idx) && Table.BroadphaseLayerNamesAnsi[idx].Num() > 0)
		{
			return Table.BroadphaseLayerNamesAnsi[idx].GetData();
		}
		return "BroadPhaseLayer";
	}
	#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	const FJoltLayerTable& Table;
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	explicit ObjectVsBroadPhaseLayerFilterImpl(const FJoltLayerTable& InTable)
		: Table(InTable) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
	{
		const int32 bp = static_cast<int32>(static_cast<JPH::BroadPhaseLayer::Type>(inLayer2));
		return Table.ObjectVsBroadphaseCollides(static_cast<int32>(inLayer1), bp);
	}

private:
	const FJoltLayerTable& Table;
};

class SaveStateFilter final : public JPH::StateRecorderFilter
{
public:
	virtual bool ShouldSaveBody(const JPH::Body& inBody) const override
	{
		for (int i = 0; i < AllowedBodiesList.size(); i++)
		{
			if (AllowedBodiesList[i] == inBody.GetID())
			{
				return true;
			}
		}
		return false;
	}

	void AddToBodyIDAllowList(const JPH::BodyID& bodyID)
	{
		AllowedBodiesList.insert(AllowedBodiesList.begin(), bodyID);
	}

private:
	JPH::Array<JPH::BodyID> AllowedBodiesList;
};

class ObjectLayersFilter_ForObjects_UE final : public JPH::ObjectLayerFilter
{
public:
	explicit ObjectLayersFilter_ForObjects_UE(
		const TSet<FName>& layerNames, const FJoltLayerTable& layerTable)
		: layerNames(layerNames)
		, layerTable(layerTable) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override
	{
		return layerNames.IsEmpty() || layerNames.Contains(layerTable.ObjectLayerNames[static_cast<int32>(inLayer)]);
	}

private:
	const TSet<FName>&     layerNames;
	const FJoltLayerTable& layerTable;
};

class BroadPhaseLayersFilter_ForObjects_UE final : public JPH::BroadPhaseLayerFilter
{
public:
	explicit BroadPhaseLayersFilter_ForObjects_UE(
		const TSet<FName>& layerNames, const FJoltLayerTable& layerTable)
		: layerNames(layerNames)
		, layerTable(layerTable) {}

	virtual bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override
	{
		return layerNames.IsEmpty() || layerNames.Contains(layerTable.BroadphaseLayerNames[static_cast<uint8>(inLayer)]);
	}

private:
	const TSet<FName>&     layerNames;
	const FJoltLayerTable& layerTable;
};

class ObjectLayersFilter_ByLayers_UE final : public JPH::ObjectLayerFilter
{
public:
	explicit ObjectLayersFilter_ByLayers_UE(
		const TArray<FName>& layerNames, const FJoltLayerTable& layerTable)
		: layerTable(layerTable)
	{
		layers.Reserve(layerNames.Num());
		for (const FName& LayerName : layerNames)
			layers.Emplace(layerTable.NameToObjectLayer[LayerName]);
	}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override
	{
		if (layers.IsEmpty())
			return true;

		for (int32 layer : layers)
		{
			if (layerTable.ObjectPairCollides(inLayer, layer))
				return true;
		}
		return false;
	}

private:
	TArray<int32>          layers;
	const FJoltLayerTable& layerTable;
};

class BroadPhaseLayersFilter_ByLayers_UE final : public JPH::BroadPhaseLayerFilter
{
public:
	explicit BroadPhaseLayersFilter_ByLayers_UE(
		const TArray<FName>& layerNames, const FJoltLayerTable& layerTable)
		: layerTable(layerTable)
	{
		layers.Reserve(layerNames.Num());
		for (const FName& LayerName : layerNames)
			layers.Emplace(layerTable.NameToObjectLayer[LayerName]);
	}

	virtual bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override
	{
		if (layers.IsEmpty())
			return true;

		for (int32 layer : layers)
		{
			if (layerTable.ObjectPairCollides(inLayer.GetValue(), layer))
				return true;
		}
		return false;
	}

private:
	TArray<int32>          layers;
	const FJoltLayerTable& layerTable;
};