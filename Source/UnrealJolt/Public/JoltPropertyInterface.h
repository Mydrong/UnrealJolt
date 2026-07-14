// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "UObject/Interface.h"
#include "JoltPropertyInterface.generated.h"

// This class does not need to be modified.
UINTERFACE()
class UJoltPropertyInterface : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class UNREALJOLT_API IJoltPropertyInterface
{
	GENERATED_BODY()

public:
	virtual FName GetLayer() const { return NAME_None; };
	virtual void  SetLayer(FName Layer) {};

	virtual float GetFriction() const { return 0; };
	virtual void  SetFriction(float Friction) {};

	virtual float GetRestitution() const { return 0; };
	virtual void  SetRestitution(float Restitution) {};

	virtual UPhysicalMaterial* GetPhysicalMaterial() const { return nullptr; };
	virtual void               SetPhysicalMaterial(UPhysicalMaterial* PhysicalMaterial) {};
};