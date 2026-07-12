// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UnrealJolt/JoltMain.h"
#include "JoltBodyID.generated.h"

USTRUCT(BlueprintType)
struct UNREALJOLT_API FJoltBodyID
{
	GENERATED_BODY()

	JPH::BodyID BodyID;

	FJoltBodyID() = default;

	FJoltBodyID(const JPH::BodyID& InBodyID)
		: BodyID(InBodyID) {}

	static FJoltBodyID FromJoltBodyID(const JPH::BodyID& InBodyID)
	{
		return FJoltBodyID(InBodyID);
	}

	bool IsValid() const
	{
		return !BodyID.IsInvalid();
	}

	JPH::BodyID ToJoltBodyID() const
	{
		return BodyID;
	}

	uint32 ToIndexAndSequenceNumber() const
	{
		return BodyID.GetIndexAndSequenceNumber();
	}

	friend bool operator==(const FJoltBodyID& Left, const FJoltBodyID& Right)
	{
		return Left.BodyID.GetIndexAndSequenceNumber() == Right.BodyID.GetIndexAndSequenceNumber();
	}

	friend uint32 GetTypeHash(const FJoltBodyID& BodyID)
	{
		return BodyID.BodyID.GetIndexAndSequenceNumber();
	}
};