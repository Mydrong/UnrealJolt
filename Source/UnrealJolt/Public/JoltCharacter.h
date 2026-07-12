// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MyCharacter.h"
#include "Jolt/Physics/Character/Character.h"

class MyCharacter;
/**
 * 
 */
class UNREALJOLT_API FJoltCharacter : public MyCharacter
{
public:
	JPH_OVERRIDE_NEW_DELETE

	FJoltCharacter(
		const MyCharacterSettings* inSettings, JPH::RVec3Arg       inPosition, JPH::QuatArg& inRotation,
		uint64                     inUserData, JPH::PhysicsSystem* inSystem)
		: MyCharacter(inSettings, inPosition, inRotation, inUserData, inSystem)
		, mPhysicsSystem(inSystem)
		, mStandingShape(inSettings->mShape)
		, mCrouchingShape(inSettings->mCrouchingShape) {}

	JPH::PhysicsSystem*       mPhysicsSystem;
	JPH::RefConst<JPH::Shape> mStandingShape;
	JPH::RefConst<JPH::Shape> mCrouchingShape;

	bool sControlMovementDuringJump = true;
};