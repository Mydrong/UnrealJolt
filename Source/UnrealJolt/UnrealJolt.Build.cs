// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class UnrealJolt : ModuleRules
{
	public UnrealJolt(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		FPSemantics = FPSemanticsMode.Precise;

		PublicDependencyModuleNames.AddRange(
				new string[]
				{
				"Core",
				"CoreUObject",
				"Engine",
				"DeveloperSettings",
				"InputCore",
				"UnrealJoltLibrary",
				"Projects",
				"Landscape",
				"PhysicsCore",
				"Chaos",
				"ChaosCore",
				"Slate",
				"SlateCore",
				}
				);
		if (Target.bBuildEditor)
		{
			PublicDependencyModuleNames.AddRange(new string[] { "UnrealEd", "EditorSubsystem" });
		}
	}
}
