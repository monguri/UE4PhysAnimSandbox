// Copyright Epic Games, Inc. All Rights Reserved.

#include "UE4PhysAnimSandboxGameMode.h"
#include "UE4PhysAnimSandboxHUD.h"
#include "UE4PhysAnimSandboxCharacter.h"
#include "UObject/ConstructorHelpers.h"

AUE4PhysAnimSandboxGameMode::AUE4PhysAnimSandboxGameMode()
	: Super()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnClassFinder(TEXT("/Game/FirstPersonCPP/Blueprints/FirstPersonCharacter"));
	DefaultPawnClass = PlayerPawnClassFinder.Class;

	// use our custom HUD class
	HUDClass = AUE4PhysAnimSandboxHUD::StaticClass();
}
