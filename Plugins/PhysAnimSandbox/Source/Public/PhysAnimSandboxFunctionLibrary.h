#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysAnimSandboxFunctionLibrary.generated.h"

UCLASS()
class PHYSANIMSANDBOX_API UPhysAnimSandboxFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

#if WITH_EDITOR
	/** 
	 * Create skeletal mesh and skeleton asset.
	 * @return True if the operation succeeds, check the log for errors if it didn't succeed.
	 */
	UFUNCTION(BlueprintCallable)
	static bool CreateSkeletalMesh();
#endif
};
