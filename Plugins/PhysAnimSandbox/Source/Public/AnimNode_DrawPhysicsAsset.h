#pragma once

#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_DrawPhysicsAsset.generated.h"

/**
 *	Debug draw physics asset.
 */
USTRUCT()
struct PHYSANIMSANDBOX_API FAnimNode_DrawPhysicsAsset : public FAnimNode_SkeletalControlBase
{
	GENERATED_BODY()

public:
	/** Physics asset to use. If empty use the skeletal mesh's default physics asset */
	UPROPERTY(EditAnywhere, Category = Settings)
	UPhysicsAsset* OverridePhysicsAsset = nullptr;

private:
	UPhysicsAsset* UsePhysicsAsset = nullptr;
};

