#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "RigidBodiesCustomMesh.generated.h"

UCLASS()
class ARigidBodiesCustomMesh : public AActor
{
	GENERATED_BODY()

public:
	ARigidBodiesCustomMesh();

	virtual void BeginPlay() override;
	virtual void Tick( float DeltaSeconds ) override;
};

