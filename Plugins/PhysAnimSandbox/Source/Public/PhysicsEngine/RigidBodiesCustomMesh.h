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

private:
	UPROPERTY(EditAnywhere)
	int32 NumThreads = 4;

	UPROPERTY(EditAnywhere)
	int32 NumIterations = 1;

	UPROPERTY(EditAnywhere)
	float FrameRate = 60.0f;

	UPROPERTY(EditAnywhere)
	float Gravity = -981.0f;

	UPROPERTY(EditAnywhere)
	int32 NumRigidBodies = 1;

	UPROPERTY(EditAnywhere)
	float Mass = 0.1f;

	UPROPERTY(EditAnywhere)
	float InitPosRadius = 50.0f;

	UPROPERTY(EditAnywhere)
	FVector InitPosCenter = FVector(0.0f, 0.0f, 300.0f);

	UPROPERTY(EditAnywhere)
	FVector CubeScale = FVector(10.0f, 10.0f, 10.0f); // for 1cm x 1cm x 1cm cube.

	UPROPERTY(EditAnywhere)
	FVector FloorScale = FVector(1000.0f, 1000.0f, 50.0f); // for 1cm x 1cm x 1cm cube.

	UPROPERTY(EditAnywhere)
	FVector FloorPosition = FVector(0.0f, 0.0f, -25.0f);

private:
	struct FEdge
	{
		int32 VertId[2];
		int32 FacetId[2];
	};

	struct FFacet
	{
		int32 VertId[3];
		int32 EdgeId[3];
		FVector Normal;
	};

	struct FCollisionShape
	{
		TArray<FVector> Vertices;
		TArray<FEdge> Edges;
		TArray<FFacet> Facets;
	};

	struct FRigidBody
	{
		FCollisionShape CollisionShape;

		FVector Position = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		FVector LinearVelocity = FVector::ZeroVector;
		FVector AngularVelocity = FVector::ZeroVector;

		class UCustomMeshComponent* DrawMesh;
	};

	TArray<FRigidBody> RigidBodies;
};
