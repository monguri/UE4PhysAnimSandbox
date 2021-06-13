#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "RigidBodies.generated.h"

UCLASS()
// ANiagaraActorÇéQçlÇ…ÇµÇƒÇ¢ÇÈ
class ARigidBodies : public AActor
{
	GENERATED_BODY()

public:
	ARigidBodies();

	virtual void PostRegisterAllComponents() override;

	virtual void BeginPlay() override;
	virtual void Tick( float DeltaSeconds ) override;

	/** Set true for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UFUNCTION(BlueprintCallable)
	void SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish);

private:
	/** Pointer to System component */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UNiagaraComponent* NiagaraComponent;

#if WITH_EDITORONLY_DATA
	// Reference to sprite visualization component
	UPROPERTY()
	class UBillboardComponent* SpriteComponent;

	// Reference to arrow visualization component
	UPROPERTY()
	class UArrowComponent* ArrowComponent;

#endif

	/** True for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UPROPERTY()
	uint32 bDestroyOnSystemFinish : 1;

	/** Callback when Niagara system finishes. */
	UFUNCTION(CallInEditor)
	void OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent);

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
	FVector CubeScale = FVector(0.1f, 0.1f, 0.1f); // for 1m x 1m x 1m cube.

	UPROPERTY(EditAnywhere)
	FVector FloorScale = FVector(10.0f, 10.0f, 0.5f); // for 1m x 1m x 1m cube.

	UPROPERTY(EditAnywhere)
	FVector FloorPosition = FVector(0.0f, 0.0f, -25.0f);

private:
	void Simulate(float DeltaSeconds);
	void DetectCollision();
	void SolveConstraint();
	void Integrate(int32 RBIdx, float DeltaSeconds);

private:
	TArray<FVector> Positions;
	TArray<FQuat> Orientations;
	TArray<FVector> Scales;
	TArray<FLinearColor> Colors;
	TArray<FVector> LinearVelocities;
	TArray<FVector> AngularVelocities;

	struct FContactPair
	{
		int32 RigidBodyA_Idx;
		int32 RigidBodyB_Idx;
	};
	TArray<FContactPair> ContactPairs;

	int32 NumThreadParticles = 0;
	int32 NumPairs = 0;

public:
	/** Returns NiagaraComponent subobject **/
	class UNiagaraComponent* GetNiagaraComponent() const { return NiagaraComponent; }
#if WITH_EDITORONLY_DATA
	/** Returns SpriteComponent subobject **/
	class UBillboardComponent* GetSpriteComponent() const { return SpriteComponent; }
	/** Returns ArrowComponent subobject **/
	class UArrowComponent* GetArrowComponent() const { return ArrowComponent; }
#endif

#if WITH_EDITOR
	// AActor interface
	virtual bool GetReferencedContentObjects(TArray<UObject*>& Objects) const override;
	// End of AActor interface

	/** Reset this actor in the level.*/
	void ResetInLevel();
#endif // WITH_EDITOR
};

