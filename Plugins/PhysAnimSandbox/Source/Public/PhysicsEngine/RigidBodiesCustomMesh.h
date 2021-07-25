#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "RigidBodiesCustomMesh.generated.h"


USTRUCT()
struct FRigidBodySetting
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	float Friction = 0.2f;

	UPROPERTY(EditAnywhere)
	float Restitution = 0.6f;

	UPROPERTY(EditAnywhere)
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere)
	FVector Location = FVector::ZeroVector;

	UPROPERTY(EditAnywhere)
	FRotator Rotation = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere)
	FVector Scale = FVector::ZeroVector;
};

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
	int32 NumIterations = 10;

	UPROPERTY(EditAnywhere)
	float FrameRate = 60.0f;

	UPROPERTY(EditAnywhere)
	float Gravity = -981.0f;
	UPROPERTY(EditAnywhere)
	float ContactBias = 0.1f;

	UPROPERTY(EditAnywhere)
	float ContactSlop = 0.1f;

	UPROPERTY(EditAnywhere)
	FVector FloorLocation = FVector(0.0f, 0.0f, -25.0f);

	UPROPERTY(EditAnywhere)
	FRotator FloorRotation = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere)
	FVector FloorScale = FVector(1000.0f, 1000.0f, 50.0f); // for 1cm x 1cm x 1cm cube.

	UPROPERTY(EditAnywhere)
	float FloorFriction = 0.6f;

	UPROPERTY(EditAnywhere)
	float FloorRestitution = 0.2f;

	UPROPERTY(EditAnywhere)
	bool bDirectSet = false;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	float InitPosRadius = 50.0f;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	FVector InitPosCenter = FVector(0.0f, 0.0f, 300.0f);

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	int32 NumRigidBodies = 1;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	float Friction = 0.6f;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	float Restitution = 0.2f;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	float Density = 0.1f;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	FRotator CubeRot = FRotator::ZeroRotator;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "!bDirectSet"))
	FVector CubeScale = FVector(10.0f, 10.0f, 10.0f); // for 1cm x 1cm x 1cm cube.

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "bDirectSet"))
	TArray<FRigidBodySetting> RigidBodySettings;

public:
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

		float Mass = 1.0f;
		float Friction = 0.2f;
		float Restitution = 0.6f;
		FMatrix Inertia = FMatrix::Identity;
		FVector Position = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		FVector LinearVelocity = FVector::ZeroVector;
		FVector AngularVelocity = FVector::ZeroVector;
	};

private:
	TArray<FRigidBody> RigidBodies;
	int32 NumThreadRBs = 0;

	struct FConstraint
	{
		FVector Axis = FVector::ZeroVector; // 拘束軸
		float JacobianDiagInv = 0.0f; // 拘束式の分母
		float RHS = 0.0f; // 初期拘束インパルス
		float LowerLimit = 0.0f; // 拘束インパルスの下限
		float UpperLimit = 0.0f; // 拘束インパルスの上限
		float AccumImpulse = 0.0f; // 蓄積された拘束インパルス

		void Reset()
		{
			AccumImpulse = 0.0f;
		}
	};

	struct FContact
	{
		FVector ContactPointA = FVector::ZeroVector; // 剛体Aのローカル座標での剛体A側のコンタクトポイント
		FVector ContactPointB = FVector::ZeroVector; // 剛体Bのローカル座標での剛体B側のコンタクトポイント
		FVector Normal = FVector::ZeroVector; // ワールド座標。Aを反発させる方向にとる。
		float PenetrationDepth = 0.0f; // 貫通深度。貫通している場合は負。
		FConstraint Constraints[3];

		void Reset()
		{
			for (int32 i = 0; i < 3; ++i)
			{
				Constraints[i].Reset();
			}
		}
	};

	enum class EContactPairState : uint8
	{
		NoContact,
		New,
		Keep,
	};

	struct FContactPair
	{
		EContactPairState State = EContactPairState::NoContact; 
		int32 RigidBodyA_Idx = 0;
		int32 RigidBodyB_Idx = 0;

		float Friction = 0.0f;

		int32 NumContact = 0;
		FContact Contacts[4];

		void Refresh(const FVector& PositionA, const FQuat& OrientationA, const FVector& PositionB, const FQuat& OrientationB);
		void AddContact(const FVector& ContactPointA, const FVector& ContactPointB, const FVector& Normal, float PenetrationDepth);
		void RemoveContact(int32 Index);
		int32 FindNearestContact(const FVector& ContactPointA, const FVector& ContactPointB, const FVector& Normal);
		int32 ChooseSwapContact(const FVector& NewPointA, float NewPenetrationDepth);
	};

	TArray<FContactPair> ContactPairs;

	struct FSolverBody
	{
		FVector DeltaLinearVelocity = FVector::ZeroVector;
		FVector DeltaAngularVelocity = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		float MassInv = 0.0f;
		FMatrix InertiaInv = FMatrix::Identity; // 4x4だが3x3行列として使う
	};
	TArray<FSolverBody> SolverBodies;

	/** Pointer to custom mesh component */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UCustomMeshComponent* DrawMesh;

	void Simulate(float DeltaSeconds);
	void DetectCollision();
	void SolveConstraint(float DeltaSeconds);
	void Integrate(int32 RBIdx, float DeltaSeconds);
	void ApplyRigidBodiesToMeshDrawing();
};

