#include "PhysicsEngine/RigidBodiesNiagara.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Texture2D.h"
#include "Components/ArrowComponent.h"
#include "Components/BillboardComponent.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFloat.h"

namespace
{
	FVector RandPointInSphere(const FBoxSphereBounds& BoxSphere, const FVector& CenterPos)
	{
		FVector Point;
		float L;

		float RadiusSq = BoxSphere.SphereRadius * BoxSphere.SphereRadius;

		do
		{
			Point = FMath::RandPointInBox(BoxSphere.GetBox());
			L = (Point - CenterPos).SizeSquared();
		}
		while (L > RadiusSq);

		return Point;
	}

	// UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector()を参考にしている
	void SetNiagaraArrayVector(UNiagaraComponent* NiagaraSystem, FName OverrideName, const TArray<FVector>& ArrayData)
	{
		if (UNiagaraDataInterfaceArrayFloat3* ArrayDI = UNiagaraFunctionLibrary::GetDataInterface<UNiagaraDataInterfaceArrayFloat3>(NiagaraSystem, OverrideName))
		{
			FRWScopeLock WriteLock(ArrayDI->ArrayRWGuard, SLT_Write);
			ArrayDI->FloatData = ArrayData;
			ArrayDI->MarkRenderDataDirty();
		}
	}

	// UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayQuat()を参考にしている
	void SetNiagaraArrayQuat(UNiagaraComponent* NiagaraSystem, FName OverrideName, const TArray<FQuat>& ArrayData)
	{
		if (UNiagaraDataInterfaceArrayQuat* ArrayDI = UNiagaraFunctionLibrary::GetDataInterface<UNiagaraDataInterfaceArrayQuat>(NiagaraSystem, OverrideName))
		{
			FRWScopeLock WriteLock(ArrayDI->ArrayRWGuard, SLT_Write);
			ArrayDI->QuatData = ArrayData;
			ArrayDI->MarkRenderDataDirty();
		}
	}

	// UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor()を参考にしている
	void SetNiagaraArrayColor(UNiagaraComponent* NiagaraSystem, FName OverrideName, const TArray<FLinearColor>& ArrayData)
	{
		if (UNiagaraDataInterfaceArrayColor* ArrayDI = UNiagaraFunctionLibrary::GetDataInterface<UNiagaraDataInterfaceArrayColor>(NiagaraSystem, OverrideName))
		{
			FRWScopeLock WriteLock(ArrayDI->ArrayRWGuard, SLT_Write);
			ArrayDI->ColorData = ArrayData;
			ArrayDI->MarkRenderDataDirty();
		}
	}
}

void ARigidBodiesNiagara::BeginPlay()
{
	Super::BeginPlay();

	NumThreadParticles = (NumRigidBodies + NumThreads - 1) / NumThreads;
	NumPairs = (NumRigidBodies * (NumRigidBodies + 1)) / 2; //TODO: 総当たり

	Positions.SetNum(NumRigidBodies);
	Orientations.SetNum(NumRigidBodies);
	Scales.SetNum(NumRigidBodies);
	Colors.SetNum(NumRigidBodies);
	LinearVelocities.SetNum(NumRigidBodies);
	AngularVelocities.SetNum(NumRigidBodies);
	ContactPairs.Reserve(NumPairs); // コンタクトペアは最大でも総当たりペア数

	// InitPosRadius半径の球内にランダムに配置
	FBoxSphereBounds BoxSphere(InitPosCenter, FVector(InitPosRadius), InitPosRadius);
	for (int32 i = 0; i < NumRigidBodies; ++i)
	{
		Positions[i] = GetActorLocation() + RandPointInSphere(BoxSphere, InitPosCenter);
	}

	// とりあえずこれらは一定値固定で
	for (int32 i = 0; i < NumRigidBodies; ++i)
	{
		Orientations[i] = FQuat::Identity;
		Scales[i] = CubeScale;
		Colors[i] = FLinearColor::White;
		LinearVelocities[i] = FVector::ZeroVector;
		AngularVelocities[i] = FVector::ZeroVector;
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetVariableInt(FName("NumRigidBodies"), NumRigidBodies);
	NiagaraComponent->SetVariableVec3(FName("FloorScale"), FloorScale);
	NiagaraComponent->SetVariableVec3(FName("FloorPosition"), FloorPosition);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
	SetNiagaraArrayQuat(NiagaraComponent, FName("Orientations"), Orientations);
	SetNiagaraArrayVector(NiagaraComponent, FName("Scales"), Scales);
	SetNiagaraArrayColor(NiagaraComponent, FName("Colors"), Colors);
}

void ARigidBodiesNiagara::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSecondsの値の変動に関わらず、シミュレーションに使うサブステップタイムは固定とする
		float SubStepDeltaSeconds = 1.0f / FrameRate / NumIterations;

		for (int32 i = 0; i < NumIterations; ++i)
		{
			Simulate(SubStepDeltaSeconds);
		}
	}

	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
}

void ARigidBodiesNiagara::Simulate(float DeltaSeconds)
{
	ContactPairs.Reset();

	//TODO: コンタクトペア配列にマルチスレッドからアクセスするのが危険なのでとりあえずシングルスレッド
	DetectCollision();

	//TODO: 同じ剛体にマルチスレッドからアクセスするのが危険なのでとりあえずシングルスレッド
	SolveConstraint();

	// ApplyPressureが他のパーティクルの圧力値を使うので、すべて圧力値を計算してから別ループにする必要がある
	ParallelFor(NumThreads,
		[this, DeltaSeconds](int32 ThreadIndex)
		{
			for (int32 RBIdx = NumThreadParticles * ThreadIndex; RBIdx < NumThreadParticles * (ThreadIndex + 1) && RBIdx < NumRigidBodies; ++RBIdx)
			{
				Integrate(RBIdx, DeltaSeconds);
			}
		}
	);
}

void ARigidBodiesNiagara::DetectCollision()
{
	for (int32 i = 0; i < NumRigidBodies; ++i)
	{
		for (int32 j = i + 1; j < NumRigidBodies; ++j)
		{
		}
	}
}

void ARigidBodiesNiagara::SolveConstraint()
{
	for (const FContactPair& ContactPair : ContactPairs)
	{
	}
}

void ARigidBodiesNiagara::Integrate(int32 RBIdx, float DeltaSeconds)
{
	LinearVelocities[RBIdx] += FVector(0.0f, 0.0f, Gravity) * DeltaSeconds;
	// TODO:仮。発散しないように
	LinearVelocities[RBIdx].Z = FMath::Max(LinearVelocities[RBIdx].Z, -1000.0f);

	Positions[RBIdx] += LinearVelocities[RBIdx] * DeltaSeconds;
	// TODO:仮。発散しないように
	Positions[RBIdx].Z = FMath::Max(Positions[RBIdx].Z, 25.0f);

	const FQuat& OrientationDifferential = FQuat(AngularVelocities[RBIdx].X, AngularVelocities[RBIdx].Y, AngularVelocities[RBIdx].Z, 0.0f) * Orientations[RBIdx] * 0.5f;
	Orientations[RBIdx] = (Orientations[RBIdx] + OrientationDifferential * DeltaSeconds).GetNormalized();
}

ARigidBodiesNiagara::ARigidBodiesNiagara()
{
	PrimaryActorTick.bCanEverTick = true;

	NiagaraComponent = CreateDefaultSubobject<UNiagaraComponent>(TEXT("NiagaraComponent0"));

	RootComponent = NiagaraComponent;

#if WITH_EDITORONLY_DATA
	SpriteComponent = CreateEditorOnlyDefaultSubobject<UBillboardComponent>(TEXT("Sprite"));
	ArrowComponent = CreateEditorOnlyDefaultSubobject<UArrowComponent>(TEXT("ArrowComponent0"));

	if (!IsRunningCommandlet())
	{
		// Structure to hold one-time initialization
		struct FConstructorStatics
		{
			ConstructorHelpers::FObjectFinderOptional<UTexture2D> SpriteTextureObject;
			FName ID_Effects;
			FText NAME_Effects;
			FConstructorStatics()
				: SpriteTextureObject(TEXT("/Niagara/Icons/S_ParticleSystem"))
				, ID_Effects(TEXT("Effects"))
				, NAME_Effects(NSLOCTEXT("SpriteCategory", "Effects", "Effects"))
			{
			}
		};
		static FConstructorStatics ConstructorStatics;

		if (SpriteComponent)
		{
			SpriteComponent->Sprite = ConstructorStatics.SpriteTextureObject.Get();
			SpriteComponent->SetRelativeScale3D_Direct(FVector(0.5f, 0.5f, 0.5f));
			SpriteComponent->bHiddenInGame = true;
			SpriteComponent->bIsScreenSizeScaled = true;
			SpriteComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			SpriteComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			SpriteComponent->SetupAttachment(NiagaraComponent);
			SpriteComponent->bReceivesDecals = false;
		}

		if (ArrowComponent)
		{
			ArrowComponent->ArrowColor = FColor(0, 255, 128);

			ArrowComponent->ArrowSize = 1.5f;
			ArrowComponent->bTreatAsASprite = true;
			ArrowComponent->bIsScreenSizeScaled = true;
			ArrowComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			ArrowComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			ArrowComponent->SetupAttachment(NiagaraComponent);
			ArrowComponent->SetUsingAbsoluteScale(true);
		}
	}
#endif // WITH_EDITORONLY_DATA
}

void ARigidBodiesNiagara::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ARigidBodiesNiagara::OnNiagaraSystemFinished);
	}
}

void ARigidBodiesNiagara::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ARigidBodiesNiagara::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ARigidBodiesNiagara::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ARigidBodiesNiagara::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR

