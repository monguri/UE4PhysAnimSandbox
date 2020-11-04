#include "AnimNode_DrawPhysicsAsset.h"
#include "Animation/AnimInstanceProxy.h"
#include "Components/SkeletalMeshComponent.h"
#include "ReferenceSkeleton.h"
#include "DrawDebugHelpers.h"

void FAnimNode_DrawPhysicsAsset::OnInitializeAnimInstance(const FAnimInstanceProxy* InProxy, const UAnimInstance* InAnimInstance)
{
	// FAnimNode_RigidBody::OnInitializeAnimInstance()���Q�l�ɂ��Ă���
	const USkeletalMeshComponent* SkeletalMeshComp = InAnimInstance->GetSkelMeshComponent();
	const USkeletalMesh* SkeletalMeshAsset = SkeletalMeshComp->SkeletalMesh;

	const FReferenceSkeleton& SkelMeshRefSkel = SkeletalMeshAsset->RefSkeleton;
	UsePhysicsAsset = OverridePhysicsAsset ? OverridePhysicsAsset : InAnimInstance->GetSkelMeshComponent()->GetPhysicsAsset();

	// UPhysicsAssetEditorSkeletalMeshComponent::UPhysicsAssetEditorSkeletalMeshComponent()���Q�l�ɂ��Ă���
	// Body materials
	UMaterialInterface* BaseElemSelectedMaterial = LoadObject<UMaterialInterface>(NULL, TEXT("/Engine/EditorMaterials/PhAT_ElemSelectedMaterial.PhAT_ElemSelectedMaterial"), NULL, LOAD_None, NULL);
	ElemSelectedMaterial = UMaterialInstanceDynamic::Create(BaseElemSelectedMaterial, GetTransientPackage());
}

bool FAnimNode_DrawPhysicsAsset::IsValidToEvaluate(const class USkeleton* Skeleton, const struct FBoneContainer& RequiredBones)
{
	return (UsePhysicsAsset != nullptr && ElemSelectedMaterial != nullptr);
}

namespace
{
	// UPhysicsAssetEditorSkeletalMeshComponent::GetPrimitiveTransform()���Q�l�ɂ��Ă���
	FTransform GetPrimitiveTransform(UBodySetup* SharedBodySetup, FTransform& BoneTM, int32 BodyIndex, EAggCollisionShape::Type PrimType, int32 PrimIndex, float Scale)
	{
		FVector Scale3D(Scale);

		if (PrimType == EAggCollisionShape::Sphere)
		{
			FTransform PrimTM = SharedBodySetup->AggGeom.SphereElems[PrimIndex].GetTransform();
			PrimTM.ScaleTranslation(Scale3D);
			return PrimTM * BoneTM;
		}
		else if (PrimType == EAggCollisionShape::Box)
		{
			FTransform PrimTM = SharedBodySetup->AggGeom.BoxElems[PrimIndex].GetTransform();
			PrimTM.ScaleTranslation(Scale3D);
			return PrimTM * BoneTM;
		}
		else if (PrimType == EAggCollisionShape::Sphyl)
		{
			FTransform PrimTM = SharedBodySetup->AggGeom.SphylElems[PrimIndex].GetTransform();
			PrimTM.ScaleTranslation(Scale3D);
			return PrimTM * BoneTM;
		}
		else if (PrimType == EAggCollisionShape::Convex)
		{
			FTransform PrimTM = SharedBodySetup->AggGeom.ConvexElems[PrimIndex].GetTransform();
			PrimTM.ScaleTranslation(Scale3D);
			return PrimTM * BoneTM;
		}
		else if (PrimType == EAggCollisionShape::TaperedCapsule)
		{
			FTransform PrimTM = SharedBodySetup->AggGeom.TaperedCapsuleElems[PrimIndex].GetTransform();
			PrimTM.ScaleTranslation(Scale3D);
			return PrimTM * BoneTM;
		}

		// Should never reach here
		check(0);
		return FTransform::Identity;
	}
}

void FAnimNode_DrawPhysicsAsset::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	const USkeletalMeshComponent* SkeletalMeshComp = Output.AnimInstanceProxy->GetSkelMeshComponent();
	UWorld* World = GEngine->GetWorldFromContextObject(SkeletalMeshComp, EGetWorldErrorMode::LogAndReturnNull);
	const FTransform& CStoWS = SkeletalMeshComp->GetComponentToWorld();

	// set opacity of our materials
	static FName OpacityName(TEXT("Opacity"));
	ElemSelectedMaterial->SetScalarParameterValue(OpacityName, 0.5f); // 0.5f�͓K��

	// TODO:������ւ�����āA���W���[������Slate��EditorStyle�̈ˑ���������������
	static FName SelectionColorName(TEXT("SelectionColor"));
	const FSlateColor SelectionColor = FEditorStyle::GetSlateColor(SelectionColorName);
	const FLinearColor LinearSelectionColor(SelectionColor.IsColorSpecified() ? SelectionColor.GetSpecifiedColor() : FLinearColor::White);

	ElemSelectedMaterial->SetVectorParameterValue(SelectionColorName, LinearSelectionColor);

	const FColor ElemSelectedColor = LinearSelectionColor.ToFColor(true);

	// UPhysicsAssetEditorSkeletalMeshComponent::RenderAssetTools()���Q�l�ɂ��Ă���

	// Draw bodies
	for (int32 i = 0; i <UsePhysicsAsset->SkeletalBodySetups.Num(); ++i)
	{
		if (!ensure(UsePhysicsAsset->SkeletalBodySetups[i]))
		{
			continue;
		}
		int32 BoneIndex = SkeletalMeshComp->GetBoneIndex(UsePhysicsAsset->SkeletalBodySetups[i]->BoneName);

		// If we found a bone for it, draw the collision.
		// The logic is as follows; always render in the ViewMode requested when not in hit mode - but if we are in hit mode and the right editing mode, render as solid
		if (BoneIndex != INDEX_NONE)
		{
			FTransform BoneTM = SkeletalMeshComp->GetBoneTransform(BoneIndex);
			float Scale = BoneTM.GetScale3D().GetAbsMax();
			FVector VectorScale(Scale);
			BoneTM.RemoveScaling();

			FKAggregateGeom* AggGeom = &UsePhysicsAsset->SkeletalBodySetups[i]->AggGeom;

			for (int32 j = 0; j <AggGeom->SphereElems.Num(); ++j)
			{
				FTransform ElemTM = AggGeom->SphereElems[j].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM * CStoWS;

#if 0
				FFunctionGraphTask::CreateAndDispatchWhenReady(
					[=]() {
						bool bPersistent = false;
						float LifeTime = 0.0f;
						const FColor& ShapeColor = FColor::Blue;

						switch (Shape)
						{
							case ETngTwoBoneFootIKQueryShape::Line:
								::DrawDebugPoint(World, InitialShapeWSPos, 5.0f, ShapeColor, bPersistent, LifeTime, ESceneDepthPriorityGroup::SDPG_Foreground);
								break;
							case ETngTwoBoneFootIKQueryShape::Sphere:
								::DrawDebugSphere(World, InitialShapeWSPos, HalfExtent.X, 16, ShapeColor, bPersistent, LifeTime, ESceneDepthPriorityGroup::SDPG_Foreground);
								break;
							case ETngTwoBoneFootIKQueryShape::Box:
								::DrawDebugBox(World, InitialShapeWSPos, HalfExtent, ShapeWSRot, ShapeColor, bPersistent, LifeTime, ESceneDepthPriorityGroup::SDPG_Foreground);
								break;
							default:
								UE_LOG(LogTemp, Error, TEXT("Shape=%d is invalid value."), Shape);
								break;
						}
					},
					TStatId(), nullptr, ENamedThreads::GameThread
				);
#endif
				FFunctionGraphTask::CreateAndDispatchWhenReady(
					[World, ElemTM, AggGeom, j, Scale, ElemSelectedColor]() {
						bool bPersistent = false;
						float LifeTime = 0.0f;
						const FColor& ShapeColor = FColor::Blue;

						::DrawDebugSphere(World, ElemTM.GetLocation(), AggGeom->SphereElems[j].Radius * Scale, 16, ElemSelectedColor, bPersistent, LifeTime, ESceneDepthPriorityGroup::SDPG_Foreground);
					},
					TStatId(), nullptr, ENamedThreads::GameThread
				);
			}

			for (int32 j = 0; j <AggGeom->BoxElems.Num(); ++j)
			{
				FTransform ElemTM = AggGeom->BoxElems[j].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM * CStoWS;

				FFunctionGraphTask::CreateAndDispatchWhenReady(
					[World, ElemTM, AggGeom, j, Scale, ElemSelectedColor]() {
						bool bPersistent = false;
						float LifeTime = 0.0f;

						::DrawDebugBox(World, ElemTM.GetLocation(), Scale * 0.5f * FVector(AggGeom->BoxElems[j].X, AggGeom->BoxElems[j].Y, AggGeom->BoxElems[j].Z), ElemTM.GetRotation(), ElemSelectedColor, bPersistent, LifeTime, ESceneDepthPriorityGroup::SDPG_Foreground);
					},
					TStatId(), nullptr, ENamedThreads::GameThread
				);
			}

			for (int32 j = 0; j <AggGeom->SphylElems.Num(); ++j)
			{
				FTransform ElemTM = AggGeom->SphylElems[j].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM * CStoWS;

				FFunctionGraphTask::CreateAndDispatchWhenReady(
					[World, ElemTM, AggGeom, j, Scale, ElemSelectedColor]() {
						bool bPersistent = false;
						float LifeTime = 0.0f;
						const FColor& ShapeColor = FColor::Blue;

						::DrawDebugCylinder(World, ElemTM.GetLocation() + ElemTM.GetUnitAxis(EAxis::Type::Z) * AggGeom->SphylElems[j].Length * Scale * 0.5f, ElemTM.GetLocation() - ElemTM.GetUnitAxis(EAxis::Type::Z) * AggGeom->SphylElems[j].Length * Scale * 0.5f, AggGeom->SphylElems[j].Radius * Scale, 16, ElemSelectedColor, bPersistent, LifeTime, ESceneDepthPriorityGroup::SDPG_Foreground);
					},
					TStatId(), nullptr, ENamedThreads::GameThread
				);
			}

			for (int32 j = 0; j <AggGeom->ConvexElems.Num(); ++j)
			{
				FTransform ElemTM = AggGeom->ConvexElems[j].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM * CStoWS;

#if 0 // TODO:�Ƃ肠�����ȗ�
				//convex doesn't have solid draw so render lines if we're in hitTestAndBodyMode
				AggGeom->ConvexElems[j].DrawElemWire(PDI, ElemTM, Scale, ElemSelectedColor);
#endif
			}

			for (int32 j = 0; j <AggGeom->TaperedCapsuleElems.Num(); ++j)
			{
				FTransform ElemTM = AggGeom->TaperedCapsuleElems[j].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM * CStoWS;

#if 0 // TODO:�Ƃ肠�����ȗ�
				AggGeom->TaperedCapsuleElems[j].DrawElemSolid(PDI, ElemTM, VectorScale, ElemSelectedMaterial->GetRenderProxy());
				AggGeom->TaperedCapsuleElems[j].DrawElemWire(PDI, ElemTM, VectorScale, ElemSelectedColor);
#endif
			}
		}
	}
}

