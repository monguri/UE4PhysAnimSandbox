#include "AnimNode_DrawPhysicsAsset.h"
#include "Animation/AnimInstanceProxy.h"
#include "Components/SkeletalMeshComponent.h"
#include "ReferenceSkeleton.h"

void FAnimNode_DrawPhysicsAsset::OnInitializeAnimInstance(const FAnimInstanceProxy* InProxy, const UAnimInstance* InAnimInstance)
{
	const USkeletalMeshComponent* SkeletalMeshComp = InAnimInstance->GetSkelMeshComponent();
	const USkeletalMesh* SkeletalMeshAsset = SkeletalMeshComp->SkeletalMesh;

	const FReferenceSkeleton& SkelMeshRefSkel = SkeletalMeshAsset->RefSkeleton;
	UsePhysicsAsset = OverridePhysicsAsset ? OverridePhysicsAsset : InAnimInstance->GetSkelMeshComponent()->GetPhysicsAsset();

	// Body materials
	UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(NULL, TEXT("/Engine/EditorMaterials/PhAT_ElemSelectedMaterial.PhAT_ElemSelectedMaterial"), NULL, LOAD_None, NULL);
	BodyMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, GetTransientPackage());
	check(BodyMaterial);
}

bool FAnimNode_DrawPhysicsAsset::IsValidToEvaluate(const class USkeleton* Skeleton, const struct FBoneContainer& RequiredBones)
{
	return (UsePhysicsAsset != nullptr && BodyMaterial != nullptr);
}

void FAnimNode_DrawPhysicsAsset::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	// Draw bodies
	for (int32 i = 0; i <UsePhysicsAsset->SkeletalBodySetups.Num(); ++i)
	{
		if (!ensure(UsePhysicsAsset->SkeletalBodySetups[i]))
		{
			continue;
		}
		if ((UsePhysicsAsset->SkeletalBodySetups[i]->PhysicsType == EPhysicsType::PhysType_Kinematic &&
			SharedData->EditorOptions->bHideKinematicBodies) ||
			(UsePhysicsAsset->SkeletalBodySetups[i]->PhysicsType == EPhysicsType::PhysType_Simulated &&
			SharedData->EditorOptions->bHideSimulatedBodies)
			)
		{
			continue;
		}
		if (SharedData->HiddenBodies.Contains(i))
		{
			continue;
		}
		int32 BoneIndex = GetBoneIndex(UsePhysicsAsset->SkeletalBodySetups[i]->BoneName);

		// If we found a bone for it, draw the collision.
		// The logic is as follows; always render in the ViewMode requested when not in hit mode - but if we are in hit mode and the right editing mode, render as solid
		if (BoneIndex != INDEX_NONE)
		{
			FTransform BoneTM = GetBoneTransform(BoneIndex);
			float Scale = BoneTM.GetScale3D().GetAbsMax();
			FVector VectorScale(Scale);
			BoneTM.RemoveScaling();

			FKAggregateGeom* AggGeom = &UsePhysicsAsset->SkeletalBodySetups[i]->AggGeom;

			for (int32 j = 0; j <AggGeom->SphereElems.Num(); ++j)
			{
				FTransform ElemTM = GetPrimitiveTransform(BoneTM, i, EAggCollisionShape::Sphere, j, Scale);

				//solids are drawn if it's the ViewMode and we're not doing a hit, or if it's hitAndBodyMode
				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid)
				{
					UMaterialInterface*	PrimMaterial = GetPrimitiveMaterial(i, EAggCollisionShape::Sphere, j);
					AggGeom->SphereElems[j].DrawElemSolid(PDI, ElemTM, VectorScale, PrimMaterial->GetRenderProxy());
				}

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid || CollisionViewMode == EPhysicsAssetEditorRenderMode::Wireframe)
				{
					AggGeom->SphereElems[j].DrawElemWire(PDI, ElemTM, VectorScale, GetPrimitiveColor(i, EAggCollisionShape::Sphere, j));
				}
			}

			for (int32 j = 0; j <AggGeom->BoxElems.Num(); ++j)
			{
				FTransform ElemTM = GetPrimitiveTransform(BoneTM, i, EAggCollisionShape::Box, j, Scale);

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid)
				{
					UMaterialInterface*	PrimMaterial = GetPrimitiveMaterial(i, EAggCollisionShape::Box, j);
					AggGeom->BoxElems[j].DrawElemSolid(PDI, ElemTM, VectorScale, PrimMaterial->GetRenderProxy());
				}

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid || CollisionViewMode == EPhysicsAssetEditorRenderMode::Wireframe)
				{
					AggGeom->BoxElems[j].DrawElemWire(PDI, ElemTM, VectorScale, GetPrimitiveColor(i, EAggCollisionShape::Box, j));
				}
			}

			for (int32 j = 0; j <AggGeom->SphylElems.Num(); ++j)
			{
				FTransform ElemTM = GetPrimitiveTransform(BoneTM, i, EAggCollisionShape::Sphyl, j, Scale);

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid)
				{
					UMaterialInterface*	PrimMaterial = GetPrimitiveMaterial(i, EAggCollisionShape::Sphyl, j);
					AggGeom->SphylElems[j].DrawElemSolid(PDI, ElemTM, VectorScale, PrimMaterial->GetRenderProxy());
				}

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid || CollisionViewMode == EPhysicsAssetEditorRenderMode::Wireframe)
				{
					AggGeom->SphylElems[j].DrawElemWire(PDI, ElemTM, VectorScale, GetPrimitiveColor(i, EAggCollisionShape::Sphyl, j));
				}
			}

			for (int32 j = 0; j <AggGeom->ConvexElems.Num(); ++j)
			{
				FTransform ElemTM = GetPrimitiveTransform(BoneTM, i, EAggCollisionShape::Convex, j, Scale);

				//convex doesn't have solid draw so render lines if we're in hitTestAndBodyMode
				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid || CollisionViewMode == EPhysicsAssetEditorRenderMode::Wireframe)
				{
					AggGeom->ConvexElems[j].DrawElemWire(PDI, ElemTM, Scale, GetPrimitiveColor(i, EAggCollisionShape::Convex, j));
				}
			}

			for (int32 j = 0; j <AggGeom->TaperedCapsuleElems.Num(); ++j)
			{
				FTransform ElemTM = GetPrimitiveTransform(BoneTM, i, EAggCollisionShape::TaperedCapsule, j, Scale);

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid)
				{
					UMaterialInterface*	PrimMaterial = GetPrimitiveMaterial(i, EAggCollisionShape::TaperedCapsule, j);
					AggGeom->TaperedCapsuleElems[j].DrawElemSolid(PDI, ElemTM, VectorScale, PrimMaterial->GetRenderProxy());
				}

				if (CollisionViewMode == EPhysicsAssetEditorRenderMode::Solid || CollisionViewMode == EPhysicsAssetEditorRenderMode::Wireframe)
				{
					AggGeom->TaperedCapsuleElems[j].DrawElemWire(PDI, ElemTM, VectorScale, GetPrimitiveColor(i, EAggCollisionShape::TaperedCapsule, j));
				}
			}
		}
	}
}

