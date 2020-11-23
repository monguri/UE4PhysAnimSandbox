#include "PhysAnimSandboxFunctionLibrary.h"
#include "Engine/SkeletalMesh.h"
#include "Rendering/SkeletalMeshLODImporterData.h"
#include "Rendering/SkeletalMeshLODModel.h"
#include "Rendering/SkeletalMeshModel.h"
#include "SkeletalMeshBuilder.h"
#include "Animation/Skeleton.h"
#include "Factories/FbxSkeletalMeshImportData.h"

#if WITH_EDITOR // USkeletalMesh::Build()が#if WITH_EDITORでの定義なので
bool UPhysAnimSandboxFunctionLibrary::CreateSkeletalMesh()
{
	UPackage* Package = CreatePackage(nullptr, TEXT("/Game/NewSkeletalMesh"));
	if(!ensure(Package))
	{
		return false;
	}

	USkeletalMesh* SkeletalMesh = NewObject<USkeletalMesh>(Package, FName("NewSkeletalMesh"), EObjectFlags::RF_Public | EObjectFlags::RF_Standalone | EObjectFlags::RF_Transactional);
	if(!ensure(SkeletalMesh))
	{
		return false;
	}

	FSkeletalMeshImportData SkeletalMeshData;
	{
		SkeletalMeshData.Points.Emplace(-10.0f, 10.0f, 0.0f);
		SkeletalMeshData.Points.Emplace(10.0f, 10.0f, 0.0f);
		SkeletalMeshData.Points.Emplace(-10.0f, -10.0f, 0.0f);

		SkeletalMeshImportData::FVertex V0, V1, V2;
		V0.VertexIndex = 0;
		V0.UVs[0] = FVector2D(0.0f, 0.0f);
		V0.MatIndex = 0;
		V1.VertexIndex = 1;
		V1.UVs[0] = FVector2D(1.0f, 0.0f);
		V1.MatIndex = 0;
		V2.VertexIndex = 2;
		V2.UVs[0] = FVector2D(0.0f, 1.0f);
		V2.MatIndex = 0;

		SkeletalMeshData.Wedges.Add(V0);
		SkeletalMeshData.Wedges.Add(V1);
		SkeletalMeshData.Wedges.Add(V2);

		SkeletalMeshImportData::FTriangle T0;
		T0.WedgeIndex[0] = 0;
		T0.WedgeIndex[1] = 1;
		T0.WedgeIndex[2] = 2;
		T0.MatIndex = 0;
		T0.SmoothingGroups = 0;

		SkeletalMeshData.Faces.Add(T0);

		SkeletalMeshImportData::FJointPos J0;
		J0.Transform = FTransform::Identity;
		J0.Length = 0.0f; // TODO
		J0.XSize = 5.0f; // TODO
		J0.YSize = 5.0f; // TODO
		J0.ZSize = 5.0f; // TODO

		SkeletalMeshImportData::FBone B0;
		B0.Name = FString("Root");
		B0.Flags = 0x02; //TODO
		B0.NumChildren = 0;
		B0.ParentIndex = INDEX_NONE; //TODO
		B0.BonePos = J0;

		SkeletalMeshData.RefBonesBinary.Add(B0);

		SkeletalMeshImportData::FRawBoneInfluence I0, I1, I2;
		I0.Weight = 1.0f;
		I0.VertexIndex = 0;
		I0.BoneIndex = 0;
		I1.Weight = 1.0f;
		I1.VertexIndex = 1;
		I1.BoneIndex = 0;
		I2.Weight = 1.0f;
		I2.VertexIndex = 2;
		I2.BoneIndex = 0;

		SkeletalMeshData.Influences.Add(I0);
		SkeletalMeshData.Influences.Add(I1);
		SkeletalMeshData.Influences.Add(I2);

		SkeletalMeshData.PointToRawMap.AddUninitialized(SkeletalMeshData.Points.Num());
		for (int32 PointIdx = 0; PointIdx < SkeletalMeshData.Points.Num(); PointIdx++)
		{
			SkeletalMeshData.PointToRawMap[PointIdx] = PointIdx;
		}

		SkeletalMeshData.NumTexCoords = 0;
		SkeletalMeshData.MaxMaterialIndex = 1; // TODO
		SkeletalMeshData.bHasVertexColors = false;
		SkeletalMeshData.bHasNormals = false;
		SkeletalMeshData.bHasTangents = false;
		SkeletalMeshData.bUseT0AsRefPose = false; // こんなのあったんだな。クロスの初期化に使えそう
		SkeletalMeshData.bDiffPose = false; // こんなのあったんだな。クロスの初期化に使えそう
	}

	FBox BoundingBox(SkeletalMeshData.Points.GetData(), SkeletalMeshData.Points.Num());

	SkeletalMesh->PreEditChange(nullptr);
	SkeletalMesh->InvalidateDeriveDataCacheGUID();

	FSkeletalMeshModel *ImportedResource = SkeletalMesh->GetImportedModel();
	check(ImportedResource->LODModels.Num() == 0);
	ImportedResource->LODModels.Empty();
	ImportedResource->LODModels.Add(new FSkeletalMeshLODModel());
	const int32 ImportLODModelIndex = 0;
	FSkeletalMeshLODModel& LODModel = ImportedResource->LODModels[ImportLODModelIndex];

	ProcessImportMeshMaterials(SkeletalMesh->Materials, SkeletalMeshData);

	int32 SkeletalDepth = 1;
	const USkeleton* ExistingSkeleton = nullptr;
	if (!ProcessImportMeshSkeleton(ExistingSkeleton, SkeletalMesh->RefSkeleton, SkeletalDepth, SkeletalMeshData))
	{
		SkeletalMesh->ClearFlags(RF_Standalone);
		SkeletalMesh->Rename(NULL, GetTransientPackage(), REN_DontCreateRedirectors);
		return false;
	}

	ProcessImportMeshInfluences(SkeletalMeshData);

	SkeletalMesh->SaveLODImportedData(ImportLODModelIndex, SkeletalMeshData);
	SkeletalMesh->SetLODImportedDataVersions(ImportLODModelIndex, ESkeletalMeshGeoImportVersions::LatestVersion, ESkeletalMeshSkinningImportVersions::LatestVersion);

	SkeletalMesh->ResetLODInfo();
	FSkeletalMeshLODInfo& NewLODInfo = SkeletalMesh->AddLODInfo();
	NewLODInfo.ReductionSettings.NumOfTrianglesPercentage = 1.0f;
	NewLODInfo.ReductionSettings.NumOfVertPercentage = 1.0f;
	NewLODInfo.ReductionSettings.MaxDeviationPercentage = 0.0f;
	NewLODInfo.LODHysteresis = 0.02f;

	SkeletalMesh->SetImportedBounds(FBoxSphereBounds(BoundingBox));

	SkeletalMesh->bHasVertexColors = SkeletalMeshData.bHasVertexColors;
	SkeletalMesh->VertexColorGuid = FGuid();

	LODModel.NumTexCoords = FMath::Max<uint32>(1, SkeletalMeshData.NumTexCoords);

	FSkeletalMeshBuildSettings BuildOptions;
	BuildOptions.bBuildAdjacencyBuffer = true;
	BuildOptions.bRecomputeNormals = true;
	BuildOptions.bRecomputeTangents = true;
	BuildOptions.bUseMikkTSpace = true; //TODO
	BuildOptions.bComputeWeightedNormals = true; //TODO
	BuildOptions.bRemoveDegenerates = true; //TODO
	BuildOptions.ThresholdPosition = 0.0f;
	BuildOptions.ThresholdTangentNormal = 0.0f;
	BuildOptions.ThresholdUV = 0.0f;
	BuildOptions.MorphThresholdPosition = 0.0f;

	check(SkeletalMesh->GetLODInfo(ImportLODModelIndex) != nullptr);
	SkeletalMesh->GetLODInfo(ImportLODModelIndex)->BuildSettings = BuildOptions;
	// TODO
	bool bRegenDepLODs = false;
	bool Success = FSkeletalMeshBuilder().Build(SkeletalMesh, ImportLODModelIndex, bRegenDepLODs);
	if (!Success)
	{
		SkeletalMesh->MarkPendingKill();
		return false;
	}

	SkeletalMesh->CalculateInvRefMatrices();
	// PhysicsAssetを作らなくても、SkeletalMeshRenderDataを作らないと描画できないので必要
	SkeletalMesh->Build();

	// AssetImportData作成は省略

	UPackage* SkeltonPackage = CreatePackage(nullptr, TEXT("/Game/NewSkeleton"));
	if(!ensure(SkeltonPackage))
	{
		return false;
	}

	USkeleton* Skeleton = NewObject<USkeleton>(SkeltonPackage, FName("NewSkeleton"), EObjectFlags::RF_Public | EObjectFlags::RF_Standalone | EObjectFlags::RF_Transactional);
	if (Skeleton == nullptr)
	{
		return false;
	}

	if (!Skeleton->MergeAllBonesToBoneTree(SkeletalMesh))
	{
		return false;
	}

	// TODO:いるか？
	Skeleton->UpdateReferencePoseFromMesh(SkeletalMesh);

	SkeletalMesh->Skeleton = Skeleton;

	// PhysicsAsset作成は省略
	SkeletalMesh->MarkPackageDirty();

	SkeletalMesh->PhysicsAsset = nullptr;
	return true;
}
#endif // WITH_EDITOR

