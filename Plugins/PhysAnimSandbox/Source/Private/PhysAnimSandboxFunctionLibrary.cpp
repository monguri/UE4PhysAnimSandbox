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

#if 0
	// 2 bone 1 triangle
	{
		SkeletalMeshData.Points.Emplace(-10.0f + 50.0f, 10.0f, 0.0f);
		SkeletalMeshData.Points.Emplace(10.0f + 50.0f, 10.0f, 0.0f);
		SkeletalMeshData.Points.Emplace(-10.0f + 50.0f, -10.0f, 0.0f);

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

		SkeletalMeshImportData::FJointPos J0, J1;
		J0.Transform = FTransform::Identity;
		J0.Length = 0.0f; // TODO
		J0.XSize = 5.0f; // TODO
		J0.YSize = 5.0f; // TODO
		J0.ZSize = 5.0f; // TODO
		J1.Transform = FTransform(FVector(50.0f, 0.0f, 0.0f));
		J1.Length = 0.0f; // TODO
		J1.XSize = 5.0f; // TODO
		J1.YSize = 5.0f; // TODO
		J1.ZSize = 5.0f; // TODO

		SkeletalMeshImportData::FBone B0, B1;
		B0.Name = FString("Root");
		B0.Flags = 0x02; //TODO
		B0.NumChildren = 1;
		B0.ParentIndex = INDEX_NONE;
		B0.BonePos = J0;
		B1.Name = FString("Child");
		B1.Flags = 0x02; //TODO
		B1.NumChildren = 0;
		B1.ParentIndex = 0;
		B1.BonePos = J1;

		SkeletalMeshData.RefBonesBinary.Add(B0);
		SkeletalMeshData.RefBonesBinary.Add(B1);

		SkeletalMeshImportData::FRawBoneInfluence I0, I1, I2;
		I0.Weight = 1.0f;
		I0.VertexIndex = 0;
		I0.BoneIndex = 1;
		I1.Weight = 1.0f;
		I1.VertexIndex = 1;
		I1.BoneIndex = 1;
		I2.Weight = 1.0f;
		I2.VertexIndex = 2;
		I2.BoneIndex = 1;

		SkeletalMeshData.Influences.Add(I0);
		SkeletalMeshData.Influences.Add(I1);
		SkeletalMeshData.Influences.Add(I2);

		SkeletalMeshData.PointToRawMap.AddUninitialized(SkeletalMeshData.Points.Num());
		for (int32 PointIdx = 0; PointIdx < SkeletalMeshData.Points.Num(); PointIdx++)
		{
			SkeletalMeshData.PointToRawMap[PointIdx] = PointIdx;
		}

		SkeletalMeshData.NumTexCoords = 1;
		SkeletalMeshData.MaxMaterialIndex = 1; // TODO
		SkeletalMeshData.bHasVertexColors = false;
		SkeletalMeshData.bHasNormals = false;
		SkeletalMeshData.bHasTangents = false;
		SkeletalMeshData.bUseT0AsRefPose = false; // こんなのあったんだな。クロスの初期化に使えそう
		SkeletalMeshData.bDiffPose = false; // こんなのあったんだな。クロスの初期化に使えそう
	}
#else
	// 9 bone 8 box
	{
		SkeletalMeshData.Points.Emplace(-10.0f + 50.0f, 10.0f, 10.0f);
		SkeletalMeshData.Points.Emplace(10.0f + 50.0f, 10.0f, 10.0f);
		SkeletalMeshData.Points.Emplace(-10.0f + 50.0f, -10.0f, 10.0f);
		SkeletalMeshData.Points.Emplace(10.0f + 50.0f, -10.0f, 10.0f);

		SkeletalMeshData.Points.Emplace(-10.0f + 50.0f, 10.0f, -10.0f);
		SkeletalMeshData.Points.Emplace(10.0f + 50.0f, 10.0f, -10.0f);
		SkeletalMeshData.Points.Emplace(-10.0f + 50.0f, -10.0f, -10.0f);
		SkeletalMeshData.Points.Emplace(10.0f + 50.0f, -10.0f, -10.0f);

		const float OneThird = 1.0f / 3.0f;
		const float OneFourth = 0.25f;

		// Trigngle T0
		SkeletalMeshImportData::FVertex V00, V01, V02;
		V00.VertexIndex = 0;
		V00.UVs[0] = FVector2D(OneThird, 0.0f);
		V00.MatIndex = 0;
		V01.VertexIndex = 1;
		V01.UVs[0] = FVector2D(OneThird * 2.0f, 0.0f);
		V01.MatIndex = 0;
		V02.VertexIndex = 2;
		V02.UVs[0] = FVector2D(OneThird, OneFourth);
		V02.MatIndex = 0;

		SkeletalMeshData.Wedges.Add(V00);
		SkeletalMeshData.Wedges.Add(V01);
		SkeletalMeshData.Wedges.Add(V02);

		SkeletalMeshImportData::FTriangle T0;
		T0.WedgeIndex[0] = 0;
		T0.WedgeIndex[1] = 1;
		T0.WedgeIndex[2] = 2;
		T0.MatIndex = 0;
		T0.SmoothingGroups = 0;

		SkeletalMeshData.Faces.Add(T0);

		// Trigngle T1
		SkeletalMeshImportData::FVertex V10, V11, V12;
		V10.VertexIndex = 3;
		V10.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth);
		V10.MatIndex = 0;
		V11.VertexIndex = 2;
		V11.UVs[0] = FVector2D(OneThird, OneFourth);
		V11.MatIndex = 0;
		V12.VertexIndex = 1;
		V12.UVs[0] = FVector2D(OneThird * 2.0f, 0.0f);
		V12.MatIndex = 0;

		SkeletalMeshData.Wedges.Add(V10);
		SkeletalMeshData.Wedges.Add(V11);
		SkeletalMeshData.Wedges.Add(V12);

		SkeletalMeshImportData::FTriangle T1;
		T1.WedgeIndex[0] = 3;
		T1.WedgeIndex[1] = 4;
		T1.WedgeIndex[2] = 5;
		T1.MatIndex = 0;
		T1.SmoothingGroups = 0;

		SkeletalMeshData.Faces.Add(T1);

		// Trigngle T2
		SkeletalMeshImportData::FVertex V20, V21, V22;
		V20.VertexIndex = 2;
		V20.UVs[0] = FVector2D(OneThird, OneFourth);
		V20.MatIndex = 0;
		V21.VertexIndex = 3;
		V21.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth);
		V21.MatIndex = 0;
		V22.VertexIndex = 6;
		V22.UVs[0] = FVector2D(OneThird, OneFourth * 2.0f);
		V22.MatIndex = 0;

		SkeletalMeshData.Wedges.Add(V20);
		SkeletalMeshData.Wedges.Add(V21);
		SkeletalMeshData.Wedges.Add(V22);

		SkeletalMeshImportData::FTriangle T2;
		T2.WedgeIndex[0] = 6;
		T2.WedgeIndex[1] = 7;
		T2.WedgeIndex[2] = 8;
		T2.MatIndex = 0;
		T2.SmoothingGroups = 0;

		SkeletalMeshData.Faces.Add(T2);

		// Trigngle T3
		SkeletalMeshImportData::FVertex V30, V31, V32;
		V30.VertexIndex = 7;
		V30.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 2.0f);
		V30.MatIndex = 0;
		V31.VertexIndex = 6;
		V31.UVs[0] = FVector2D(OneThird, OneFourth * 2.0f);
		V31.MatIndex = 0;
		V32.VertexIndex = 3;
		V32.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth);
		V32.MatIndex = 0;

		SkeletalMeshData.Wedges.Add(V30);
		SkeletalMeshData.Wedges.Add(V31);
		SkeletalMeshData.Wedges.Add(V32);

		SkeletalMeshImportData::FTriangle T3;
		T3.WedgeIndex[0] = 9;
		T3.WedgeIndex[1] = 10;
		T3.WedgeIndex[2] = 11;
		T3.MatIndex = 0;
		T3.SmoothingGroups = 0;

		SkeletalMeshData.Faces.Add(T3);


		SkeletalMeshImportData::FJointPos J0, J1;
		J0.Transform = FTransform::Identity;
		J0.Length = 0.0f; // TODO
		J0.XSize = 5.0f; // TODO
		J0.YSize = 5.0f; // TODO
		J0.ZSize = 5.0f; // TODO
		J1.Transform = FTransform(FVector(50.0f, 0.0f, 0.0f));
		J1.Length = 0.0f; // TODO
		J1.XSize = 5.0f; // TODO
		J1.YSize = 5.0f; // TODO
		J1.ZSize = 5.0f; // TODO

		SkeletalMeshImportData::FBone B0, B1;
		B0.Name = FString("Root");
		B0.Flags = 0x02; //TODO
		B0.NumChildren = 1;
		B0.ParentIndex = INDEX_NONE;
		B0.BonePos = J0;
		B1.Name = FString("Child");
		B1.Flags = 0x02; //TODO
		B1.NumChildren = 0;
		B1.ParentIndex = 0;
		B1.BonePos = J1;

		SkeletalMeshData.RefBonesBinary.Add(B0);
		SkeletalMeshData.RefBonesBinary.Add(B1);

		SkeletalMeshImportData::FRawBoneInfluence I0, I1, I2, I3, I4, I5, I6, I7;
		I0.Weight = 1.0f;
		I0.VertexIndex = 0;
		I0.BoneIndex = 1;
		I1.Weight = 1.0f;
		I1.VertexIndex = 1;
		I1.BoneIndex = 1;
		I2.Weight = 1.0f;
		I2.VertexIndex = 2;
		I2.BoneIndex = 1;
		I3.Weight = 1.0f;
		I3.VertexIndex = 3;
		I3.BoneIndex = 1;
		I4.Weight = 1.0f;
		I4.VertexIndex = 4;
		I4.BoneIndex = 1;
		I5.Weight = 1.0f;
		I5.VertexIndex = 5;
		I5.BoneIndex = 1;
		I6.Weight = 1.0f;
		I6.VertexIndex = 6;
		I6.BoneIndex = 1;
		I7.Weight = 1.0f;
		I7.VertexIndex = 7;
		I7.BoneIndex = 1;

		SkeletalMeshData.Influences.Add(I0);
		SkeletalMeshData.Influences.Add(I1);
		SkeletalMeshData.Influences.Add(I2);
		SkeletalMeshData.Influences.Add(I3);
		SkeletalMeshData.Influences.Add(I4);
		SkeletalMeshData.Influences.Add(I5);
		SkeletalMeshData.Influences.Add(I6);
		SkeletalMeshData.Influences.Add(I7);

		SkeletalMeshData.PointToRawMap.AddUninitialized(SkeletalMeshData.Points.Num());
		for (int32 PointIdx = 0; PointIdx < SkeletalMeshData.Points.Num(); PointIdx++)
		{
			SkeletalMeshData.PointToRawMap[PointIdx] = PointIdx;
		}

		SkeletalMeshData.NumTexCoords = 1;
		SkeletalMeshData.MaxMaterialIndex = 1; // TODO
		SkeletalMeshData.bHasVertexColors = false;
		SkeletalMeshData.bHasNormals = false;
		SkeletalMeshData.bHasTangents = false;
		SkeletalMeshData.bUseT0AsRefPose = false; // こんなのあったんだな。クロスの初期化に使えそう
		SkeletalMeshData.bDiffPose = false; // こんなのあったんだな。クロスの初期化に使えそう
	}
#endif

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

