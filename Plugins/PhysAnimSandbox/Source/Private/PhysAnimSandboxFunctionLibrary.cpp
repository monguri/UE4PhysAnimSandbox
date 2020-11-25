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
#elif 1
	// 2 bone 1 sphere
	{
		const int32 DIVISION = 4; // 180度の分割数
		const int32 NumPoints = 1 + (DIVISION - 1) * 2 * DIVISION + 1;
		SkeletalMeshData.Points.Reserve(NumPoints);

		// 北極の頂点
		SkeletalMeshData.Points.Emplace(0.0f + 50.0f, 0.0f, 10.0f);

		// 行ループ
		for (int32 Row = 1; Row < DIVISION; ++Row)
		{
			float RowSin, RowCos = 0.0f;
			FMath::SinCos(&RowSin, &RowCos, Row * PI / DIVISION);

			// 列ループ
			for (int32 Column = 0; Column < 2 * DIVISION; ++Column)
			{
				float ColumnSin, ColumnCos = 0.0f;
				FMath::SinCos(&ColumnSin, &ColumnCos, Column * PI / DIVISION);
				SkeletalMeshData.Points.Emplace(10.0f * RowSin * ColumnCos + 50.0f, 10.0f * RowSin * ColumnSin, 10.0f * RowCos);
			}
		}

		// 南極の頂点
		SkeletalMeshData.Points.Emplace(0.0f + 50.0f, 0.0f, -10.0f);

		uint32 VertexIndex = 0;

		// 北極のTriangle
		for (int32 Column = 0; Column < 2 * DIVISION; ++Column)
		{
			SkeletalMeshImportData::FVertex V0, V1, V2;
			V0.VertexIndex = 0;
			V0.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * (Column + 0.5f), 0.0f);
			V0.MatIndex = 0;
			V1.VertexIndex = Column + 1;
			V1.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * Column, 1.0f / DIVISION);
			V1.MatIndex = 0;
			V2.VertexIndex = (Column + 1) % (2 * DIVISION) + 1;
			V2.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * (Column + 1), 1.0f / DIVISION);
			V2.MatIndex = 0;
			SkeletalMeshData.Wedges.Add(V0);
			SkeletalMeshData.Wedges.Add(V2); // 時計回りにするので
			SkeletalMeshData.Wedges.Add(V1); // 時計回りにするので

			SkeletalMeshImportData::FTriangle T;
			T.WedgeIndex[0] = VertexIndex;
			T.WedgeIndex[1] = VertexIndex + 1;
			T.WedgeIndex[2] = VertexIndex + 2;
			T.MatIndex = 0;
			T.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T);

			VertexIndex += 3;
		}

		// 北極と南極の間の行のQuad
		for (int32 Row = 1; Row < DIVISION - 1; ++Row)
		{
			for (int32 Column = 0; Column < 2 * DIVISION; ++Column)
			{
				SkeletalMeshImportData::FVertex V0, V1, V2, V3;
				V0.VertexIndex = (Row - 1) * 2 * DIVISION + Column + 1;
				V0.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * Column, 1.0f / DIVISION * Row);
				V0.MatIndex = 0;
				V1.VertexIndex = (Row - 1) * 2 * DIVISION + (Column + 1) % (2 * DIVISION) + 1;
				V1.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * (Column + 1), 1.0f / DIVISION * Row);
				V1.MatIndex = 0;
				V2.VertexIndex = Row * 2 * DIVISION + Column + 1;
				V2.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * Column, 1.0f / DIVISION * (Row + 1));
				V2.MatIndex = 0;
				V3.VertexIndex = Row * 2 * DIVISION + (Column + 1) % (2 * DIVISION) + 1;
				V3.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * (Column + 1), 1.0f / DIVISION * (Row + 1));
				V3.MatIndex = 0;

				SkeletalMeshData.Wedges.Add(V0);
				SkeletalMeshData.Wedges.Add(V1);
				SkeletalMeshData.Wedges.Add(V2);

				SkeletalMeshData.Wedges.Add(V3);
				SkeletalMeshData.Wedges.Add(V2);
				SkeletalMeshData.Wedges.Add(V1);

				SkeletalMeshImportData::FTriangle T1, T2;
				T1.WedgeIndex[0] = VertexIndex;
				T1.WedgeIndex[1] = VertexIndex + 1;
				T1.WedgeIndex[2] = VertexIndex + 2;
				T1.MatIndex = 0;
				T1.SmoothingGroups = 0;

				SkeletalMeshData.Faces.Add(T1);

				VertexIndex += 3;

				T2.WedgeIndex[0] = VertexIndex;
				T2.WedgeIndex[1] = VertexIndex + 1;
				T2.WedgeIndex[2] = VertexIndex + 2;
				T2.MatIndex = 0;
				T2.SmoothingGroups = 0;

				SkeletalMeshData.Faces.Add(T2);

				VertexIndex += 3;
			}
		}

		// 南極のTriangle
		for (int32 Column = 0; Column < 2 * DIVISION; ++Column)
		{
			SkeletalMeshImportData::FVertex V0, V1, V2;
			V0.VertexIndex = NumPoints - 1;
			V0.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * (Column + 0.5f), 1.0f);
			V0.MatIndex = 0;
			V1.VertexIndex = NumPoints - 1 - DIVISION * 2 + Column;
			V1.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * (Column + 1), 1.0f - 1.0f / DIVISION);
			V1.MatIndex = 0;
			V2.VertexIndex = NumPoints - 1 - DIVISION * 2 + (Column + 1) % (2 * DIVISION);
			V2.UVs[0] = FVector2D(1.0f / (2 * DIVISION) * Column, 1.0f - 1.0f / DIVISION);
			V2.MatIndex = 0;
			SkeletalMeshData.Wedges.Add(V0);
			SkeletalMeshData.Wedges.Add(V1);
			SkeletalMeshData.Wedges.Add(V2);

			SkeletalMeshImportData::FTriangle T;
			T.WedgeIndex[0] = VertexIndex;
			T.WedgeIndex[1] = VertexIndex + 1;
			T.WedgeIndex[2] = VertexIndex + 2;
			T.MatIndex = 0;
			T.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T);

			VertexIndex += 3;
		}

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

		SkeletalMeshData.Influences.AddUninitialized(NumPoints);
		for (int32 PointIdx = 0; PointIdx < NumPoints; PointIdx++)
		{
			SkeletalMeshImportData::FRawBoneInfluence I;
			I.Weight = 1.0f;
			I.VertexIndex = PointIdx;
			I.BoneIndex = 1;
			SkeletalMeshData.Influences[PointIdx] = I;
		}

		SkeletalMeshData.PointToRawMap.AddUninitialized(NumPoints);
		for (int32 PointIdx = 0; PointIdx < NumPoints; PointIdx++)
		{
			SkeletalMeshData.PointToRawMap[PointIdx] = PointIdx;
		}
	}
#else
	// 2 bone 1 box
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
		{
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
		}

		// Trigngle T1
		{
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
		}

		// Trigngle T2
		{
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
		}

		// Trigngle T3
		{
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
		}

		// Trigngle T4
		{
			SkeletalMeshImportData::FVertex V40, V41, V42;
			V40.VertexIndex = 6;
			V40.UVs[0] = FVector2D(OneThird, OneFourth * 2.0f);
			V40.MatIndex = 0;
			V41.VertexIndex = 7;
			V41.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 2.0f);
			V41.MatIndex = 0;
			V42.VertexIndex = 4;
			V42.UVs[0] = FVector2D(OneThird, OneFourth * 3.0f);
			V42.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V40);
			SkeletalMeshData.Wedges.Add(V41);
			SkeletalMeshData.Wedges.Add(V42);

			SkeletalMeshImportData::FTriangle T4;
			T4.WedgeIndex[0] = 12;
			T4.WedgeIndex[1] = 13;
			T4.WedgeIndex[2] = 14;
			T4.MatIndex = 0;
			T4.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T4);
		}

		// Trigngle T5
		{
			SkeletalMeshImportData::FVertex V50, V51, V52;
			V50.VertexIndex = 5;
			V50.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 3.0f);
			V50.MatIndex = 0;
			V51.VertexIndex = 4;
			V51.UVs[0] = FVector2D(OneThird, OneFourth * 3.0f);
			V51.MatIndex = 0;
			V52.VertexIndex = 7;
			V52.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 2.0f);
			V52.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V50);
			SkeletalMeshData.Wedges.Add(V51);
			SkeletalMeshData.Wedges.Add(V52);

			SkeletalMeshImportData::FTriangle T5;
			T5.WedgeIndex[0] = 15;
			T5.WedgeIndex[1] = 16;
			T5.WedgeIndex[2] = 17;
			T5.MatIndex = 0;
			T5.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T5);
		}

		// Trigngle T6
		{
			SkeletalMeshImportData::FVertex V60, V61, V62;
			V60.VertexIndex = 4;
			V60.UVs[0] = FVector2D(OneThird, OneFourth * 3.0f);
			V60.MatIndex = 0;
			V61.VertexIndex = 5;
			V61.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 3.0f);
			V61.MatIndex = 0;
			V62.VertexIndex = 0;
			V62.UVs[0] = FVector2D(OneThird, 1.0f);
			V62.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V60);
			SkeletalMeshData.Wedges.Add(V61);
			SkeletalMeshData.Wedges.Add(V62);

			SkeletalMeshImportData::FTriangle T6;
			T6.WedgeIndex[0] = 18;
			T6.WedgeIndex[1] = 19;
			T6.WedgeIndex[2] = 20;
			T6.MatIndex = 0;
			T6.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T6);
		}

		// Trigngle T7
		{
			SkeletalMeshImportData::FVertex V70, V71, V72;
			V70.VertexIndex = 1;
			V70.UVs[0] = FVector2D(OneThird * 2.0f, 1.0f);
			V70.MatIndex = 0;
			V71.VertexIndex = 0;
			V71.UVs[0] = FVector2D(OneThird, 1.0f);
			V71.MatIndex = 0;
			V72.VertexIndex = 5;
			V72.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 3.0f);
			V72.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V70);
			SkeletalMeshData.Wedges.Add(V71);
			SkeletalMeshData.Wedges.Add(V72);

			SkeletalMeshImportData::FTriangle T7;
			T7.WedgeIndex[0] = 21;
			T7.WedgeIndex[1] = 22;
			T7.WedgeIndex[2] = 23;
			T7.MatIndex = 0;
			T7.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T7);
		}

		// Trigngle T8
		{
			SkeletalMeshImportData::FVertex V80, V81, V82;
			V80.VertexIndex = 0;
			V80.UVs[0] = FVector2D(0.0f, OneFourth);
			V80.MatIndex = 0;
			V81.VertexIndex = 2;
			V81.UVs[0] = FVector2D(OneThird, OneFourth);
			V81.MatIndex = 0;
			V82.VertexIndex = 4;
			V82.UVs[0] = FVector2D(0.0f, OneFourth * 2.0f);
			V82.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V80);
			SkeletalMeshData.Wedges.Add(V81);
			SkeletalMeshData.Wedges.Add(V82);

			SkeletalMeshImportData::FTriangle T8;
			T8.WedgeIndex[0] = 24;
			T8.WedgeIndex[1] = 25;
			T8.WedgeIndex[2] = 26;
			T8.MatIndex = 0;
			T8.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T8);
		}

		// Trigngle T9
		{
			SkeletalMeshImportData::FVertex V90, V91, V92;
			V90.VertexIndex = 6;
			V90.UVs[0] = FVector2D(OneThird, OneFourth * 2.0f);
			V90.MatIndex = 0;
			V91.VertexIndex = 4;
			V91.UVs[0] = FVector2D(0.0f, OneFourth * 2.0f);
			V91.MatIndex = 0;
			V92.VertexIndex = 2;
			V92.UVs[0] = FVector2D(OneThird, OneFourth);
			V92.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V90);
			SkeletalMeshData.Wedges.Add(V91);
			SkeletalMeshData.Wedges.Add(V92);

			SkeletalMeshImportData::FTriangle T9;
			T9.WedgeIndex[0] = 27;
			T9.WedgeIndex[1] = 28;
			T9.WedgeIndex[2] = 29;
			T9.MatIndex = 0;
			T9.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T9);
		}

		// Trigngle T10
		{
			SkeletalMeshImportData::FVertex V100, V101, V102;
			V100.VertexIndex = 3;
			V100.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth);
			V100.MatIndex = 0;
			V101.VertexIndex = 1;
			V101.UVs[0] = FVector2D(OneThird * 3.0f, OneFourth);
			V101.MatIndex = 0;
			V102.VertexIndex = 7;
			V102.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 2.0f);
			V102.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V100);
			SkeletalMeshData.Wedges.Add(V101);
			SkeletalMeshData.Wedges.Add(V102);

			SkeletalMeshImportData::FTriangle T10;
			T10.WedgeIndex[0] = 30;
			T10.WedgeIndex[1] = 31;
			T10.WedgeIndex[2] = 32;
			T10.MatIndex = 0;
			T10.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T10);
		}

		// Trigngle T11
		{
			SkeletalMeshImportData::FVertex V110, V111, V112;
			V110.VertexIndex = 5;
			V110.UVs[0] = FVector2D(1.0f, OneFourth * 2.0f);
			V110.MatIndex = 0;
			V111.VertexIndex = 7;
			V111.UVs[0] = FVector2D(OneThird * 2.0f, OneFourth * 2.0f);
			V111.MatIndex = 0;
			V112.VertexIndex = 1;
			V112.UVs[0] = FVector2D(1.0f, OneFourth);
			V112.MatIndex = 0;

			SkeletalMeshData.Wedges.Add(V110);
			SkeletalMeshData.Wedges.Add(V111);
			SkeletalMeshData.Wedges.Add(V112);

			SkeletalMeshImportData::FTriangle T11;
			T11.WedgeIndex[0] = 33;
			T11.WedgeIndex[1] = 34;
			T11.WedgeIndex[2] = 35;
			T11.MatIndex = 0;
			T11.SmoothingGroups = 0;

			SkeletalMeshData.Faces.Add(T11);
		}

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

