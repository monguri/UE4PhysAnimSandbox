#include "PhysicsEngine/RigidBodiesCustomMesh.h"
#include "UObject/ConstructorHelpers.h"
#include "CustomMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "StaticMeshResources.h"

ARigidBodiesCustomMesh::ARigidBodiesCustomMesh()
{
	PrimaryActorTick.bCanEverTick = true;

	DrawMesh = CreateDefaultSubobject<UCustomMeshComponent>(TEXT("CustomMeshComponent0"));
	RootComponent = DrawMesh;
}

namespace
{
	FVector RandPointInSphereCustomMesh(const FBoxSphereBounds& BoxSphere, const FVector& CenterPos)
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

	float CalculateMass(ERigdBodyGeometry Geometry, const FVector& HalfExtent, float Height, float Density)
	{
		const FVector& Extent = HalfExtent * 2.0f;
		switch (Geometry)
		{
		case ERigdBodyGeometry::Box:
			return Extent.X * Extent.Y * Extent.Z * Density;
		case ERigdBodyGeometry::Ellipsoid:
			return 4.0f / 3.0f * PI * HalfExtent.X * HalfExtent.Y * HalfExtent.Z * Density;
		case ERigdBodyGeometry::Capsule:
			return PI * Density * HalfExtent.X * HalfExtent.Y * (4.0f / 3.0f * HalfExtent.Z + Height);
		case ERigdBodyGeometry::Cylinder:
			return PI * HalfExtent.X * HalfExtent.Y * Extent.Z * Density;
		case ERigdBodyGeometry::Tetrahedron:
			return HalfExtent.X * HalfExtent.Y * HalfExtent.Z / 6.0f * Density;
		default:
			check(false);
			return 1.0f;
		}
	}

	FMatrix CalculateInertia(ERigdBodyGeometry Geometry, float Mass, float Density, const FVector& HalfExtent, float Height)
	{
		const FVector& Extent = HalfExtent * 2.0f;
		FMatrix Ret = FMatrix::Identity;

		switch (Geometry)
		{
		case ERigdBodyGeometry::Box:
			Ret.M[0][0] = Mass * (Extent.Y * Extent.Y + Extent.Z * Extent.Z) / 12.0f;
			Ret.M[1][1] = Mass * (Extent.Z * Extent.Z + Extent.X * Extent.X) / 12.0f;
			Ret.M[2][2] = Mass * (Extent.X * Extent.X + Extent.Y * Extent.Y) / 12.0f;
			break;
		case ERigdBodyGeometry::Ellipsoid:
			Ret.M[0][0] = Mass * (HalfExtent.Y * HalfExtent.Y + HalfExtent.Z * HalfExtent.Z) * 0.2f;
			Ret.M[1][1] = Mass * (HalfExtent.Z * HalfExtent.Z + HalfExtent.X * HalfExtent.X) * 0.2f;
			Ret.M[2][2] = Mass * (HalfExtent.X * HalfExtent.X + HalfExtent.Y * HalfExtent.Y) * 0.2f;
			break;
		case ERigdBodyGeometry::Capsule:
			Ret.M[0][0] = Density * PI * HalfExtent.X * HalfExtent.Y / 60.0f * (16.0f * HalfExtent.Z * HalfExtent.Z * HalfExtent.Z + 30.0f * HalfExtent.Z * HalfExtent.Z * Height + 20.0f * HalfExtent.Z * Height * Height + 16.0f * HalfExtent.Z * HalfExtent.Y * HalfExtent.Y + 15.0f * HalfExtent.Y * HalfExtent.Y * Height + 5.0f * Height * Height * Height);
			Ret.M[1][1] = Density * PI * HalfExtent.X * HalfExtent.Y / 60.0f * (16.0f * HalfExtent.Z * HalfExtent.Z * HalfExtent.Z + 30.0f * HalfExtent.Z * HalfExtent.Z * Height + 20.0f * HalfExtent.Z * Height * Height + 16.0f * HalfExtent.Z * HalfExtent.X * HalfExtent.X + 15.0f * HalfExtent.X * HalfExtent.X * Height + 5.0f * Height * Height * Height);
			Ret.M[2][2] = Density * PI * HalfExtent.X * HalfExtent.Y / 60.0f * (HalfExtent.X * HalfExtent.X + HalfExtent.Y * HalfExtent.Y) * (15.0f * Height + 16.0f * HalfExtent.Z);
			break;
		case ERigdBodyGeometry::Cylinder:
			Ret.M[0][0] = Mass * (HalfExtent.Y * HalfExtent.Y + Extent.Z * Extent.Z / 3.0f) * 0.25f;
			Ret.M[1][1] = Mass * (HalfExtent.X * HalfExtent.X + Extent.Z * Extent.Z / 3.0f) * 0.25f;
			Ret.M[2][2] = Mass * (HalfExtent.X * HalfExtent.X + HalfExtent.Y * HalfExtent.Y) * 0.25f;
			break;
		case ERigdBodyGeometry::Tetrahedron:
		{
			Ret.M[0][0] = Mass * (HalfExtent.Y * HalfExtent.Y + HalfExtent.Z * HalfExtent.Z) * 3.0f / 80.0f;
			Ret.M[1][1] = Mass * (HalfExtent.Z * HalfExtent.Z + HalfExtent.X * HalfExtent.X) * 3.0f / 80.0f;
			Ret.M[2][2] = Mass * (HalfExtent.X * HalfExtent.X + HalfExtent.Y * HalfExtent.Y) * 3.0f / 80.0f;
			Ret.M[0][1] = Ret.M[1][0] = Mass * (HalfExtent.X * HalfExtent.Y) / 80.0f;
			Ret.M[1][2] = Ret.M[2][1] = Mass * (HalfExtent.Y * HalfExtent.Z) / 80.0f;
			Ret.M[2][0] = Ret.M[0][2] = Mass * (HalfExtent.Z * HalfExtent.X) / 80.0f;
		}
			break;
		default:
			check(false);
			break;
		}

		return Ret;
	}

	void CreateStaticMeshVerticesIndices(UStaticMesh* StaticMesh, TArray<FVector>& Vertices, TArray<FIntVector>& Indices)
	{
		// TODO:とりあえずチェック
		check(StaticMesh != nullptr);

		const FStaticMeshLODResources& LOD0Resource = StaticMesh->GetLODForExport(0);
		const FPositionVertexBuffer& PosVB = LOD0Resource.VertexBuffers.PositionVertexBuffer;
	}

	void CreateConvexCollisionShape(ERigdBodyGeometry Geometry, const FVector& Scale, float Height, UStaticMesh* StaticMesh, ARigidBodiesCustomMesh::FCollisionShape& CollisionShape)
	{
		// HalfExtentやRadiusが1なのは、ScaleをHalfExtentとして計算している場所があるので必須
		static const TArray<FVector> BoxVertices = 
		{
			FVector(-1.0f, -1.0f, -1.0f), // 0
			FVector(+1.0f, -1.0f, -1.0f), // 1
			FVector(-1.0f, +1.0f, -1.0f), // 2
			FVector(+1.0f, +1.0f, -1.0f), // 3
			FVector(-1.0f, -1.0f, +1.0f), // 4
			FVector(+1.0f, -1.0f, +1.0f), // 5
			FVector(-1.0f, +1.0f, +1.0f), // 6
			FVector(+1.0f, +1.0f, +1.0f), // 7
		};

		static const TArray<FIntVector> BoxIndices = 
		{
			FIntVector(0, 1, 2),
			FIntVector(1, 3, 2),
			FIntVector(4, 7, 5),
			FIntVector(4, 6, 7),
			FIntVector(2, 3, 6),
			FIntVector(3, 7, 6),
			FIntVector(1, 0, 5),
			FIntVector(0, 4, 5),
			FIntVector(0, 2, 4),
			FIntVector(2, 6, 4),
			FIntVector(3, 1, 7),
			FIntVector(1, 5, 7)
		};

		float CosPIover4 = FMath::Cos(PI * 0.25f);
		float SinPIover4 = FMath::Cos(PI * 0.25f);
		static const TArray<FVector> EllipsoidVertices =
		{
			FVector(0.0f, 0.0f, 1.0f), // 0

			FVector(SinPIover4 * 1.0f, SinPIover4 * 0.0f, CosPIover4), // 1
			FVector(SinPIover4 * CosPIover4, SinPIover4 * -SinPIover4, CosPIover4), // 2
			FVector(SinPIover4 * 0.0f, SinPIover4 * -1.0f, CosPIover4), // 3
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * -SinPIover4, CosPIover4), // 4
			FVector(SinPIover4 * -1.0f, SinPIover4 * 0.0f, CosPIover4), // 5
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * SinPIover4, CosPIover4), // 6
			FVector(SinPIover4 * 0.0f, SinPIover4 * 1.0f, CosPIover4), // 7
			FVector(SinPIover4 * CosPIover4, SinPIover4 * SinPIover4, CosPIover4), // 8

			FVector(1.0f, 0.0f, 0.0f), // 9
			FVector(CosPIover4, -SinPIover4, 0.0f), // 10
			FVector(0.0f, -1.0f, 0.0f), // 11
			FVector(-CosPIover4, -SinPIover4, 0.0f), // 12
			FVector(-1.0f, 0.0f, 0.0f), // 13
			FVector(-CosPIover4, SinPIover4, 0.0f), // 14
			FVector(0.0f, 1.0f, 0.0f), // 15
			FVector(CosPIover4, SinPIover4, 0.0f), // 16

			FVector(SinPIover4 * 1.0f, SinPIover4 * 0.0f, -CosPIover4), // 17
			FVector(SinPIover4 * CosPIover4, SinPIover4 * -SinPIover4, -CosPIover4), // 18
			FVector(SinPIover4 * 0.0f, SinPIover4 * -1.0f, -CosPIover4), // 19
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * -SinPIover4, -CosPIover4), // 20
			FVector(SinPIover4 * -1.0f, SinPIover4 * 0.0f, -CosPIover4), // 21
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * SinPIover4, -CosPIover4), // 22
			FVector(SinPIover4 * 0.0f, SinPIover4 * 1.0f, -CosPIover4), // 23
			FVector(SinPIover4 * CosPIover4, SinPIover4 * SinPIover4, -CosPIover4), // 24

			FVector(0.0f, 0.0f, -1.0f), // 25
		};

		static const TArray<FIntVector> EllipsoidIndices = 
		{
			FIntVector(0, 1, 2),
			FIntVector(0, 2, 3),
			FIntVector(0, 3, 4),
			FIntVector(0, 4, 5),
			FIntVector(0, 5, 6),
			FIntVector(0, 6, 7),
			FIntVector(0, 7, 8),
			FIntVector(0, 8, 1),

			FIntVector(1, 9, 2),
			FIntVector(2, 9, 10),
			FIntVector(2, 10, 3),
			FIntVector(3, 10, 11),
			FIntVector(3, 11, 4),
			FIntVector(4, 11, 12),
			FIntVector(4, 12, 5),
			FIntVector(5, 12, 13),
			FIntVector(5, 13, 6),
			FIntVector(6, 13, 14),
			FIntVector(6, 14, 7),
			FIntVector(7, 14, 15),
			FIntVector(7, 15, 8),
			FIntVector(8, 15, 16),
			FIntVector(8, 16, 1),
			FIntVector(1, 16, 9),

			FIntVector(9, 17, 10),
			FIntVector(10, 17, 18),
			FIntVector(10, 18, 11),
			FIntVector(11, 18, 19),
			FIntVector(11, 19, 12),
			FIntVector(12, 19, 20),
			FIntVector(12, 20, 13),
			FIntVector(13, 20, 21),
			FIntVector(13, 21, 14),
			FIntVector(14, 21, 22),
			FIntVector(14, 22, 15),
			FIntVector(15, 22, 23),
			FIntVector(15, 23, 16),
			FIntVector(16, 23, 24),
			FIntVector(16, 24, 9),
			FIntVector(9, 24, 17),

			FIntVector(17, 25, 18),
			FIntVector(18, 25, 19),
			FIntVector(19, 25, 20),
			FIntVector(20, 25, 21),
			FIntVector(21, 25, 22),
			FIntVector(22, 25, 23),
			FIntVector(23, 25, 24),
			FIntVector(24, 25, 17),
		};

		static const TArray<FVector> CylinderVertices =
		{
			FVector(0.0f, 0.0f, 1.0f), // 0

			FVector(1.0f, 0.0f, 1.0f), // 1
			FVector(CosPIover4, -SinPIover4, 1.0f), // 2
			FVector(0.0f, -1.0f, 1.0f), // 3
			FVector(-CosPIover4, -SinPIover4, 1.0f), // 4
			FVector(-1.0f, 0.0f, 1.0f), // 5
			FVector(-CosPIover4, SinPIover4, 1.0f), // 6
			FVector(0.0f, 1.0f, 1.0f), // 7
			FVector(CosPIover4, SinPIover4, 1.0f), // 8

			FVector(1.0f, 0.0f, -1.0f), // 9
			FVector(CosPIover4, -SinPIover4, -1.0f), // 10
			FVector(0.0f, -1.0f, -1.0f), // 11
			FVector(-CosPIover4, -SinPIover4, -1.0f), // 12
			FVector(-1.0f, 0.0f, -1.0f), // 13
			FVector(-CosPIover4, SinPIover4, -1.0f), // 14
			FVector(0.0f, 1.0f, -1.0f), // 15
			FVector(CosPIover4, SinPIover4, -1.0f), // 16

			FVector(0.0f, 0.0f, -1.0f), // 17
		};

		static const TArray<FIntVector> CylinderIndices = 
		{
			FIntVector(0, 1, 2),
			FIntVector(0, 2, 3),
			FIntVector(0, 3, 4),
			FIntVector(0, 4, 5),
			FIntVector(0, 5, 6),
			FIntVector(0, 6, 7),
			FIntVector(0, 7, 8),
			FIntVector(0, 8, 1),

			FIntVector(1, 9, 2),
			FIntVector(2, 9, 10),
			FIntVector(2, 10, 3),
			FIntVector(3, 10, 11),
			FIntVector(3, 11, 4),
			FIntVector(4, 11, 12),
			FIntVector(4, 12, 5),
			FIntVector(5, 12, 13),
			FIntVector(5, 13, 6),
			FIntVector(6, 13, 14),
			FIntVector(6, 14, 7),
			FIntVector(7, 14, 15),
			FIntVector(7, 15, 8),
			FIntVector(8, 15, 16),
			FIntVector(8, 16, 1),
			FIntVector(1, 16, 9),

			FIntVector(9, 17, 10),
			FIntVector(10, 17, 11),
			FIntVector(11, 17, 12),
			FIntVector(12, 17, 13),
			FIntVector(13, 17, 14),
			FIntVector(14, 17, 15),
			FIntVector(15, 17, 16),
			FIntVector(16, 17, 9),
		};

		static const TArray<FVector> CapsuleVertices =
		{
			// 上半球部分および円柱部分。+Height/2 + Radius_Zは後から行う
			FVector(0.0f, 0.0f, 1.0f), // 0

			FVector(SinPIover4 * 1.0f, SinPIover4 * 0.0f, CosPIover4), // 1
			FVector(SinPIover4 * CosPIover4, SinPIover4 * -SinPIover4, CosPIover4), // 2
			FVector(SinPIover4 * 0.0f, SinPIover4 * -1.0f, CosPIover4), // 3
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * -SinPIover4, CosPIover4), // 4
			FVector(SinPIover4 * -1.0f, SinPIover4 * 0.0f, CosPIover4), // 5
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * SinPIover4, CosPIover4), // 6
			FVector(SinPIover4 * 0.0f, SinPIover4 * 1.0f, CosPIover4), // 7
			FVector(SinPIover4 * CosPIover4, SinPIover4 * SinPIover4, CosPIover4), // 8

			FVector(1.0f, 0.0f, 0.0f), // 9
			FVector(CosPIover4, -SinPIover4, 0.0f), // 10
			FVector(0.0f, -1.0f, 0.0f), // 11
			FVector(-CosPIover4, -SinPIover4, 0.0f), // 12
			FVector(-1.0f, 0.0f, 0.0f), // 13
			FVector(-CosPIover4, SinPIover4, 0.0f), // 14
			FVector(0.0f, 1.0f, 0.0f), // 15
			FVector(CosPIover4, SinPIover4, 0.0f), // 16

			// 下半球部分および円柱部分。-Height/2 - Radius_Zは後から行う
			FVector(1.0f, 0.0f, 0.0f), // 17
			FVector(CosPIover4, -SinPIover4, 0.0f), // 18
			FVector(0.0f, -1.0f, 0.0f), // 19
			FVector(-CosPIover4, -SinPIover4, 0.0f), // 20
			FVector(-1.0f, 0.0f, 0.0f), // 21
			FVector(-CosPIover4, SinPIover4, 0.0f), // 22
			FVector(0.0f, 1.0f, 0.0f), // 23
			FVector(CosPIover4, SinPIover4, 0.0f), // 24

			FVector(SinPIover4 * 1.0f, SinPIover4 * 0.0f, -CosPIover4), // 25
			FVector(SinPIover4 * CosPIover4, SinPIover4 * -SinPIover4, -CosPIover4), // 26
			FVector(SinPIover4 * 0.0f, SinPIover4 * -1.0f, -CosPIover4), // 27
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * -SinPIover4, -CosPIover4), // 28
			FVector(SinPIover4 * -1.0f, SinPIover4 * 0.0f, -CosPIover4), // 29
			FVector(SinPIover4 * -CosPIover4, SinPIover4 * SinPIover4, -CosPIover4), // 30
			FVector(SinPIover4 * 0.0f, SinPIover4 * 1.0f, -CosPIover4), // 31
			FVector(SinPIover4 * CosPIover4, SinPIover4 * SinPIover4, -CosPIover4), // 32

			FVector(0.0f, 0.0f, -1.0f), // 33
		};

		static const TArray<FIntVector> CapsuleIndices = 
		{
			// 上半球部分
			FIntVector(0, 1, 2),
			FIntVector(0, 2, 3),
			FIntVector(0, 3, 4),
			FIntVector(0, 4, 5),
			FIntVector(0, 5, 6),
			FIntVector(0, 6, 7),
			FIntVector(0, 7, 8),
			FIntVector(0, 8, 1),

			FIntVector(1, 9, 2),
			FIntVector(2, 9, 10),
			FIntVector(2, 10, 3),
			FIntVector(3, 10, 11),
			FIntVector(3, 11, 4),
			FIntVector(4, 11, 12),
			FIntVector(4, 12, 5),
			FIntVector(5, 12, 13),
			FIntVector(5, 13, 6),
			FIntVector(6, 13, 14),
			FIntVector(6, 14, 7),
			FIntVector(7, 14, 15),
			FIntVector(7, 15, 8),
			FIntVector(8, 15, 16),
			FIntVector(8, 16, 1),
			FIntVector(1, 16, 9),

			// 円柱部分
			FIntVector(9, 17, 10),
			FIntVector(10, 17, 18),
			FIntVector(10, 18, 11),
			FIntVector(11, 18, 19),
			FIntVector(11, 19, 12),
			FIntVector(12, 19, 20),
			FIntVector(12, 20, 13),
			FIntVector(13, 20, 21),
			FIntVector(13, 21, 14),
			FIntVector(14, 21, 22),
			FIntVector(14, 22, 15),
			FIntVector(15, 22, 23),
			FIntVector(15, 23, 16),
			FIntVector(16, 23, 24),
			FIntVector(16, 24, 9),
			FIntVector(9, 24, 17),

			// 下半球部分
			FIntVector(17, 25, 18),
			FIntVector(18, 25, 26),
			FIntVector(18, 26, 19),
			FIntVector(19, 26, 27),
			FIntVector(19, 27, 20),
			FIntVector(20, 27, 28),
			FIntVector(20, 28, 21),
			FIntVector(21, 28, 29),
			FIntVector(21, 29, 22),
			FIntVector(22, 29, 30),
			FIntVector(22, 30, 23),
			FIntVector(23, 30, 31),
			FIntVector(23, 31, 24),
			FIntVector(24, 31, 32),
			FIntVector(24, 32, 17),
			FIntVector(17, 32, 25),

			FIntVector(25, 33, 26),
			FIntVector(26, 33, 27),
			FIntVector(27, 33, 28),
			FIntVector(28, 33, 29),
			FIntVector(29, 33, 30),
			FIntVector(30, 33, 31),
			FIntVector(31, 33, 32),
			FIntVector(32, 33, 25),
		};

		// 他のジオメトリはモデル座標の原点を重心にしているが、四面体は原点を頂点0とした方がわかりやすいので後から重心座標に変換する
		static const TArray<FVector> TetrahedronVertices = 
		{
			FVector(0.0f, 0.0f, 0.0f), // 0
			FVector(1.0f, 0.0f, 0.0f), // 1
			FVector(0.0f, 1.0f, 0.0f), // 2
			FVector(0.0f, 0.0f, 1.0f), // 3
		};

		static const TArray<FIntVector> TetrahedronIndices = 
		{
			FIntVector(0, 1, 2),
			FIntVector(0, 3, 1),
			FIntVector(0, 2, 3),
			FIntVector(1, 3, 2),
		};


		TArray<FIntVector> Indices;

		switch (Geometry)
		{
		case ERigdBodyGeometry::Box:
			CollisionShape.Vertices = BoxVertices;
			Indices = BoxIndices;
			break;
		case ERigdBodyGeometry::Ellipsoid:
			CollisionShape.Vertices = EllipsoidVertices;
			Indices = EllipsoidIndices;
			break;
		case ERigdBodyGeometry::Capsule:
			CollisionShape.Vertices = CapsuleVertices;
			Indices = CapsuleIndices;
			break;
		case ERigdBodyGeometry::Cylinder:
			CollisionShape.Vertices = CylinderVertices;
			Indices = CylinderIndices;
			break;
		case ERigdBodyGeometry::Tetrahedron:
			CollisionShape.Vertices = TetrahedronVertices;
			Indices = TetrahedronIndices;
			break;
		case ERigdBodyGeometry::StaticMesh:
			CreateStaticMeshVerticesIndices(StaticMesh, CollisionShape.Vertices, Indices);
			break;
		default:
			check(false);
			break;
		}

		CollisionShape.Edges.SetNum(CollisionShape.Vertices.Num() + Indices.Num() - 2); // オイラーの多面体定理　v - e + f = 2
		CollisionShape.Facets.SetNum(Indices.Num());

		// 四面体の場合は重心座標がローカル座標の原点になるようにここで調整する
		if (Geometry == ERigdBodyGeometry::Tetrahedron)
		{
			const FVector& CoM = FVector(0.25f);

			for (FVector& Vertex : CollisionShape.Vertices)
			{
				Vertex -= CoM;
			}
		}

		for (FVector& Vertex : CollisionShape.Vertices)
		{
			Vertex *= Scale;
		}

		// カプセルの場合はスケールだけでなくHeightの影響もプラス
		if (Geometry == ERigdBodyGeometry::Capsule)
		{
			for (int32 i = 0; i < CollisionShape.Vertices.Num(); i++)
			{
				FVector& Vertex = CollisionShape.Vertices[i];
				if (i < 17)
				{
					Vertex += FVector(0.0f, 0.0f, Height * 0.5f);
				}
				else
				{
					Vertex -= FVector(0.0f, 0.0f, Height * 0.5f);
				}
			}
		}

		for (int32 i = 0; i < CollisionShape.Facets.Num(); i++)
		{
			ARigidBodiesCustomMesh::FFacet& Facet = CollisionShape.Facets[i];
			Facet.VertId[0] = Indices[i].X;
			Facet.VertId[1] = Indices[i].Y;
			Facet.VertId[2] = Indices[i].Z;
		}

		// 2頂点間に必ずエッジがあるわけではないので単純に8頂点の組み合わせ数ではない。エッジは18本である。組み合わせ数だけ多めにとっておく
		check(CollisionShape.Vertices.Num() < 255);
		TArray<uint8> EdgeIdTable; // とりあえずuint8なので255エッジまで。FFは識別値に使う
		EdgeIdTable.SetNum(CollisionShape.Vertices.Num() * (CollisionShape.Vertices.Num() - 1) / 2); // n(n-1)/2 n = 8
		for (uint8& EdgeId : EdgeIdTable)
		{
			EdgeId = 0xff;
		}

		// NormalとEdgeId[3]の計算。EdgesとFacetsの作成。
		int32 EdgeIdx = 0;
		for (int32 FacetId = 0; FacetId < CollisionShape.Facets.Num(); FacetId++)
		{
			ARigidBodiesCustomMesh::FFacet& Facet = CollisionShape.Facets[FacetId];

			const FVector& P0 = CollisionShape.Vertices[Facet.VertId[0]];
			const FVector& P1 = CollisionShape.Vertices[Facet.VertId[1]];
			const FVector& P2 = CollisionShape.Vertices[Facet.VertId[2]];
			// 左手系。三角形の縮退は考慮してない。
			Facet.Normal = -FVector::CrossProduct(P1 - P0, P2 - P0).GetUnsafeNormal();

			for (int32 TriVert = 0; TriVert < 3; TriVert++)
			{
				int32 VertId0 = FMath::Min(Facet.VertId[TriVert % 3], Facet.VertId[(TriVert + 1) % 3]);
				int32 VertId1 = FMath::Max(Facet.VertId[TriVert % 3], Facet.VertId[(TriVert + 1) % 3]);
				int32 TableId = VertId1 * (VertId1 - 1) / 2 + VertId0;
				if (EdgeIdTable[TableId] == 0xff)
				{
					// 初回登録
					CollisionShape.Edges[EdgeIdx].VertId[0] = VertId0;
					CollisionShape.Edges[EdgeIdx].VertId[1] = VertId1;
					CollisionShape.Edges[EdgeIdx].FacetId[0] = FacetId;
					CollisionShape.Edges[EdgeIdx].FacetId[1] = FacetId; // 初回登録時は両方同じFacetIdに。別のfacetで同じエッジに出会ったときに更新する
					Facet.EdgeId[TriVert] = EdgeIdx;

					EdgeIdTable[TableId] = EdgeIdx;
					EdgeIdx++;
				}
				else
				{
					CollisionShape.Edges[EdgeIdTable[TableId]].FacetId[1] = FacetId;
					Facet.EdgeId[TriVert] = EdgeIdTable[TableId];
				}
			}
		}
	}
}

void ARigidBodiesCustomMesh::BeginPlay()
{
	Super::BeginPlay();

	if (!bDirectSet)
	{
		// InitPosRadius半径の球内にランダムに配置
		FBoxSphereBounds BoxSphere(InitPosCenter, FVector(InitPosRadius), InitPosRadius);

		for (int32 i = 0; i < NumRigidBodies; i++)
		{
			FRigidBodySetting Setting;
			if (bRandom)
			{
				Setting.Geometry = static_cast<ERigdBodyGeometry>(i % 5);
			}
			else
			{
				Setting.Geometry = Geometry;
			}
			Setting.Friction = Friction;
			Setting.Restitution = Restitution;
			// TODO:bDirectSet=falseではとりあえずBox
			Setting.Mass = CalculateMass(Geometry, HalfExtent, Height, Density);
			Setting.Location = RandPointInSphereCustomMesh(BoxSphere, InitPosCenter);
			Setting.Rotation = Rotation;
			Setting.HalfExtent = HalfExtent;
			Setting.Density = Density;
			Setting.Height = Height;
			RigidBodySettings.Add(Setting);
		}
	}

	NumRigidBodies = RigidBodySettings.Num();
	NumThreadRBs = (NumRigidBodies + 2 + NumThreads - 1) / NumThreads; // +2は弾とフロアの分

	RigidBodies.SetNum(NumRigidBodies + 2); // +2は弾とフロアの分

	// RigidBodiesは0番目は弾で1番目はフロアに。2番目以降が各剛体。
	FRigidBody& AmmoRigidBody = RigidBodies[0];
	CreateConvexCollisionShape(ERigdBodyGeometry::Ellipsoid, FVector(50.0f), 0.0f, nullptr, AmmoRigidBody.CollisionShape);
	AmmoRigidBody.MotionType = ERigdBodyMotionType::Static;
	AmmoRigidBody.Mass = 1.0f; // 弾はStaticなので無限質量扱いにしてるので使っていない
	AmmoRigidBody.Inertia = CalculateInertia(ERigdBodyGeometry::Ellipsoid, AmmoRigidBody.Mass, 0.01f, FVector(50.0f), 50.0f);
	AmmoRigidBody.Position = FVector(5000.0f, 5000.0f, 5000.0f);

	FRigidBody& FloorRigidBody = RigidBodies[1];
	CreateConvexCollisionShape(ERigdBodyGeometry::Box, FloorScale, 0.0f, nullptr, FloorRigidBody.CollisionShape);
	FloorRigidBody.MotionType = ERigdBodyMotionType::Static;
	FloorRigidBody.Mass = 0.0f; // フロアはStaticなので無限質量扱いにしてるので使っていない
	FloorRigidBody.Inertia = FMatrix::Identity; // フロアはStaticなので無限質量扱いにしてるので使っていない
	FloorRigidBody.Friction = FloorFriction;
	FloorRigidBody.Restitution = FloorRestitution;
	FloorRigidBody.Position = FloorLocation;
	FloorRigidBody.Orientation = FloorRotation.Quaternion();
	// TODO:とりあえずその他の物理パラメータは初期値のまま

	for (int32 i = 2; i < NumRigidBodies + 2; i++)
	{
		const FRigidBodySetting& Setting = RigidBodySettings[i - 2];

		FRigidBody& RigidBody = RigidBodies[i];
		CreateConvexCollisionShape(Setting.Geometry, Setting.HalfExtent, Setting.Height, Setting.StaticMesh, RigidBody.CollisionShape);

		RigidBody.MotionType = Setting.MotionType;
		RigidBody.Mass = Setting.Mass;
		RigidBody.Inertia = CalculateInertia(Setting.Geometry, RigidBody.Mass, Setting.Density, Setting.HalfExtent, Setting.Height) * Setting.InertiaScale;
		RigidBody.Friction = Setting.Friction;
		RigidBody.Restitution = Setting.Restitution;
		RigidBody.Position = Setting.Location;
		RigidBody.Orientation = Setting.Rotation.Quaternion();
		RigidBody.LinearVelocity = Setting.LinearVelocity;
		RigidBody.AngularVelocity = Setting.AngularVelocity;
		// TODO:とりあえずその他の物理パラメータは初期値のまま
	}

	Joints.SetNum(JointSettings.Num());
	for (int32 i = 0; i < JointSettings.Num(); i++)
	{
		Joints[i].RigidBodyA_Idx = JointSettings[i].RigidBodyA_Idx + 2; // +2は弾とフロアの分
		Joints[i].RigidBodyB_Idx = JointSettings[i].RigidBodyB_Idx + 2; // +2は弾とフロアの分
		Joints[i].Bias = JointSettings[i].Bias;
		Joints[i].AnchorA = JointSettings[i].AnchorA;
		Joints[i].AnchorB = JointSettings[i].AnchorB;
	}

	ContactPairs.SetNum(((NumRigidBodies + 2) * (NumRigidBodies + 1)) / 2); //TODO: コンタクトペアは最大でも総当たりペア数。最終的には大きすぎるがとりあえずこれで。

	int32 ContactPairIdx = 0;
	for (int32 i = 0; i < NumRigidBodies + 2; i++)
	{
		for (int32 j = i + 1; j < NumRigidBodies + 2; j++)
		{
			ContactPairs[ContactPairIdx].RigidBodyA_Idx = i;
			ContactPairs[ContactPairIdx].RigidBodyB_Idx = j;
			ContactPairIdx++;
		}
	}

	SolverBodies.SetNum(NumRigidBodies + 2);
}

void ARigidBodiesCustomMesh::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSecondsの値の変動に関わらず、シミュレーションに使うサブステップタイムは固定とする
		float SubStepDeltaSeconds = 1.0f / FrameRate;
		Simulate(SubStepDeltaSeconds);
	}

	ApplyRigidBodiesToMeshDrawing();
}

void ARigidBodiesCustomMesh::Simulate(float DeltaSeconds)
{
	//TODO: コンタクトペア配列にマルチスレッドからアクセスするのが危険なのでとりあえずシングルスレッド
	DetectCollision();

	//TODO: 同じ剛体にマルチスレッドからアクセスするのが危険なのでとりあえずシングルスレッド
	SolveConstraint(DeltaSeconds);

	ParallelFor(NumThreads,
		[this, DeltaSeconds](int32 ThreadIndex)
		{
			for (int32 RBIdx = NumThreadRBs * ThreadIndex; RBIdx < NumThreadRBs * (ThreadIndex + 1) && RBIdx < NumRigidBodies + 2; ++RBIdx)
			{
				Integrate(RBIdx, DeltaSeconds);
			}
		}
	);
}

namespace
{
	void GetConvexProjectedRange(const ARigidBodiesCustomMesh::FCollisionShape& CollisionShape, const FVector& Axis, float& OutMin, float& OutMax)
	{
		float Min = FLT_MAX;
		float Max = -FLT_MAX;

		for (const FVector& Vertex : CollisionShape.Vertices)
		{
			float ProjectedVal = FVector::DotProduct(Axis, Vertex);
			Min = FMath::Min(Min, ProjectedVal);
			Max = FMath::Max(Max, ProjectedVal);
		}

		OutMin = Min;
		OutMax = Max;
	}

	enum SeparationAxisType : uint8
	{
		PointAFacetB,
		PointBFacetA,
		EdgeEdge,
	};

	bool CheckSeparationPlaneExistAndUpdateMinPenetration(float MinA, float MaxA, float MinB, float MaxB, const FVector& Axis, SeparationAxisType SAType, float& InOutDistanceMin, FVector& InOutAxisMin, SeparationAxisType& InOutSAType, bool& InOut_bAxisFlip)
	{
		float Dist1 = MinA - MaxB;
		float Dist2 = MinB - MaxA;

		if (Dist1 >= 0.0f || Dist2 >= 0.0f)
		{
			return true;
		}

		if (InOutDistanceMin < Dist1)
		{
			InOutDistanceMin = Dist1;
			InOutAxisMin = Axis;
			InOutSAType = SAType;
			InOut_bAxisFlip = false;
		}

		if (InOutDistanceMin < Dist2)
		{
			InOutDistanceMin = Dist2;
			InOutAxisMin = -Axis;
			InOutSAType = SAType;
			InOut_bAxisFlip = true;
		}

		return false;
	}

	bool DetectConvexConvexContact(const ARigidBodiesCustomMesh::FRigidBody& RigidBodyA, const ARigidBodiesCustomMesh::FRigidBody& RigidBodyB, FVector& OutNormal, float& OutPenetrationDepth, FVector& OutContactPointA, FVector& OutContactPointB)
	{
		// 最も浅い貫通深度とそのときの分離軸（判定軸と呼ぶ）。Aのローカル座標で扱い、Aを押し返すという考え方で扱う。
		float DistanceMin = -FLT_MAX;
		FVector AxisMin = FVector::ZeroVector;

		SeparationAxisType SAType = SeparationAxisType::EdgeEdge;
		bool bAxisFlip = false; // Aを押し返す方向が分離軸の方向と一致すればfalse。逆方向ならtrue。

		const FTransform& ALocalToWorld = FTransform(RigidBodyA.Orientation, RigidBodyA.Position);
		const FTransform& BLocalToWorld = FTransform(RigidBodyB.Orientation, RigidBodyB.Position);

		// Aローカル座標からBローカル座標への変換 BLocalToWorld^-1 * ALocalToWorld
		const FTransform& ALocalToBLocal = ALocalToWorld * BLocalToWorld.Inverse();
		// Bローカル座標からAローカル座標への変換 BLocalToWorld^-1 * ALocalToWorld
		const FTransform& BLocalToALocal = ALocalToBLocal.Inverse();

		// ConvexAの面法線を分離軸にしたとき
		for (const ARigidBodiesCustomMesh::FFacet& FacetA : RigidBodyA.CollisionShape.Facets)
		{
			const FVector& SeparatingAxis = FacetA.Normal;

			// ConvexAを分離軸に投影
			float MinA, MaxA;
			GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

			// ConvexBを分離軸に投影
			float MinB, MaxB;
			GetConvexProjectedRange(RigidBodyB.CollisionShape, ALocalToBLocal.TransformVector(SeparatingAxis), MinB, MaxB);
			float DistanceAlongSeparationAxis = FVector::DotProduct(BLocalToALocal.GetTranslation(), SeparatingAxis);
			MinB += DistanceAlongSeparationAxis;
			MaxB += DistanceAlongSeparationAxis;

			// 分離軸の存在判定と最も浅い貫通深度および判定軸の更新
			bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::PointBFacetA, DistanceMin, AxisMin, SAType, bAxisFlip);
			if (bSeparationPlaneExist)
			{
				return false;
			}
		}

		// ConvexBの面法線を分離軸にしたとき
		for (const ARigidBodiesCustomMesh::FFacet& FacetB : RigidBodyB.CollisionShape.Facets)
		{
			const FVector& SeparatingAxis = BLocalToALocal.TransformVector(FacetB.Normal).GetUnsafeNormal();

			// ConvexAを分離軸に投影
			float MinA, MaxA;
			GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

			// ConvexBを分離軸に投影
			float MinB, MaxB;
			GetConvexProjectedRange(RigidBodyB.CollisionShape, FacetB.Normal, MinB, MaxB);
			float DistanceAlongSeparationAxis = FVector::DotProduct(BLocalToALocal.GetTranslation(), SeparatingAxis);
			MinB += DistanceAlongSeparationAxis;
			MaxB += DistanceAlongSeparationAxis;

			// 分離軸の存在判定と最も浅い貫通深度および判定軸の更新
			bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::PointAFacetB, DistanceMin, AxisMin, SAType, bAxisFlip);
			if (bSeparationPlaneExist)
			{
				return false;
			}
		}

		// ConvexAとConvexBのエッジの外積を分離軸としたとき
		for (const ARigidBodiesCustomMesh::FEdge& EdgeA : RigidBodyA.CollisionShape.Edges)
		{
			const FVector& EdgeVecA = RigidBodyA.CollisionShape.Vertices[EdgeA.VertId[1]] - RigidBodyA.CollisionShape.Vertices[EdgeA.VertId[0]];

			for (const ARigidBodiesCustomMesh::FEdge& EdgeB : RigidBodyB.CollisionShape.Edges)
			{
				const FVector& EdgeVecB = BLocalToALocal.TransformVector(RigidBodyB.CollisionShape.Vertices[EdgeB.VertId[1]] - RigidBodyB.CollisionShape.Vertices[EdgeB.VertId[0]]);

				FVector SeparatingAxis = FVector::CrossProduct(EdgeVecA, EdgeVecB);
				if (SeparatingAxis.SizeSquared() < SMALL_NUMBER)
				{
					continue;
				}

				SeparatingAxis = SeparatingAxis.GetUnsafeNormal();

				// ConvexAを分離軸に投影
				float MinA, MaxA;
				GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

				// ConvexBを分離軸に投影
				float MinB, MaxB;
				GetConvexProjectedRange(RigidBodyB.CollisionShape, ALocalToBLocal.TransformVector(SeparatingAxis), MinB, MaxB);
				float DistanceAlongSeparationAxis = FVector::DotProduct(BLocalToALocal.GetTranslation(), SeparatingAxis);
				MinB += DistanceAlongSeparationAxis;
				MaxB += DistanceAlongSeparationAxis;

				bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::EdgeEdge, DistanceMin, AxisMin, SAType, bAxisFlip);
				// 分離軸の存在判定と最も浅い貫通深度および判定軸の更新
				if (bSeparationPlaneExist)
				{
					return false;
				}
			}
		}

		// ここまでくれば、分離軸は存在せず衝突したことはわかっている。
		// 衝突座標を検出する。
		float ClosestDistanceSq = FLT_MAX;
		FVector ClosestPointA, ClosestPointB = FVector::ZeroVector;
		const FVector& Separation = 1.1f * FMath::Abs(DistanceMin) * AxisMin;

		for (const ARigidBodiesCustomMesh::FFacet& FacetA : RigidBodyA.CollisionShape.Facets)
		{
			float FacetACheckValue = FVector::DotProduct(FacetA.Normal, -AxisMin);
			if (FacetACheckValue < 0.0f)
			{
				// 判定軸と逆向きの面はSeparationTypeがなんであろうと評価しない
				continue;
			}

			if (SAType == SeparationAxisType::PointBFacetA && FacetACheckValue < 0.99f && bAxisFlip)
			{
				// 判定軸がAの面法線のとき、向きの違うAの面は判定しない
				continue;
			}

			for (const ARigidBodiesCustomMesh::FFacet& FacetB : RigidBodyB.CollisionShape.Facets)
			{
				float FacetBCheckValue = FVector::DotProduct(FacetB.Normal, (ALocalToBLocal.TransformVector(AxisMin)));
				if (FacetBCheckValue < 0.0f)
				{
					// 判定軸と逆向きの面はSeparationTypeがなんであろうと評価しない
					continue;
				}

				if (SAType == SeparationAxisType::PointAFacetB && FacetBCheckValue < 0.99f && !bAxisFlip)
				{
					// 判定軸がBの面法線のとき、向きの違うBの面は判定しない
					continue;
				}

				// 面Aと面Bの最近接点を求める

				FVector TriangleA[3] = {
					Separation + RigidBodyA.CollisionShape.Vertices[FacetA.VertId[0]],
					Separation + RigidBodyA.CollisionShape.Vertices[FacetA.VertId[1]],
					Separation + RigidBodyA.CollisionShape.Vertices[FacetA.VertId[2]],
				};

				FVector TriangleB[3] = {
					BLocalToALocal.TransformPosition(RigidBodyB.CollisionShape.Vertices[FacetB.VertId[0]]),
					BLocalToALocal.TransformPosition(RigidBodyB.CollisionShape.Vertices[FacetB.VertId[1]]),
					BLocalToALocal.TransformPosition(RigidBodyB.CollisionShape.Vertices[FacetB.VertId[2]]),
				};

				// エッジ同士の最近接点算出
				for (int32 i = 0; i < 3; ++i)
				{
					for (int32 j = 0; j < 3; ++j)
					{
						FVector PointA, PointB;
						FMath::SegmentDistToSegment(TriangleA[i], TriangleA[(i + 1) % 3], TriangleB[i], TriangleB[(i + 1) % 3], PointA, PointB);

						float DistSq = (PointA - PointB).SizeSquared();
						if (DistSq < ClosestDistanceSq)
						{
							ClosestDistanceSq = DistSq;
							ClosestPointA = PointA;
							ClosestPointB = PointB;
						}
					}
				}

				// 頂点Aと面Bの最近接点算出
				for (int32 i = 0; i < 3; ++i)
				{
					for (int32 j = 0; j < 3; ++j)
					{
						const FVector& PointA = TriangleA[i];
						const FVector& PointB = FMath::ClosestPointOnTriangleToPoint(PointA, TriangleB[0], TriangleB[1], TriangleB[2]);

						float DistSq = (PointA - PointB).SizeSquared();
						if (DistSq < ClosestDistanceSq)
						{
							ClosestDistanceSq = DistSq;
							ClosestPointA = PointA;
							ClosestPointB = PointB;
						}
					}
				}

				// 頂点Bと面Aの最近接点算出
				for (int32 i = 0; i < 3; ++i)
				{
					for (int32 j = 0; j < 3; ++j)
					{
						const FVector& PointB = TriangleB[i];
						const FVector& PointA = FMath::ClosestPointOnTriangleToPoint(PointB, TriangleA[0], TriangleA[1], TriangleA[2]);

						float DistSq = (PointA - PointB).SizeSquared();
						if (DistSq < ClosestDistanceSq)
						{
							ClosestDistanceSq = DistSq;
							ClosestPointA = PointA;
							ClosestPointB = PointB;
						}
					}
				}
			}
		}

		OutNormal = ALocalToWorld.TransformVector(AxisMin);
		OutPenetrationDepth = DistanceMin;
		OutContactPointA = ClosestPointA - Separation;
		OutContactPointB = ALocalToBLocal.TransformPosition(ClosestPointB);
		//UE_LOG(LogTemp, Log, TEXT("SAType=%d"), (uint8)SAType);
		return true;
	}
};

DECLARE_CYCLE_STAT(TEXT("RigidBody_DetectCollision"), STAT_RigidBody_DetectCollision, STATGROUP_Physics);

void ARigidBodiesCustomMesh::DetectCollision()
{
	SCOPE_CYCLE_COUNTER(STAT_RigidBody_DetectCollision);

	int32 ContactPairIdx = 0;

	for (int32 i = 0; i < NumRigidBodies + 2; i++)
	{
		for (int32 j = i + 1; j < NumRigidBodies + 2; j++)
		{
			const FRigidBody& RigidBodyA = RigidBodies[i];
			const FRigidBody& RigidBodyB = RigidBodies[j];

			// TODO:本来はブロードフェーズ段階でリフレッシュする
			ContactPairs[ContactPairIdx].Refresh(RigidBodyA.Position, RigidBodyA.Orientation, RigidBodyB.Position, RigidBodyB.Orientation);

			// TODO:box同士なので、ひとつの面に2つのトライアングルで扱う必要なく、4頂点の1面で扱ったほうがエッジも減るし計算量減らせるが
			// とりあえずconvex-convexで計算する
			FVector Normal;
			float PenetrationDepth;
			FVector ContactPointA;
			FVector ContactPointB;

			bool bJointExist = false;
			for (const FJoint& Joint : Joints)
			{
				if ((Joint.RigidBodyA_Idx == i && Joint.RigidBodyB_Idx == j)
					|| (Joint.RigidBodyA_Idx == j && Joint.RigidBodyB_Idx == i))
				{
					bJointExist = true;
				}
			}

			// TODO:EasyPhysicsでは面数を見てAとBのどちらを座標系基準にするか決めてるがとりあえず省略
			if (!bJointExist)
			{
				bool bContacting = DetectConvexConvexContact(RigidBodyA, RigidBodyB, Normal, PenetrationDepth, ContactPointA, ContactPointB);
				if (bContacting)
				{
					// コンタクトペア情報はキャッシュしてない。毎フレーム作り直している
					ContactPairs[ContactPairIdx].AddContact(ContactPointA, ContactPointB, Normal, PenetrationDepth);
				}
			}

			ContactPairIdx++; 	
		}
	}

#if ENABLE_DRAW_DEBUG
	if (bDebugDrawContact)
	{
		// コンタクト情報のデバッグ表示
		for (FContactPair& ContactPair : ContactPairs)
		{
			const FRigidBody& RigidBodyA = RigidBodies[ContactPair.RigidBodyA_Idx];
			const FRigidBody& RigidBodyB = RigidBodies[ContactPair.RigidBodyB_Idx];

			for (int32 i = 0; i < ContactPair.NumContact; ++i)
			{
				FContact& Contact = ContactPair.Contacts[i];
				bool bPersistentLines = false;
				float LifeTime = -1.0f;

				const FVector& PointA_WS = RigidBodyA.Position + RigidBodyA.Orientation * Contact.ContactPointA;
				DrawDebugPoint(GetWorld(), PointA_WS, 5.0f, FColor::Red, bPersistentLines, LifeTime, SDPG_Foreground);
				DrawDebugLine(GetWorld(), PointA_WS, PointA_WS + Contact.Normal * 10.0f, FColor::Cyan, bPersistentLines, LifeTime, SDPG_Foreground);

				const FVector& PointB_WS = RigidBodyB.Position + RigidBodyB.Orientation * Contact.ContactPointB;
				DrawDebugPoint(GetWorld(), PointB_WS, 5.0f, FColor::Blue, bPersistentLines, LifeTime, SDPG_Foreground);
				DrawDebugLine(GetWorld(), PointB_WS, PointB_WS - Contact.Normal * 10.0f, FColor::Cyan, bPersistentLines, LifeTime, SDPG_Foreground);
			}
		}
	}
#endif
}

namespace
{
	FMatrix CrossMatrix(const FVector& V)
	{
		// FMatrix.M[][]はM[Column][Row]だが、コンストラクタでは各ベクトルが同じColumnに入る
		FMatrix Ret(
			FVector(0.0f, V.Z, -V.Y),
			FVector(-V.Z, 0.0f, V.X),
			FVector(V.Y, -V.X, 0.0f),
			FVector(0.0f, 0.0f, 0.0f)
		);
		return Ret;
	}
}

DECLARE_CYCLE_STAT(TEXT("RigidBody_SolveConstraint"), STAT_RigidBody_SolveConstraint, STATGROUP_Physics);

void ARigidBodiesCustomMesh::SolveConstraint(float DeltaSeconds)
{
	SCOPE_CYCLE_COUNTER(STAT_RigidBody_SolveConstraint);

	// コンストレイントソルバー用の剛体ワークデータを設定
	for (int32 i = 0; i < NumRigidBodies + 2; i++)
	{
		const FRigidBody& RigidBody = RigidBodies[i];
		FSolverBody& SolverBody = SolverBodies[i];

		SolverBody.Orientation = RigidBody.Orientation; // TODO:このコピーは必要ないな。RigidBody.Orientationを直に使えば十分だ
		SolverBody.DeltaLinearVelocity = FVector::ZeroVector;
		SolverBody.DeltaAngularVelocity = FVector::ZeroVector;

		switch (RigidBody.MotionType)
		{
		case ERigdBodyMotionType::Active:
			{
				SolverBody.MassInv = 1.0f / RigidBody.Mass;
				//SolverBody.InertiaInv = (SolverBody.Orientation.Inverse() * RigidBody.Inertia).InverseFast(); // TODO:これは間違い。原因を調べよう。乗算が逆扱いになる？
				const FMatrix& OrientationMat = FTransform(SolverBody.Orientation).ToMatrixNoScale();
				SolverBody.InertiaInv = OrientationMat * RigidBody.Inertia.Inverse() * OrientationMat.GetTransposed();
			}
			break;
		case ERigdBodyMotionType::Static:
			SolverBody.MassInv = 0.0f;
			SolverBody.InertiaInv *= 0.0f; // 手軽に0行列にできるコンストラクタがないので乗算で0行列にする
			break;
		default:
			check(false);
			break;
		}
	}

	// ジョイントコンストレイントのセットアップ
	for (FJoint& Joint : Joints)
	{
		const FRigidBody& RigidBodyA = RigidBodies[Joint.RigidBodyA_Idx];
		const FRigidBody& RigidBodyB = RigidBodies[Joint.RigidBodyB_Idx];
		FSolverBody& SolverBodyA = SolverBodies[Joint.RigidBodyA_Idx];
		FSolverBody& SolverBodyB = SolverBodies[Joint.RigidBodyB_Idx];

		const FVector& RotatedPointA = SolverBodyA.Orientation * Joint.AnchorA;
		const FVector& RotatedPointB = SolverBodyB.Orientation * Joint.AnchorB;

		const FVector& PositionA = RigidBodyA.Position + RotatedPointA;
		const FVector& PositionB = RigidBodyB.Position + RotatedPointB;

		FVector Direction = FVector::ZeroVector;
		float Distance = 0.0f;
		(PositionA - PositionB).ToDirectionAndLength(Direction, Distance);

		if (Distance < SMALL_NUMBER)
		{
			// インパルスが発生しないようにする
			Joint.Constraint.Axis = FVector(1.0f, 0.0f, 0.0f);
			Joint.Constraint.JacobianDiagInv = 0.0f;
			Joint.Constraint.RHS = 0.0f;
			Joint.Constraint.LowerLimit = -FLT_MAX;
			Joint.Constraint.UpperLimit = FLT_MAX;
			continue;
		}

		const FMatrix& K = FMatrix::Identity * (SolverBodyA.MassInv + SolverBodyB.MassInv) + (CrossMatrix(RotatedPointA) * SolverBodyA.InertiaInv * CrossMatrix(RotatedPointA) * -1) + (CrossMatrix(RotatedPointB) * SolverBodyB.InertiaInv * CrossMatrix(RotatedPointB) * -1);

		const FVector& VelocityA = RigidBodyA.LinearVelocity + FVector::CrossProduct(RigidBodyA.AngularVelocity, RotatedPointA); // TODO:角速度による速度ってrxwじゃなかったっけ？
		const FVector& VelocityB = RigidBodyB.LinearVelocity + FVector::CrossProduct(RigidBodyB.AngularVelocity, RotatedPointB);
		const FVector& RelativeVelocity = VelocityA - VelocityB;

		Joint.Constraint.Axis = Direction;
		const FVector& KdotAxis = K.TransformVector(Direction);
		Joint.Constraint.JacobianDiagInv = 1.0f / FVector::DotProduct(KdotAxis, Direction);
		Joint.Constraint.RHS = -FVector::DotProduct(RelativeVelocity, Direction); // velocity error
		Joint.Constraint.RHS -= Joint.Bias * Distance / DeltaSeconds; // position error
		Joint.Constraint.RHS *= Joint.Constraint.JacobianDiagInv;
		Joint.Constraint.LowerLimit = -FLT_MAX;
		Joint.Constraint.UpperLimit = FLT_MAX;
		Joint.Constraint.AccumImpulse = 0.0f;
	}

	// コリジョンコンストレイントのセットアップ
	for (FContactPair& ContactPair : ContactPairs)
	{
		const FRigidBody& RigidBodyA = RigidBodies[ContactPair.RigidBodyA_Idx];
		const FRigidBody& RigidBodyB = RigidBodies[ContactPair.RigidBodyB_Idx];
		FSolverBody& SolverBodyA = SolverBodies[ContactPair.RigidBodyA_Idx];
		FSolverBody& SolverBodyB = SolverBodies[ContactPair.RigidBodyB_Idx];

		ContactPair.Friction = FMath::Sqrt(RigidBodyA.Friction * RigidBodyB.Friction);
		for (int32 i = 0; i < ContactPair.NumContact; ++i)
		{
			FContact& Contact = ContactPair.Contacts[i];

			const FVector& RotatedPointA = SolverBodyA.Orientation * Contact.ContactPointA;
			const FVector& RotatedPointB = SolverBodyB.Orientation * Contact.ContactPointB;
			
			//// ログを出すのをコンタクトがあるときに限定したいのでここに入れている
			//if (i == 0)
			//{
			//	UE_LOG(LogTemp, Log, TEXT("RigidBodyB.LinearVelocity=(%f, %f, %f)"), RigidBodyB.LinearVelocity.X, RigidBodyB.LinearVelocity.Y, RigidBodyB.LinearVelocity.Z);
			//	UE_LOG(LogTemp, Log, TEXT("RigidBodyB.AngularVelocity=(%f, %f, %f)"), RigidBodyB.AngularVelocity.X, RigidBodyB.AngularVelocity.Y, RigidBodyB.AngularVelocity.Z);
			//}

			//UE_LOG(LogTemp, Log, TEXT("i=%d, ContactPointB=(%f, %f, %f)"), i, Contact.ContactPointB.X, Contact.ContactPointB.Y, Contact.ContactPointB.Z);
			//UE_LOG(LogTemp, Log, TEXT("i=%d, PenetrationDepth=%f"), i, Contact.PenetrationDepth);

			// FMatrixにはoperator+()はあるがoperator-()がない。
			const FMatrix& K = FMatrix::Identity * (SolverBodyA.MassInv + SolverBodyB.MassInv) + (CrossMatrix(RotatedPointA) * SolverBodyA.InertiaInv * CrossMatrix(RotatedPointA) * -1) + (CrossMatrix(RotatedPointB) * SolverBodyB.InertiaInv * CrossMatrix(RotatedPointB) * -1);

			const FVector& VelocityA = RigidBodyA.LinearVelocity + FVector::CrossProduct(RigidBodyA.AngularVelocity, RotatedPointA); // TODO:角速度による速度ってrxwじゃなかったっけ？
			const FVector& VelocityB = RigidBodyB.LinearVelocity + FVector::CrossProduct(RigidBodyB.AngularVelocity, RotatedPointB);
			//UE_LOG(LogTemp, Log, TEXT("i=%d, VelocityA=(%f, %f, %f)"), i, VelocityA.X, VelocityA.Y, VelocityA.Z);
			//UE_LOG(LogTemp, Log, TEXT("i=%d, VelocityB=(%f, %f, %f)"), i, VelocityB.X, VelocityB.Y, VelocityB.Z);
			const FVector& RelativeVelocity = VelocityA - VelocityB;

			FVector Tangent1, Tangent2;
			Contact.Normal.FindBestAxisVectors(Tangent1, Tangent2); // 左手系で計算しているのに注意

			float ContactRestitution = 0.0f;
			if (ContactPair.State == EContactPairState::New)
			{
				// 新規に衝突したときのみ反発力を発生させる
				ContactRestitution = (RigidBodyA.Restitution + RigidBodyB.Restitution) * 0.5f;
			}
			//UE_LOG(LogTemp, Log, TEXT("i=%d, ContactRestitution=%f"), i, ContactRestitution);

			// Normal
			{
				const FVector& Axis = Contact.Normal;
				//UE_LOG(LogTemp, Log, TEXT("Normal Axis i=%d, (%f, %f, %f)"), i, Axis.X, Axis.Y, Axis.Z);
				Contact.Constraints[0].Axis = Axis;
				const FVector& KdotAxis = K.TransformVector(Axis);
				//UE_LOG(LogTemp, Log, TEXT("Normal KdotAxis i=%d, (%f, %f, %f)"), i, KdotAxis.X, KdotAxis.Y, KdotAxis.Z);
				Contact.Constraints[0].JacobianDiagInv = 1.0f / FVector::DotProduct(KdotAxis, Axis);
				//Contact.Constraints[0].JacobianDiagInv = 1.0f / (FVector(K.TransformVector(Axis)) | Axis);
				//UE_LOG(LogTemp, Log, TEXT("Normal i=%d, JacobianDiagInv=%f"), i, Contact.Constraints[0].JacobianDiagInv);
				Contact.Constraints[0].RHS = -(1.0f + ContactRestitution) * FVector::DotProduct(RelativeVelocity, Axis); // velocity error
				//UE_LOG(LogTemp, Log, TEXT("Normal i=%d, RHS=%f"), i, Contact.Constraints[0].RHS);
				Contact.Constraints[0].RHS -= (ContactBias * FMath::Min(0.0f, Contact.PenetrationDepth + ContactSlop)) / DeltaSeconds; // position error
				//UE_LOG(LogTemp, Log, TEXT("Normal i=%d, RHS=%f"), i, Contact.Constraints[0].RHS);
				Contact.Constraints[0].RHS *= Contact.Constraints[0].JacobianDiagInv;
				//UE_LOG(LogTemp, Log, TEXT("Normal i=%d, RHS=%f"), i, Contact.Constraints[0].RHS);
				Contact.Constraints[0].LowerLimit = 0.0f;
				Contact.Constraints[0].UpperLimit = FLT_MAX;
			}

			// Tangent1
			{
				const FVector& Axis = Tangent1;
				//UE_LOG(LogTemp, Log, TEXT("Tangent1 Axis i=%d, (%f, %f, %f)"), i, Axis.X, Axis.Y, Axis.Z);
				Contact.Constraints[1].Axis = Axis;
				const FVector& KdotAxis = K.TransformVector(Axis);
				//UE_LOG(LogTemp, Log, TEXT("Tangent1 KdotAxis i=%d, (%f, %f, %f)"), i, KdotAxis.X, KdotAxis.Y, KdotAxis.Z);
				Contact.Constraints[1].JacobianDiagInv = 1.0f / FVector::DotProduct(KdotAxis, Axis);
				//Contact.Constraints[1].JacobianDiagInv = 1.0f / (FVector(K.TransformVector(Axis)) | Axis);
				//UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, JacobianDiagInv=%f"), i, Contact.Constraints[1].JacobianDiagInv);
				Contact.Constraints[1].RHS = -FVector::DotProduct(RelativeVelocity, Axis);
				//UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, RHS=%f"), i, Contact.Constraints[1].RHS);
				Contact.Constraints[1].RHS *= Contact.Constraints[1].JacobianDiagInv;
				//UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, RHS=%f"), i, Contact.Constraints[1].RHS);
				Contact.Constraints[1].LowerLimit = 0.0f;
				Contact.Constraints[1].UpperLimit = 0.0f;
			}

			// Tangent2
			{
				const FVector& Axis = Tangent2;
				//UE_LOG(LogTemp, Log, TEXT("Tangent2 Axis i=%d, (%f, %f, %f)"), i, Axis.X, Axis.Y, Axis.Z);
				Contact.Constraints[2].Axis = Axis;
				const FVector& KdotAxis = K.TransformVector(Axis);
				//UE_LOG(LogTemp, Log, TEXT("Tangent2 KdotAxis i=%d, (%f, %f, %f)"), i, KdotAxis.X, KdotAxis.Y, KdotAxis.Z);
				Contact.Constraints[2].JacobianDiagInv = 1.0f / FVector::DotProduct(KdotAxis, Axis);
				//Contact.Constraints[2].JacobianDiagInv = 1.0f / (FVector(K.TransformVector(Axis)) | Axis);
				//UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, JacobianDiagInv=%f"), i, Contact.Constraints[2].JacobianDiagInv);
				Contact.Constraints[2].RHS = -FVector::DotProduct(RelativeVelocity, Axis);
				//UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, RHS=%f"), i, Contact.Constraints[2].RHS);
				Contact.Constraints[2].RHS *= Contact.Constraints[2].JacobianDiagInv;
				//UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, RHS=%f"), i, Contact.Constraints[2].RHS);
				Contact.Constraints[2].LowerLimit = 0.0f;
				Contact.Constraints[2].UpperLimit = 0.0f;
			}
		}
	}

	// コリジョンコンストレイントのウォームスタート
	for (FContactPair& ContactPair : ContactPairs)
	{
		FSolverBody& SolverBodyA = SolverBodies[ContactPair.RigidBodyA_Idx];
		FSolverBody& SolverBodyB = SolverBodies[ContactPair.RigidBodyB_Idx];

		for (int32 i = 0; i < ContactPair.NumContact; ++i)
		{
			FContact& Contact = ContactPair.Contacts[i];

			const FVector& RotatedPointA = SolverBodyA.Orientation * Contact.ContactPointA;
			const FVector& RotatedPointB = SolverBodyB.Orientation * Contact.ContactPointB;

			for (const FConstraint& Constraint : Contact.Constraints)
			{
				float DeltaImpulse = Constraint.AccumImpulse;
				SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
				SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointA, Constraint.Axis)));
				SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
				SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointB, Constraint.Axis)));
			}
		}
	}

	// コンストレイントの反復演算
	for (int32 Itr = 0; Itr < NumIterations; Itr++)
	{
		for (FJoint& Joint : Joints)
		{
			FSolverBody& SolverBodyA = SolverBodies[Joint.RigidBodyA_Idx];
			FSolverBody& SolverBodyB = SolverBodies[Joint.RigidBodyB_Idx];
			
			const FVector& RotatedPointA = SolverBodyA.Orientation * Joint.AnchorA;
			const FVector& RotatedPointB = SolverBodyB.Orientation * Joint.AnchorB;

			FConstraint& Constraint = Joint.Constraint;
			float DeltaImpulse = Constraint.RHS;
			const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyA.DeltaAngularVelocity, RotatedPointA);
			const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyB.DeltaAngularVelocity, RotatedPointB);
			DeltaImpulse -= Constraint.JacobianDiagInv * FVector::DotProduct(Constraint.Axis, (DeltaVelocityA - DeltaVelocityB));

			float OldImpulse = Constraint.AccumImpulse;
			Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
			DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

			SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
			SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointA, Constraint.Axis)));
			SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
			SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointB, Constraint.Axis)));
		}

		for (FContactPair& ContactPair : ContactPairs)
		{
			FSolverBody& SolverBodyA = SolverBodies[ContactPair.RigidBodyA_Idx];
			FSolverBody& SolverBodyB = SolverBodies[ContactPair.RigidBodyB_Idx];

			for (int32 i = 0; i < ContactPair.NumContact; ++i)
			{
				FContact& Contact = ContactPair.Contacts[i];

				const FVector& RotatedPointA = SolverBodyA.Orientation * Contact.ContactPointA;
				const FVector& RotatedPointB = SolverBodyB.Orientation * Contact.ContactPointB;
				//UE_LOG(LogTemp, Log, TEXT("RotatedPointB i=%d, (%f, %f, %f)"), i, RotatedPointB.X, RotatedPointB.Y, RotatedPointB.Z);

				// Normal
				{
					FConstraint& Constraint = Contact.Constraints[0];
					float DeltaImpulse = Constraint.RHS;
					const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyA.DeltaAngularVelocity, RotatedPointA);
					const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyB.DeltaAngularVelocity, RotatedPointB);
					//UE_LOG(LogTemp, Log, TEXT("Normal DeltaVelocityB i=%d, (%f, %f, %f)"), i, DeltaVelocityB.X, DeltaVelocityB.Y, DeltaVelocityB.Z);
					DeltaImpulse -= Constraint.JacobianDiagInv * FVector::DotProduct(Constraint.Axis, (DeltaVelocityA - DeltaVelocityB));
					//UE_LOG(LogTemp, Log, TEXT("Normal i=%d, DeltaImpuse=%f"), i, DeltaImpulse);

					float OldImpulse = Constraint.AccumImpulse;
					Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
					DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

					SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
					SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointA, Constraint.Axis)));
					SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
					//UE_LOG(LogTemp, Log, TEXT("Normal SolverBodyB.DeltaLinearVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaLinearVelocity.X, SolverBodyB.DeltaLinearVelocity.Y, SolverBodyB.DeltaLinearVelocity.Z);
					SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointB, Constraint.Axis)));
					//UE_LOG(LogTemp, Log, TEXT("Normal SolverBodyB.DeltaAngularVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaAngularVelocity.X, SolverBodyB.DeltaAngularVelocity.Y, SolverBodyB.DeltaAngularVelocity.Z);
				}

				float MaxFriction = ContactPair.Friction * FMath::Abs(Contact.Constraints[0].AccumImpulse);
				Contact.Constraints[1].LowerLimit = -MaxFriction;
				Contact.Constraints[1].UpperLimit = MaxFriction;
				Contact.Constraints[2].LowerLimit = -MaxFriction;
				Contact.Constraints[2].UpperLimit = MaxFriction;

				// Tangent1
				{
					FConstraint& Constraint = Contact.Constraints[1];
					float DeltaImpulse = Constraint.RHS;
					const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyA.DeltaAngularVelocity, RotatedPointA);
					const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyB.DeltaAngularVelocity, RotatedPointB);
					//UE_LOG(LogTemp, Log, TEXT("Tangent1 DeltaVelocityB i=%d, (%f, %f, %f)"), i, DeltaVelocityB.X, DeltaVelocityB.Y, DeltaVelocityB.Z);
					DeltaImpulse -= Constraint.JacobianDiagInv * FVector::DotProduct(Constraint.Axis, (DeltaVelocityA - DeltaVelocityB));
					//UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, DeltaImpuse=%f"), i, DeltaImpulse);

					float OldImpulse = Constraint.AccumImpulse;
					Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
					DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

					SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
					SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointA, Constraint.Axis)));
					SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
					//UE_LOG(LogTemp, Log, TEXT("Tangent1 SolverBodyB.DeltaLinearVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaLinearVelocity.X, SolverBodyB.DeltaLinearVelocity.Y, SolverBodyB.DeltaLinearVelocity.Z);
					SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointB, Constraint.Axis)));
					//UE_LOG(LogTemp, Log, TEXT("Tangent1 SolverBodyB.DeltaAngularVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaAngularVelocity.X, SolverBodyB.DeltaAngularVelocity.Y, SolverBodyB.DeltaAngularVelocity.Z);
				}

				// Tangent2
				{
					FConstraint& Constraint = Contact.Constraints[2];
					float DeltaImpulse = Constraint.RHS;
					const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyA.DeltaAngularVelocity, RotatedPointA);
					const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + FVector::CrossProduct(SolverBodyB.DeltaAngularVelocity, RotatedPointB);
					//UE_LOG(LogTemp, Log, TEXT("Tangent2 DeltaVelocityB i=%d, (%f, %f, %f)"), i, DeltaVelocityB.X, DeltaVelocityB.Y, DeltaVelocityB.Z);
					DeltaImpulse -= Constraint.JacobianDiagInv * FVector::DotProduct(Constraint.Axis, (DeltaVelocityA - DeltaVelocityB));
					//UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, DeltaImpuse=%f"), i, DeltaImpulse);

					float OldImpulse = Constraint.AccumImpulse;
					Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
					DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

					SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
					SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointA, Constraint.Axis)));
					SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
					//UE_LOG(LogTemp, Log, TEXT("Tangent2 SolverBodyB.DeltaLinearVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaLinearVelocity.X, SolverBodyB.DeltaLinearVelocity.Y, SolverBodyB.DeltaLinearVelocity.Z);
					SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(FVector::CrossProduct(RotatedPointB, Constraint.Axis)));
					//UE_LOG(LogTemp, Log, TEXT("Tangent2 SolverBodyB.DeltaAngularVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaAngularVelocity.X, SolverBodyB.DeltaAngularVelocity.Y, SolverBodyB.DeltaAngularVelocity.Z);
				}
			}
		}
	}

	// 速度を更新
	for (int32 i = 0; i < NumRigidBodies + 2; i++)
	{
		// StaticであればInvMassやInvInertiaTensorが0なのでDeltaLinearVelocityとDeltaAngularVelocityも0だが一応。
		if (RigidBodies[i].MotionType == ERigdBodyMotionType::Static)
		{
			continue;
		}

		RigidBodies[i].LinearVelocity += SolverBodies[i].DeltaLinearVelocity;
		RigidBodies[i].AngularVelocity += SolverBodies[i].DeltaAngularVelocity;
	}

	//UE_LOG(LogTemp, Log, TEXT("=======================================================")); // 改行
}

DECLARE_CYCLE_STAT(TEXT("RigidBody_Integrate"), STAT_RigidBody_Integrate, STATGROUP_Physics);

void ARigidBodiesCustomMesh::Integrate(int32 RBIdx, float DeltaSeconds)
{
	SCOPE_CYCLE_COUNTER(STAT_RigidBody_Integrate);

	if (RigidBodies[RBIdx].MotionType == ERigdBodyMotionType::Static)
	{
		return;
	}

	RigidBodies[RBIdx].LinearVelocity += FVector(0.0f, 0.0f, Gravity) * DeltaSeconds;
	RigidBodies[RBIdx].Position += RigidBodies[RBIdx].LinearVelocity * DeltaSeconds;

	// 速度と位置が発散しないように適当な値でクランプ
	RigidBodies[RBIdx].LinearVelocity = RigidBodies[RBIdx].LinearVelocity.BoundToCube(100000);
	RigidBodies[RBIdx].Position = RigidBodies[RBIdx].Position.BoundToCube(100000);

	const FQuat& OrientationDifferential = FQuat(RigidBodies[RBIdx].AngularVelocity.X, RigidBodies[RBIdx].AngularVelocity.Y, RigidBodies[RBIdx].AngularVelocity.Z, 0.0f) * RigidBodies[RBIdx].Orientation * 0.5f;
	RigidBodies[RBIdx].Orientation = (RigidBodies[RBIdx].Orientation + OrientationDifferential * DeltaSeconds).GetNormalized();
}

void ARigidBodiesCustomMesh::ApplyRigidBodiesToMeshDrawing()
{
	// CustomMeshComponentのメッシュの設定
	TArray<FCustomMeshTriangle> CustomMeshTris;

	for (const FRigidBody& RigidBody : RigidBodies)
	{
		for (const FFacet& Facet : RigidBody.CollisionShape.Facets)
		{
			FCustomMeshTriangle Tri;
			Tri.Vertex0 = RigidBody.Position + RigidBody.Orientation * RigidBody.CollisionShape.Vertices[Facet.VertId[0]];
			Tri.Vertex1 = RigidBody.Position + RigidBody.Orientation * RigidBody.CollisionShape.Vertices[Facet.VertId[1]];
			Tri.Vertex2 = RigidBody.Position + RigidBody.Orientation * RigidBody.CollisionShape.Vertices[Facet.VertId[2]];
			CustomMeshTris.Add(Tri);
		}
	}

	DrawMesh->SetCustomMeshTriangles(CustomMeshTris);
}


void ARigidBodiesCustomMesh::FContactPair::Refresh(const FVector& PositionA, const FQuat& OrientationA, const FVector& PositionB, const FQuat& OrientationB)
{
	// 前フレームでNewあるいはKeepだったものをNoContactにするかKeepにするか判定する
	// NoContacrのものをNewにするのはAddContact()の役割
	static const float CONTACT_THRESHOLD_NORMAL = 1.0f;
	static const float CONTACT_SQ_THRESHOLD_TANGENT = 5.0f;

	switch (State)
	{
	case EContactPairState::NoContact:
		// 何もしない
		break;
	case EContactPairState::New:
	case EContactPairState::Keep:
		{
			// 既存コンタクトポイントがコンタクトし続けているかチェックし、してなければ削除
			for (int32 i = 0; i < NumContact; ++i)
			{
				const FVector& Normal = Contacts[i].Normal;
				const FVector& ContactPointA = PositionA + OrientationA * Contacts[i].ContactPointA;
				const FVector& ContactPointB = PositionB + OrientationB * Contacts[i].ContactPointB;

				// 貫通深度の更新。プラスになり閾値を超えたらコンタクトポイント削除。
				float PenetrationDepth = FVector::DotProduct(Normal, (ContactPointA - ContactPointB));
				if (PenetrationDepth > CONTACT_THRESHOLD_NORMAL)
				{
					RemoveContact(i);
					i--;
					continue;
				}
				Contacts[i].PenetrationDepth = PenetrationDepth;

				// 深度方向を除去した距離が閾値を超えたらコンタクトポイント削除。
				float TangentDistance = ((ContactPointA - PenetrationDepth * Normal) - ContactPointB).SizeSquared();
				if (TangentDistance > CONTACT_SQ_THRESHOLD_TANGENT)
				{
					RemoveContact(i);
					i--;
					continue;
				}
			}

			// 更新されたコンタクト数から状態更新
			if (NumContact == 0)
			{
				State = EContactPairState::NoContact;
			}
			else
			{
				State = EContactPairState::Keep;
			}
		}
		break;
	default:
		check(false);
		break;
	}
}

void ARigidBodiesCustomMesh::FContactPair::AddContact(const FVector& ContactPointA, const FVector& ContactPointB, const FVector& Normal, float PenetrationDepth)
{
	// AddContact内でNewなのかKeepなのか判定する

	switch (State)
	{
	case EContactPairState::NoContact:
		{
			check(NumContact == 0);
			NumContact = 1;
			Contacts[0].ContactPointA = ContactPointA;
			Contacts[0].ContactPointB = ContactPointB;
			Contacts[0].Normal = Normal;
			Contacts[0].PenetrationDepth = PenetrationDepth;
			Contacts[0].Reset();
			State = EContactPairState::New;
		}
		break;
	case EContactPairState::Keep:
		{
			int32 Idx = FindNearestContact(ContactPointA, ContactPointB, Normal);
			if (Idx < 0) // 同じとみなせるコンタクトポイントがなかったとき
			{
				if (NumContact < 4) // TODO:マジックナンバー
				{
					// 最後の要素に追加
					Idx = NumContact;
					NumContact++;
				}
				else
				{
					// 要素を選んで交換。
					Idx = ChooseSwapContact(ContactPointA, PenetrationDepth);
				}

				Contacts[Idx].Reset();
			}

			Contacts[Idx].ContactPointA = ContactPointA;
			Contacts[Idx].ContactPointB = ContactPointB;
			Contacts[Idx].Normal = Normal;
			Contacts[Idx].PenetrationDepth = PenetrationDepth;
		}
		break;
	case EContactPairState::New:
		// NewにするのはAddContact()内のみなのでここでNewになっているのはありえない
	default:
		check(false);
		break;
	}
}

void ARigidBodiesCustomMesh::FContactPair::RemoveContact(int32 Index)
{
	// 最後の要素をもってくる。ソートはAddContact()で別途行う。
	Contacts[Index] = Contacts[NumContact - 1];
	NumContact--;
}

int32 ARigidBodiesCustomMesh::FContactPair::FindNearestContact(const FVector& ContactPointA, const FVector& ContactPointB, const FVector& Normal)
{
	static const float CONTACT_SAME_POINT_SQ_THRESHOLD = 1.0f; // TODO:EasyPhysicsではMKS単位系で0.01にしているがそれだと2乗でない距離は10cmなので大きすぎると思う
	int32 NearestIdx = -1;

	float MinDiff = CONTACT_SAME_POINT_SQ_THRESHOLD;
	for (int32 i = 0; i < NumContact; ++i)
	{
		float DiffA = (ContactPointA - Contacts[i].ContactPointA).SizeSquared();
		float DiffB = (ContactPointB - Contacts[i].ContactPointB).SizeSquared();
		if (DiffA < MinDiff && DiffB < MinDiff && FVector::DotProduct(Normal, Contacts[i].Normal) > 0.99f)
		{
			MinDiff = FMath::Max(DiffA, DiffB);
			NearestIdx = i;
		}
	}

	return NearestIdx;
}

namespace
{
	float CalculateAreaSquared(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& P3)
	{
		// 4点で決まる面積の中で最大のものを選ぶ
		float AreqSqA = FVector::CrossProduct(P0 - P1, P2 - P3).SizeSquared();
		float AreqSqB = FVector::CrossProduct(P0 - P2 , P1 - P3).SizeSquared();
		float AreqSqC = FVector::CrossProduct(P0 - P3 , P1 - P2).SizeSquared();
		return FMath::Max(FMath::Max(AreqSqA, AreqSqB), AreqSqC);
	}
}

int32 ARigidBodiesCustomMesh::FContactPair::ChooseSwapContact(const FVector& NewPointA, float NewPenetrationDepth)
{
	check(NumContact == 4);

	int32 MaxPenetrationIdx = -1;
	float MaxPenetration = NewPenetrationDepth;

	// 最も深い衝突点は排除対象から外す
	for (int32 i = 0; i < NumContact; ++i)
	{
		if (Contacts[i].PenetrationDepth < MaxPenetration)
		{
			MaxPenetrationIdx = i;
			MaxPenetration = Contacts[i].PenetrationDepth;
		}
	}

	// 各点を除いたときの衝突点が作る面積のうち最大になるものを選択
	FVector Point[4] = {
		Contacts[0].ContactPointA,
		Contacts[1].ContactPointA,
		Contacts[2].ContactPointA,
		Contacts[3].ContactPointA,
	};

	float AreaSq[4] = { 0.0f };

	if (MaxPenetrationIdx != 0)
	{
		AreaSq[0] = CalculateAreaSquared(NewPointA, Point[1], Point[2], Point[3]);
	}

	if (MaxPenetrationIdx != 1)
	{
		AreaSq[1] = CalculateAreaSquared(NewPointA, Point[0], Point[2], Point[3]);
	}

	if (MaxPenetrationIdx != 2)
	{
		AreaSq[2] = CalculateAreaSquared(NewPointA, Point[0], Point[1], Point[3]);
	}

	if (MaxPenetrationIdx != 3)
	{
		AreaSq[3] = CalculateAreaSquared(NewPointA, Point[0], Point[1], Point[2]);
	}

	int32 MaxIndex = 0;
	float MaxArea = AreaSq[0];

	for (int32 i = 1; i < 4; ++i)
	{
		if (AreaSq[i] > MaxArea)
		{
			MaxIndex = i;
			MaxArea = AreaSq[i];
		}
	}

	return MaxIndex;
}

void ARigidBodiesCustomMesh::FireAmmo(const FVector& RayStartWSPos, const FVector& RayWSDir)
{
	// この関数が呼ばれるたびに位置や速度を初期化する

	FRigidBody& AmmoRigidBody = RigidBodies[0];
	// 初期値のStaticからActiveに変更
	AmmoRigidBody.MotionType = ERigdBodyMotionType::Active;
	AmmoRigidBody.Position = RayStartWSPos;
	AmmoRigidBody.Orientation = FQuat::Identity;
	AmmoRigidBody.LinearVelocity = RayWSDir * 5000.0f; // 速度は50m/s=180km/hに固定
	AmmoRigidBody.AngularVelocity = FVector::ZeroVector;
}

