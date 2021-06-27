#include "PhysicsEngine/RigidBodiesCustomMesh.h"
#include "UObject/ConstructorHelpers.h"
#include "CustomMeshComponent.h"

ARigidBodiesCustomMesh::ARigidBodiesCustomMesh()
{
	PrimaryActorTick.bCanEverTick = true;

	DrawMesh = CreateDefaultSubobject<UCustomMeshComponent>(TEXT("CustomMeshComponent0"));
	RootComponent = DrawMesh;
}

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
}

void ARigidBodiesCustomMesh::BeginPlay()
{
	Super::BeginPlay();

	NumThreadRBs = (NumRigidBodies + 1 + NumThreads - 1) / NumThreads; // +1はフロアの分

	static TArray<FVector> BoxVertices = 
	{
		FVector(-0.5, -0.5, -0.5),
		FVector(+0.5, -0.5, -0.5),
		FVector(-0.5, +0.5, -0.5),
		FVector(+0.5, +0.5, -0.5),
		FVector(-0.5, -0.5, +0.5),
		FVector(+0.5, -0.5, +0.5),
		FVector(-0.5, +0.5, +0.5),
		FVector(+0.5, +0.5, +0.5),
	};

	static TArray<FEdge> BoxEdges = 
	{
		{{2, 3}, {4, 1}},
		{{2, 1}, {0, 1}},
		{{3, 1}, {10, 1}},
		{{0, 2}, {0, 8}},
		{{0, 1}, {0, 6}},
		{{4, 5}, {2, 7}},
		{{4, 7}, {2, 3}},
		{{5, 7}, {2, 11}},
		{{4, 6}, {3, 9}},
		{{4, 0}, {7, 8}},
		{{6, 2}, {9, 4}},
		{{6, 3}, {4, 5}},
		{{7, 3}, {5, 10}},
		{{1, 5}, {11, 6}},
		{{1, 4}, {7, 6}},
		{{0, 4}, {7, 8}},
		{{2, 4}, {8, 9}},
		{{7, 1}, {10, 11}},
	};

	static TArray<FFacet> BoxFacets = 
	{
		{{0, 1, 2}, {1, 3, 4}, FVector(0, 0, -1)},
		{{1, 3, 2}, {0, 1, 2}, FVector(0, 0, -1)},
		{{4, 7, 5}, {5, 6, 7}, FVector(0, 0, +1)},
		{{4, 6, 7}, {6, 8, 9}, FVector(0, 0, +1)},
		{{2, 3, 6}, {0, 10, 11}, FVector(0, +1, 0)},
		{{3, 7, 6}, {9, 11, 12}, FVector(0, +1, 0)},
		{{1, 0, 5}, {4, 13, 14}, FVector(0, -1, 0)},
		{{0, 4, 5}, {5, 14, 15}, FVector(0, -1, 0)},
		{{0, 2, 4}, {3, 15, 16}, FVector(-1, 0, 0)},
		{{2, 6, 4}, {8, 10, 16}, FVector(-1, 0, 0)},
		{{3, 1, 7}, {2, 12, 17}, FVector(+1, 0, 0)},
		{{1, 5, 7}, {7, 13, 17}, FVector(+1, 0, 0)},
	};

	TArray<FVector> BoxVerticesFloor;
	for (const FVector& Vertex : BoxVertices)
	{
		BoxVerticesFloor.Add(Vertex * FloorScale);
	}

	RigidBodies.SetNum(NumRigidBodies + 1); // +1はフロアの分

	// RigidBodiesは0番目はフロアに。1番目以降がキューブ。
	FRigidBody& FloorRigidBody = RigidBodies[0];
	FloorRigidBody.CollisionShape.Vertices = BoxVerticesFloor;
	FloorRigidBody.CollisionShape.Edges = BoxEdges;
	FloorRigidBody.CollisionShape.Facets = BoxFacets;
	FloorRigidBody.Position = GetActorLocation() + FloorPosition;
	// TODO:とりあえずその他の物理パラメータは初期値のまま

	TArray<FVector> BoxVerticesScaled;
	for (const FVector& Vertex : BoxVertices)
	{
		BoxVerticesScaled.Add(Vertex * CubeScale);
	}

	// InitPosRadius半径の球内にランダムに配置
	FBoxSphereBounds BoxSphere(InitPosCenter, FVector(InitPosRadius), InitPosRadius);

	for (int32 i = 1; i < RigidBodies.Num(); i++)
	{
		FRigidBody& CubeRigidBody = RigidBodies[i];
		CubeRigidBody.CollisionShape.Vertices = BoxVerticesScaled;
		CubeRigidBody.CollisionShape.Edges = BoxEdges;
		CubeRigidBody.CollisionShape.Facets = BoxFacets;
		CubeRigidBody.Position = GetActorLocation() + RandPointInSphere(BoxSphere, InitPosCenter);
		// TODO:とりあえずその他の物理パラメータは初期値のまま
	}

	ContactPairs.Reserve(((NumRigidBodies + 1) * (NumRigidBodies + 2)) / 2); //TODO: コンタクトペアは最大でも総当たりペア数。最終的には大きすぎるがとりあえずこれで。
}

void ARigidBodiesCustomMesh::Tick(float DeltaSeconds)
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

	ApplyRigidBodiesToMeshDrawing();
}

void ARigidBodiesCustomMesh::Simulate(float DeltaSeconds)
{
	ContactPairs.Reset();

	//TODO: コンタクトペア配列にマルチスレッドからアクセスするのが危険なのでとりあえずシングルスレッド
	DetectCollision();

	//TODO: 同じ剛体にマルチスレッドからアクセスするのが危険なのでとりあえずシングルスレッド
	SolveConstraint();

	ParallelFor(NumThreads,
		[this, DeltaSeconds](int32 ThreadIndex)
		{
			for (int32 RBIdx = NumThreadRBs * ThreadIndex; RBIdx < NumThreadRBs * (ThreadIndex + 1) && RBIdx < NumRigidBodies + 1; ++RBIdx)
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
			float ProjectedVal = Axis | Vertex;
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
		// TODO:EasyPhysicsでは面数を見てAとBのどちらを座標系基準にするか決めてるがとりあえずいいや

		// 最も浅い貫通深度とその分離軸。Aのローカル座標で扱い、Aを押し返すという考え方で扱う。
		float DistanceMin = -FLT_MAX;
		FVector AxisMin = FVector::ZeroVector;

		SeparationAxisType SAType = EdgeEdge;
		bool bAxisFlip = false; // Aを押し返す方向が分離軸の方向と一致すればfalse。逆方向ならtrue。

		const FTransform& ALocalToWorld = FTransform(RigidBodyA.Orientation, RigidBodyA.Position);
		const FTransform& BLocalToWorld = FTransform(RigidBodyB.Orientation, RigidBodyB.Position);

		// Aローカル座標からBローカル座標への変換 BLocalToWorld^-1 * ALocalToWorld
		const FTransform& ALocalToBLocal = ALocalToWorld * BLocalToWorld.Inverse();
		// Bローカル座標からAローカル座標への変換 BLocalToWorld^-1 * ALocalToWorld
		const FTransform& BLocalToALocal = ALocalToBLocal.Inverse();

		// ConvexAの面法線を分離軸にしたとき。Aのローカル座標であつかう　
		for (const ARigidBodiesCustomMesh::FFacet& Facet : RigidBodyA.CollisionShape.Facets)
		{
			const FVector& SeparatingAxis = Facet.Normal;

			// ConvexAを分離軸に投影
			float MinA, MaxA;
			GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

			// ConvexBを分離軸に投影
			float MinB, MaxB;
			GetConvexProjectedRange(RigidBodyB.CollisionShape, ALocalToBLocal.TransformVector(SeparatingAxis), MinB, MaxB);
			float DistanceAlongSeparationAxis = BLocalToALocal.GetTranslation() | SeparatingAxis;
			MinB += DistanceAlongSeparationAxis;
			MaxB += DistanceAlongSeparationAxis;

			// 分離軸の存在判定と最も浅い貫通深度およびその分離軸の更新
			bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::PointBFacetA, DistanceMin, AxisMin, SAType, bAxisFlip);
			if (bSeparationPlaneExist)
			{
				return false;
			}
		}

		// ConvexBの面法線を分離軸にしたとき。Bのローカル座標であつかう　
		for (const ARigidBodiesCustomMesh::FFacet& Facet : RigidBodyB.CollisionShape.Facets)
		{
			const FVector& SeparatingAxis = BLocalToALocal.TransformVector(Facet.Normal);

			// ConvexAを分離軸に投影
			float MinA, MaxA;
			GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

			// ConvexBを分離軸に投影
			float MinB, MaxB;
			GetConvexProjectedRange(RigidBodyB.CollisionShape, Facet.Normal, MinB, MaxB);
			float DistanceAlongSeparationAxis = BLocalToALocal.GetTranslation() | SeparatingAxis;
			MinB += DistanceAlongSeparationAxis;
			MaxB += DistanceAlongSeparationAxis;

			// 分離軸の存在判定と最も浅い貫通深度およびその分離軸の更新
			bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::PointAFacetB, DistanceMin, AxisMin, SAType, bAxisFlip);
			if (bSeparationPlaneExist)
			{
				return false;
			}
		}

		return false;
	}
};

void ARigidBodiesCustomMesh::DetectCollision()
{
	for (int32 i = 0; i < NumRigidBodies + 1; i++)
	{
		for (int32 j = i + 1; j < NumRigidBodies + 1; j++)
		{
			const FRigidBody& RigidBodyA = RigidBodies[i];
			const FRigidBody& RigidBodyB = RigidBodies[j];

			// TODO:box同士なので、ひとつの面に2つのトライアングルで扱う必要なく、4頂点の1面で扱ったほうがエッジも減るし計算量減らせるが
			// とりあえずconvex-convexで計算する
			FVector Normal;
			float PenetrationDepth;
			FVector ContactPointA;
			FVector ContactPointB;
			bool bContacting = DetectConvexConvexContact(RigidBodyA, RigidBodyB, Normal, PenetrationDepth, ContactPointA, ContactPointB);
			if (bContacting)
			{
				ContactPairs.Emplace(i, j, ContactPointA, ContactPointB, Normal, PenetrationDepth);
			}
		}
	}
}

void ARigidBodiesCustomMesh::SolveConstraint()
{
	for (const FContactPair& ContactPair : ContactPairs)
	{
		;
	}
}

void ARigidBodiesCustomMesh::Integrate(int32 RBIdx, float DeltaSeconds)
{
	// TODO:フロアをStatic扱いするのをとりあえずIntegrateのスキップで行う
	if (RBIdx == 0)
	{
		return;
	}

	RigidBodies[RBIdx].LinearVelocity += FVector(0.0f, 0.0f, Gravity) * DeltaSeconds;
	// TODO:仮。発散しないように
	RigidBodies[RBIdx].LinearVelocity.Z = FMath::Max(RigidBodies[RBIdx].LinearVelocity.Z, -1000.0f);

	RigidBodies[RBIdx].Position += RigidBodies[RBIdx].LinearVelocity * DeltaSeconds;
	// TODO:仮。発散しないように
	RigidBodies[RBIdx].Position.Z = FMath::Max(RigidBodies[RBIdx].Position.Z, 25.0f);

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

