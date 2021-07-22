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

	FMatrix CalculateInertiaBox(float Density, FVector Extent)
	{
		float Mass = Extent.X * Extent.Y * Extent.Z * Density;

		FMatrix Ret = FMatrix::Identity;
		Ret.M[0][0] = Mass * (Extent.Y * Extent.Y + Extent.Z * Extent.Z) / 12.0f;
		Ret.M[1][1] = Mass * (Extent.Z * Extent.Z + Extent.X * Extent.X) / 12.0f;
		Ret.M[2][2] = Mass * (Extent.X * Extent.X + Extent.Y * Extent.Y) / 12.0f;

		return Ret;
	}
}

void ARigidBodiesCustomMesh::BeginPlay()
{
	Super::BeginPlay();

	NumThreadRBs = (NumRigidBodies + 1 + NumThreads - 1) / NumThreads; // +1�̓t���A�̕�

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

	RigidBodies.SetNum(NumRigidBodies + 1); // +1�̓t���A�̕�

	// RigidBodies��0�Ԗڂ̓t���A�ɁB1�Ԗڈȍ~���L���[�u�B
	FRigidBody& FloorRigidBody = RigidBodies[0];
	FloorRigidBody.CollisionShape.Vertices = BoxVerticesFloor;
	FloorRigidBody.CollisionShape.Edges = BoxEdges;
	FloorRigidBody.CollisionShape.Facets = BoxFacets;
	FloorRigidBody.Mass = FloorScale.X * FloorScale.Y * FloorScale.Z * Density; // �t���A��Static�Ȃ̂Ŗ������ʈ����ɂ��Ă�̂Ŏg���Ă��Ȃ�
	FloorRigidBody.Inertia = CalculateInertiaBox(Density, FloorScale); // �t���A��Static�Ȃ̂Ŗ������ʈ����ɂ��Ă�̂Ŏg���Ă��Ȃ�
	FloorRigidBody.Friction = Friction;
	FloorRigidBody.Restitution = Restitution;
	FloorRigidBody.Position = GetActorLocation() + FloorPosition;
	// TODO:�Ƃ肠�������̑��̕����p�����[�^�͏����l�̂܂�

	TArray<FVector> BoxVerticesScaled;
	for (const FVector& Vertex : BoxVertices)
	{
		BoxVerticesScaled.Add(Vertex * CubeScale);
	}

	// InitPosRadius���a�̋����Ƀ����_���ɔz�u
	FBoxSphereBounds BoxSphere(InitPosCenter, FVector(InitPosRadius), InitPosRadius);

	for (int32 i = 1; i < RigidBodies.Num(); i++)
	{
		FRigidBody& CubeRigidBody = RigidBodies[i];
		CubeRigidBody.CollisionShape.Vertices = BoxVerticesScaled;
		CubeRigidBody.CollisionShape.Edges = BoxEdges;
		CubeRigidBody.CollisionShape.Facets = BoxFacets;
		CubeRigidBody.Mass = CubeScale.X * CubeScale.Y * CubeScale.Z * Density;
		CubeRigidBody.Inertia = CalculateInertiaBox(Density, CubeScale);
		CubeRigidBody.Friction = Friction;
		CubeRigidBody.Restitution = Restitution;
		CubeRigidBody.Position = GetActorLocation() + RandPointInSphereCustomMesh(BoxSphere, InitPosCenter);
		// TODO:�Ƃ肠�������̑��̕����p�����[�^�͏����l�̂܂�
	}

	ContactPairs.SetNum(((NumRigidBodies + 1) * NumRigidBodies) / 2); //TODO: �R���^�N�g�y�A�͍ő�ł���������y�A���B�ŏI�I�ɂ͑傫�����邪�Ƃ肠��������ŁB

	int32 ContactPairIdx = 0;
	for (int32 i = 0; i < NumRigidBodies + 1; i++)
	{
		for (int32 j = i + 1; j < NumRigidBodies + 1; j++)
		{
			ContactPairs[ContactPairIdx].RigidBodyA_Idx = i;
			ContactPairs[ContactPairIdx].RigidBodyB_Idx = j;
			ContactPairIdx++;
		}
	}

	SolverBodies.SetNum(NumRigidBodies + 1);
}

void ARigidBodiesCustomMesh::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSeconds�̒l�̕ϓ��Ɋւ�炸�A�V�~�����[�V�����Ɏg���T�u�X�e�b�v�^�C���͌Œ�Ƃ���
		float SubStepDeltaSeconds = 1.0f / FrameRate;
		Simulate(SubStepDeltaSeconds);
	}

	ApplyRigidBodiesToMeshDrawing();
}

void ARigidBodiesCustomMesh::Simulate(float DeltaSeconds)
{
	//TODO: �R���^�N�g�y�A�z��Ƀ}���`�X���b�h����A�N�Z�X����̂��댯�Ȃ̂łƂ肠�����V���O���X���b�h
	DetectCollision();

	//TODO: �������̂Ƀ}���`�X���b�h����A�N�Z�X����̂��댯�Ȃ̂łƂ肠�����V���O���X���b�h
	SolveConstraint(DeltaSeconds);

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
		// �ł��󂢊ђʐ[�x�Ƃ��̂Ƃ��̕������i���莲�ƌĂԁj�BA�̃��[�J�����W�ň����AA�������Ԃ��Ƃ����l�����ň����B
		float DistanceMin = -FLT_MAX;
		FVector AxisMin = FVector::ZeroVector;

		SeparationAxisType SAType = SeparationAxisType::EdgeEdge;
		bool bAxisFlip = false; // A�������Ԃ��������������̕����ƈ�v�����false�B�t�����Ȃ�true�B

		const FTransform& ALocalToWorld = FTransform(RigidBodyA.Orientation, RigidBodyA.Position);
		const FTransform& BLocalToWorld = FTransform(RigidBodyB.Orientation, RigidBodyB.Position);

		// A���[�J�����W����B���[�J�����W�ւ̕ϊ� BLocalToWorld^-1 * ALocalToWorld
		const FTransform& ALocalToBLocal = ALocalToWorld * BLocalToWorld.Inverse();
		// B���[�J�����W����A���[�J�����W�ւ̕ϊ� BLocalToWorld^-1 * ALocalToWorld
		const FTransform& BLocalToALocal = ALocalToBLocal.Inverse();

		// ConvexA�̖ʖ@���𕪗����ɂ����Ƃ�
		for (const ARigidBodiesCustomMesh::FFacet& FacetA : RigidBodyA.CollisionShape.Facets)
		{
			const FVector& SeparatingAxis = FacetA.Normal;

			// ConvexA�𕪗����ɓ��e
			float MinA, MaxA;
			GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

			// ConvexB�𕪗����ɓ��e
			float MinB, MaxB;
			GetConvexProjectedRange(RigidBodyB.CollisionShape, ALocalToBLocal.TransformVector(SeparatingAxis), MinB, MaxB);
			float DistanceAlongSeparationAxis = BLocalToALocal.GetTranslation() | SeparatingAxis;
			MinB += DistanceAlongSeparationAxis;
			MaxB += DistanceAlongSeparationAxis;

			// �������̑��ݔ���ƍł��󂢊ђʐ[�x����є��莲�̍X�V
			bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::PointBFacetA, DistanceMin, AxisMin, SAType, bAxisFlip);
			if (bSeparationPlaneExist)
			{
				return false;
			}
		}

		// ConvexB�̖ʖ@���𕪗����ɂ����Ƃ�
		for (const ARigidBodiesCustomMesh::FFacet& FacetB : RigidBodyB.CollisionShape.Facets)
		{
			const FVector& SeparatingAxis = BLocalToALocal.TransformVector(FacetB.Normal).GetUnsafeNormal();

			// ConvexA�𕪗����ɓ��e
			float MinA, MaxA;
			GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

			// ConvexB�𕪗����ɓ��e
			float MinB, MaxB;
			GetConvexProjectedRange(RigidBodyB.CollisionShape, FacetB.Normal, MinB, MaxB);
			float DistanceAlongSeparationAxis = BLocalToALocal.GetTranslation() | SeparatingAxis;
			MinB += DistanceAlongSeparationAxis;
			MaxB += DistanceAlongSeparationAxis;

			// �������̑��ݔ���ƍł��󂢊ђʐ[�x����є��莲�̍X�V
			bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::PointAFacetB, DistanceMin, AxisMin, SAType, bAxisFlip);
			if (bSeparationPlaneExist)
			{
				return false;
			}
		}

		// ConvexA��ConvexB�̃G�b�W�̊O�ς𕪗����Ƃ����Ƃ�
		for (const ARigidBodiesCustomMesh::FEdge& EdgeA : RigidBodyA.CollisionShape.Edges)
		{
			const FVector& EdgeVecA = RigidBodyA.CollisionShape.Vertices[EdgeA.VertId[1]] - RigidBodyA.CollisionShape.Vertices[EdgeA.VertId[0]];

			for (const ARigidBodiesCustomMesh::FEdge& EdgeB : RigidBodyB.CollisionShape.Edges)
			{
				const FVector& EdgeVecB = BLocalToALocal.TransformVector(RigidBodyB.CollisionShape.Vertices[EdgeB.VertId[1]] - RigidBodyB.CollisionShape.Vertices[EdgeB.VertId[0]]);

				FVector SeparatingAxis = EdgeVecA ^ EdgeVecB;
				if (SeparatingAxis.SizeSquared() < SMALL_NUMBER)
				{
					continue;
				}

				SeparatingAxis = SeparatingAxis.GetUnsafeNormal();

				// ConvexA�𕪗����ɓ��e
				float MinA, MaxA;
				GetConvexProjectedRange(RigidBodyA.CollisionShape, SeparatingAxis, MinA, MaxA);

				// ConvexB�𕪗����ɓ��e
				float MinB, MaxB;
				GetConvexProjectedRange(RigidBodyB.CollisionShape, ALocalToBLocal.TransformVector(SeparatingAxis), MinB, MaxB);
				float DistanceAlongSeparationAxis = BLocalToALocal.GetTranslation() | SeparatingAxis;
				MinB += DistanceAlongSeparationAxis;
				MaxB += DistanceAlongSeparationAxis;

				bool bSeparationPlaneExist = CheckSeparationPlaneExistAndUpdateMinPenetration(MinA, MaxA, MinB, MaxB, SeparatingAxis, SeparationAxisType::EdgeEdge, DistanceMin, AxisMin, SAType, bAxisFlip);
				// �������̑��ݔ���ƍł��󂢊ђʐ[�x����є��莲�̍X�V
				if (bSeparationPlaneExist)
				{
					return false;
				}
			}
		}

		// �����܂ł���΁A�������͑��݂����Փ˂������Ƃ͂킩���Ă���B
		// �Փˍ��W�����o����B
		float ClosestDistanceSq = FLT_MAX;
		FVector ClosestPointA, ClosestPointB = FVector::ZeroVector;
		const FVector& Separation = 1.1f * FMath::Abs(DistanceMin) * AxisMin;

		for (const ARigidBodiesCustomMesh::FFacet& FacetA : RigidBodyA.CollisionShape.Facets)
		{
			float FacetACheckValue = FacetA.Normal | -AxisMin;
			if (FacetACheckValue < 0.0f)
			{
				// ���莲�Ƌt�����̖ʂ�SeparationType���Ȃ�ł��낤�ƕ]�����Ȃ�
				continue;
			}

			if (SAType == SeparationAxisType::PointBFacetA && FacetACheckValue < 0.99f && bAxisFlip)
			{
				// ���莲��A�̖ʖ@���̂Ƃ��A�����̈ႤA�̖ʂ͔��肵�Ȃ�
				continue;
			}

			for (const ARigidBodiesCustomMesh::FFacet& FacetB : RigidBodyB.CollisionShape.Facets)
			{
				float FacetBCheckValue = FacetB.Normal | (ALocalToBLocal.TransformVector(AxisMin));
				if (FacetBCheckValue < 0.0f)
				{
					// ���莲�Ƌt�����̖ʂ�SeparationType���Ȃ�ł��낤�ƕ]�����Ȃ�
					continue;
				}

				if (SAType == SeparationAxisType::PointAFacetB && FacetBCheckValue < 0.99f && !bAxisFlip)
				{
					// ���莲��B�̖ʖ@���̂Ƃ��A�����̈ႤB�̖ʂ͔��肵�Ȃ�
					continue;
				}

				// ��A�Ɩ�B�̍ŋߐړ_�����߂�

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

				// �G�b�W���m�̍ŋߐړ_�Z�o
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

				// ���_A�Ɩ�B�̍ŋߐړ_�Z�o
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

				// ���_B�Ɩ�A�̍ŋߐړ_�Z�o
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
		UE_LOG(LogTemp, Log, TEXT("SAType=%d"), (uint8)SAType);
		return true;
	}
};

void ARigidBodiesCustomMesh::DetectCollision()
{
	int32 ContactPairIdx = 0;

	for (int32 i = 0; i < NumRigidBodies + 1; i++)
	{
		for (int32 j = i + 1; j < NumRigidBodies + 1; j++)
		{
			const FRigidBody& RigidBodyA = RigidBodies[i];
			const FRigidBody& RigidBodyB = RigidBodies[j];

			// TODO:�{���̓u���[�h�t�F�[�Y�i�K�Ń��t���b�V������
			ContactPairs[ContactPairIdx].Refresh(RigidBodyA.Position, RigidBodyA.Orientation, RigidBodyB.Position, RigidBodyB.Orientation);

			// TODO:box���m�Ȃ̂ŁA�ЂƂ̖ʂ�2�̃g���C�A���O���ň����K�v�Ȃ��A4���_��1�ʂň������ق����G�b�W�����邵�v�Z�ʌ��点�邪
			// �Ƃ肠����convex-convex�Ōv�Z����
			FVector Normal;
			float PenetrationDepth;
			FVector ContactPointA;
			FVector ContactPointB;
			// TODO:EasyPhysics�ł͖ʐ�������A��B�̂ǂ�������W�n��ɂ��邩���߂Ă邪�Ƃ肠�����ȗ�
			bool bContacting = DetectConvexConvexContact(RigidBodyA, RigidBodyB, Normal, PenetrationDepth, ContactPointA, ContactPointB);
			if (bContacting)
			{
				// �R���^�N�g�y�A���̓L���b�V�����ĂȂ��B���t���[����蒼���Ă���
				ContactPairs[ContactPairIdx].AddContact(ContactPointA, ContactPointB, Normal, PenetrationDepth);
			}

			ContactPairIdx++; 	
		}
	}
}

namespace
{
	FMatrix CrossMatrix(const FVector& V)
	{
		// FMatrix.M[][]��M[Column][Row]�����A�R���X�g���N�^�ł͊e�x�N�g��������Column�ɓ���
		FMatrix Ret(
			FVector(0.0f, V.Z, -V.Y),
			FVector(-V.Z, 0.0f, V.X),
			FVector(V.Y, -V.X, 0.0f),
			FVector(0.0f, 0.0f, 0.0f)
		);
		return Ret;
	}

#if 0 // ���ǂ��₵���Ǝv�����̂͊��Ⴂ�������悤�Ȃ̂ŃR�����g�A�E�g
	// FMatrix operator*(const FMatrix& M) const;���AFQuat::Inverse()��PLATFORM_ENABLE_VECTORINTRINSICS=1�̂Ƃ��̓��삪���₵������0�̂Ƃ��ɂ���������
	FMatrix MultiplyQuatToMatrix(const FQuat& Q, const FMatrix& M)
	{
		FMatrix Result;
		FQuat VT, VR;
		FQuat Inv = FQuat(-Q.X, -Q.Y, -Q.Z, Q.W);
		for (int32 I=0; I<4; ++I)
		{
			FQuat VQ(M.M[I][0], M.M[I][1], M.M[I][2], M.M[I][3]);
			VectorQuaternionMultiply(&VT, &Q, &VQ);
			VectorQuaternionMultiply(&VR, &VT, &Inv);
			Result.M[I][0] = VR.X;
			Result.M[I][1] = VR.Y;
			Result.M[I][2] = VR.Z;
			Result.M[I][3] = VR.W;
		}

		return Result;
	}
#endif
}

void ARigidBodiesCustomMesh::SolveConstraint(float DeltaSeconds)
{
	// �R���X�g���C���g�\���o�[�p�̍��̃��[�N�f�[�^��ݒ�
	for (int32 i = 0; i < NumRigidBodies + 1; i++)
	{
		const FRigidBody& RigidBody = RigidBodies[i];
		FSolverBody& SolverBody = SolverBodies[i];

		SolverBody.Orientation = RigidBody.Orientation;
		SolverBody.DeltaLinearVelocity = FVector::ZeroVector;
		SolverBody.DeltaAngularVelocity = FVector::ZeroVector;

		if (i == 0)
		{
			// �t���A��Static����
			SolverBody.MassInv = 0.0f;
			SolverBody.InertiaInv *= 0.0f; // ��y��0�s��ɂł���R���X�g���N�^���Ȃ��̂ŏ�Z��0�s��ɂ���
		}
		else
		{
			SolverBody.MassInv = 1.0f / RigidBody.Mass;
			SolverBody.InertiaInv = (SolverBody.Orientation * RigidBody.Inertia).InverseFast();
		}
	}

	// �R���W�����R���X�g���C���g�̃Z�b�g�A�b�v
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
			
			// ���O���o���̂��R���^�N�g������Ƃ��Ɍ��肵�����̂ł����ɓ���Ă���
			if (i == 0)
			{
				UE_LOG(LogTemp, Log, TEXT("RigidBodyB.LinearVelocity=(%f, %f, %f)"), RigidBodyB.LinearVelocity.X, RigidBodyB.LinearVelocity.Y, RigidBodyB.LinearVelocity.Z);
				UE_LOG(LogTemp, Log, TEXT("RigidBodyB.AngularVelocity=(%f, %f, %f)"), RigidBodyB.AngularVelocity.X, RigidBodyB.AngularVelocity.Y, RigidBodyB.AngularVelocity.Z);
			}

			UE_LOG(LogTemp, Log, TEXT("i=%d, ContactPointB=(%f, %f, %f)"), i, Contact.ContactPointB.X, Contact.ContactPointB.Y, Contact.ContactPointB.Z);
			UE_LOG(LogTemp, Log, TEXT("i=%d, PenetrationDepth=%f"), i, Contact.PenetrationDepth);

			// FMatrix�ɂ�operator+()�͂��邪operator-()���Ȃ��B
			const FMatrix& K = FMatrix::Identity * (SolverBodyA.MassInv + SolverBodyB.MassInv) + (CrossMatrix(RotatedPointA) * SolverBodyA.InertiaInv * CrossMatrix(RotatedPointA) * -1) + (CrossMatrix(RotatedPointB) * SolverBodyB.InertiaInv * CrossMatrix(RotatedPointB) * -1);

			const FVector& VelocityA = RigidBodyA.LinearVelocity + (RigidBodyA.AngularVelocity ^ RotatedPointA); // TODO:�p���x�ɂ�鑬�x����rxw����Ȃ����������H
			const FVector& VelocityB = RigidBodyB.LinearVelocity + (RigidBodyB.AngularVelocity ^ RotatedPointB);
			UE_LOG(LogTemp, Log, TEXT("i=%d, VelocityA=(%f, %f, %f)"), i, VelocityA.X, VelocityA.Y, VelocityA.Z);
			UE_LOG(LogTemp, Log, TEXT("i=%d, VelocityB=(%f, %f, %f)"), i, VelocityB.X, VelocityB.Y, VelocityB.Z);
			const FVector& RelativeVelocity = VelocityA - VelocityB;

			FVector Tangent1, Tangent2;
			Contact.Normal.FindBestAxisVectors(Tangent1, Tangent2); // ����n�Ōv�Z���Ă���̂ɒ���

			float ContactRestitution = 0.0f;
			if (ContactPair.State == EContactPairState::New)
			{
				// �V�K�ɏՓ˂����Ƃ��̂ݔ����͂𔭐�������
				ContactRestitution = (RigidBodyA.Restitution + RigidBodyB.Restitution) * 0.5f;
			}
			UE_LOG(LogTemp, Log, TEXT("i=%d, ContactRestitution=%f"), i, ContactRestitution);

			// Normal
			{
				const FVector& Axis = Contact.Normal;
				UE_LOG(LogTemp, Log, TEXT("Normal Axis i=%d, (%f, %f, %f)"), i, Axis.X, Axis.Y, Axis.Z);
				Contact.Constraints[0].Axis = Axis;
				const FVector& KdotAxis = K.TransformVector(Axis);
				UE_LOG(LogTemp, Log, TEXT("Normal KdotAxis i=%d, (%f, %f, %f)"), i, KdotAxis.X, KdotAxis.Y, KdotAxis.Z);
				Contact.Constraints[0].JacobianDiagInv = 1.0f / (KdotAxis | Axis);
				//Contact.Constraints[0].JacobianDiagInv = 1.0f / (FVector(K.TransformVector(Axis)) | Axis);
				UE_LOG(LogTemp, Log, TEXT("Normal i=%d, JacobianDiagInv=%f"), i, Contact.Constraints[0].JacobianDiagInv);
				Contact.Constraints[0].RHS = -(1.0f + ContactRestitution) * (RelativeVelocity | Axis); // velocity error
				UE_LOG(LogTemp, Log, TEXT("Normal i=%d, RHS=%f"), i, Contact.Constraints[0].RHS);
				Contact.Constraints[0].RHS -= (ContactBias * FMath::Min(0.0f, Contact.PenetrationDepth + ContactSlop)) / DeltaSeconds; // position error
				UE_LOG(LogTemp, Log, TEXT("Normal i=%d, RHS=%f"), i, Contact.Constraints[0].RHS);
				Contact.Constraints[0].RHS *= Contact.Constraints[0].JacobianDiagInv;
				UE_LOG(LogTemp, Log, TEXT("Normal i=%d, RHS=%f"), i, Contact.Constraints[0].RHS);
				Contact.Constraints[0].LowerLimit = 0.0f;
				Contact.Constraints[0].UpperLimit = FLT_MAX;
			}

			// Tangent1
			{
				const FVector& Axis = Tangent1;
				UE_LOG(LogTemp, Log, TEXT("Tangent1 Axis i=%d, (%f, %f, %f)"), i, Axis.X, Axis.Y, Axis.Z);
				Contact.Constraints[1].Axis = Axis;
				const FVector& KdotAxis = K.TransformVector(Axis);
				UE_LOG(LogTemp, Log, TEXT("Tangent1 KdotAxis i=%d, (%f, %f, %f)"), i, KdotAxis.X, KdotAxis.Y, KdotAxis.Z);
				Contact.Constraints[1].JacobianDiagInv = 1.0f / (KdotAxis | Axis);
				//Contact.Constraints[1].JacobianDiagInv = 1.0f / (FVector(K.TransformVector(Axis)) | Axis);
				UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, JacobianDiagInv=%f"), i, Contact.Constraints[1].JacobianDiagInv);
				Contact.Constraints[1].RHS = -RelativeVelocity | Axis;
				UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, RHS=%f"), i, Contact.Constraints[1].RHS);
				Contact.Constraints[1].RHS *= Contact.Constraints[1].JacobianDiagInv;
				UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, RHS=%f"), i, Contact.Constraints[1].RHS);
				Contact.Constraints[1].LowerLimit = 0.0f;
				Contact.Constraints[1].UpperLimit = 0.0f;
			}

			// Tangent2
			{
				const FVector& Axis = Tangent2;
				UE_LOG(LogTemp, Log, TEXT("Tangent2 Axis i=%d, (%f, %f, %f)"), i, Axis.X, Axis.Y, Axis.Z);
				Contact.Constraints[2].Axis = Axis;
				const FVector& KdotAxis = K.TransformVector(Axis);
				UE_LOG(LogTemp, Log, TEXT("Tangent2 KdotAxis i=%d, (%f, %f, %f)"), i, KdotAxis.X, KdotAxis.Y, KdotAxis.Z);
				Contact.Constraints[2].JacobianDiagInv = 1.0f / (KdotAxis | Axis);
				//Contact.Constraints[2].JacobianDiagInv = 1.0f / (FVector(K.TransformVector(Axis)) | Axis);
				UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, JacobianDiagInv=%f"), i, Contact.Constraints[2].JacobianDiagInv);
				Contact.Constraints[2].RHS = -RelativeVelocity | Axis;
				UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, RHS=%f"), i, Contact.Constraints[2].RHS);
				Contact.Constraints[2].RHS *= Contact.Constraints[2].JacobianDiagInv;
				UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, RHS=%f"), i, Contact.Constraints[2].RHS);
				Contact.Constraints[2].LowerLimit = 0.0f;
				Contact.Constraints[2].UpperLimit = 0.0f;
			}
		}
	}

	// �E�H�[���X�^�[�g
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
				SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(RotatedPointA ^ Constraint.Axis));
				SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
				SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(RotatedPointB ^ Constraint.Axis));
			}
		}
	}

	// �R���X�g���C���g�̔������Z
	for (int32 Itr = 0; Itr < NumIterations; Itr++)
	{
		for (FContactPair& ContactPair : ContactPairs)
		{
			FSolverBody& SolverBodyA = SolverBodies[ContactPair.RigidBodyA_Idx];
			FSolverBody& SolverBodyB = SolverBodies[ContactPair.RigidBodyB_Idx];

			for (int32 i = 0; i < ContactPair.NumContact; ++i)
			{
				FContact& Contact = ContactPair.Contacts[i];

				const FVector& RotatedPointA = SolverBodyA.Orientation * Contact.ContactPointA;
				const FVector& RotatedPointB = SolverBodyB.Orientation * Contact.ContactPointB;
				UE_LOG(LogTemp, Log, TEXT("RotatedPointB i=%d, (%f, %f, %f)"), i, RotatedPointB.X, RotatedPointB.Y, RotatedPointB.Z);

				// Normal
				{
					FConstraint& Constraint = Contact.Constraints[0];
					float DeltaImpulse = Constraint.RHS;
					const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + (SolverBodyA.DeltaAngularVelocity ^ RotatedPointA);
					const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + (SolverBodyB.DeltaAngularVelocity ^ RotatedPointB);
					UE_LOG(LogTemp, Log, TEXT("Normal DeltaVelocityB i=%d, (%f, %f, %f)"), i, DeltaVelocityB.X, DeltaVelocityB.Y, DeltaVelocityB.Z);
					DeltaImpulse -= Constraint.JacobianDiagInv * (Constraint.Axis | (DeltaVelocityA - DeltaVelocityB));
					UE_LOG(LogTemp, Log, TEXT("Normal i=%d, DeltaImpuse=%f"), i, DeltaImpulse);

					float OldImpulse = Constraint.AccumImpulse;
					Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
					DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

					SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
					SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(RotatedPointA ^ Constraint.Axis));
					SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
					UE_LOG(LogTemp, Log, TEXT("Normal SolverBodyB.DeltaLinearVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaLinearVelocity.X, SolverBodyB.DeltaLinearVelocity.Y, SolverBodyB.DeltaLinearVelocity.Z);
					SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(RotatedPointB ^ Constraint.Axis));
					UE_LOG(LogTemp, Log, TEXT("Normal SolverBodyB.DeltaAngularVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaAngularVelocity.X, SolverBodyB.DeltaAngularVelocity.Y, SolverBodyB.DeltaAngularVelocity.Z);
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
					const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + (SolverBodyA.DeltaAngularVelocity ^ RotatedPointA);
					const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + (SolverBodyB.DeltaAngularVelocity ^ RotatedPointB);
					UE_LOG(LogTemp, Log, TEXT("Tangent1 DeltaVelocityB i=%d, (%f, %f, %f)"), i, DeltaVelocityB.X, DeltaVelocityB.Y, DeltaVelocityB.Z);
					DeltaImpulse -= Constraint.JacobianDiagInv * (Constraint.Axis | (DeltaVelocityA - DeltaVelocityB));
					UE_LOG(LogTemp, Log, TEXT("Tangent1 i=%d, DeltaImpuse=%f"), i, DeltaImpulse);

					float OldImpulse = Constraint.AccumImpulse;
					Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
					DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

					SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
					SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(RotatedPointA ^ Constraint.Axis));
					SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
					UE_LOG(LogTemp, Log, TEXT("Tangent1 SolverBodyB.DeltaLinearVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaLinearVelocity.X, SolverBodyB.DeltaLinearVelocity.Y, SolverBodyB.DeltaLinearVelocity.Z);
					SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(RotatedPointB ^ Constraint.Axis));
					UE_LOG(LogTemp, Log, TEXT("Tangent1 SolverBodyB.DeltaAngularVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaAngularVelocity.X, SolverBodyB.DeltaAngularVelocity.Y, SolverBodyB.DeltaAngularVelocity.Z);
				}

				// Tangent2
				{
					FConstraint& Constraint = Contact.Constraints[2];
					float DeltaImpulse = Constraint.RHS;
					const FVector& DeltaVelocityA = SolverBodyA.DeltaLinearVelocity + (SolverBodyA.DeltaAngularVelocity ^ RotatedPointA);
					const FVector& DeltaVelocityB = SolverBodyB.DeltaLinearVelocity + (SolverBodyB.DeltaAngularVelocity ^ RotatedPointB);
					UE_LOG(LogTemp, Log, TEXT("Tangent2 DeltaVelocityB i=%d, (%f, %f, %f)"), i, DeltaVelocityB.X, DeltaVelocityB.Y, DeltaVelocityB.Z);
					DeltaImpulse -= Constraint.JacobianDiagInv * (Constraint.Axis | (DeltaVelocityA - DeltaVelocityB));
					UE_LOG(LogTemp, Log, TEXT("Tangent2 i=%d, DeltaImpuse=%f"), i, DeltaImpulse);

					float OldImpulse = Constraint.AccumImpulse;
					Constraint.AccumImpulse = FMath::Clamp(OldImpulse + DeltaImpulse, Constraint.LowerLimit, Constraint.UpperLimit);
					DeltaImpulse = Constraint.AccumImpulse - OldImpulse;

					SolverBodyA.DeltaLinearVelocity += DeltaImpulse * SolverBodyA.MassInv * Constraint.Axis;
					SolverBodyA.DeltaAngularVelocity += DeltaImpulse * FVector(SolverBodyA.InertiaInv.TransformVector(RotatedPointA ^ Constraint.Axis));
					SolverBodyB.DeltaLinearVelocity -= DeltaImpulse * SolverBodyB.MassInv * Constraint.Axis;
					UE_LOG(LogTemp, Log, TEXT("Tangent2 SolverBodyB.DeltaLinearVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaLinearVelocity.X, SolverBodyB.DeltaLinearVelocity.Y, SolverBodyB.DeltaLinearVelocity.Z);
					SolverBodyB.DeltaAngularVelocity -= DeltaImpulse * FVector(SolverBodyB.InertiaInv.TransformVector(RotatedPointB ^ Constraint.Axis));
					UE_LOG(LogTemp, Log, TEXT("Tangent2 SolverBodyB.DeltaAngularVelocity i=%d, (%f, %f, %f)"), i, SolverBodyB.DeltaAngularVelocity.X, SolverBodyB.DeltaAngularVelocity.Y, SolverBodyB.DeltaAngularVelocity.Z);
				}
			}
		}
	}

	// ���x���X�V
	for (int32 i = 0; i < NumRigidBodies + 1; i++)
	{
		// TODO:�t���A��Static��������̂��Ƃ肠�����X�L�b�v�ōs��
		if (i == 0)
		{
			continue;
		}

		RigidBodies[i].LinearVelocity += SolverBodies[i].DeltaLinearVelocity;
		RigidBodies[i].AngularVelocity += SolverBodies[i].DeltaAngularVelocity;
	}

	UE_LOG(LogTemp, Log, TEXT("=======================================================")); // ���s
}

void ARigidBodiesCustomMesh::Integrate(int32 RBIdx, float DeltaSeconds)
{
	// TODO:�t���A��Static��������̂��Ƃ肠����Integrate�̃X�L�b�v�ōs��
	if (RBIdx == 0)
	{
		return;
	}

	RigidBodies[RBIdx].LinearVelocity += FVector(0.0f, 0.0f, Gravity) * DeltaSeconds;
	RigidBodies[RBIdx].Position += RigidBodies[RBIdx].LinearVelocity * DeltaSeconds;

	const FQuat& OrientationDifferential = FQuat(RigidBodies[RBIdx].AngularVelocity.X, RigidBodies[RBIdx].AngularVelocity.Y, RigidBodies[RBIdx].AngularVelocity.Z, 0.0f) * RigidBodies[RBIdx].Orientation * 0.5f;
	RigidBodies[RBIdx].Orientation = (RigidBodies[RBIdx].Orientation + OrientationDifferential * DeltaSeconds).GetNormalized();
}

void ARigidBodiesCustomMesh::ApplyRigidBodiesToMeshDrawing()
{
	// CustomMeshComponent�̃��b�V���̐ݒ�
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
	// �O�t���[����New���邢��Keep���������̂�NoContact�ɂ��邩Keep�ɂ��邩���肷��
	// NoContacr�̂��̂�New�ɂ���̂�AddContact()�̖���
	static const float CONTACT_THRESHOLD_NORMAL = 1.0f;
	static const float CONTACT_SQ_THRESHOLD_TANGENT = 5.0f;

	switch (State)
	{
	case EContactPairState::NoContact:
		// �������Ȃ�
		break;
	case EContactPairState::New:
	case EContactPairState::Keep:
		{
			// �����R���^�N�g�|�C���g���R���^�N�g�������Ă��邩�`�F�b�N���A���ĂȂ���΍폜
			for (int32 i = 0; i < NumContact; ++i)
			{
				const FVector& Normal = Contacts[i].Normal;
				const FVector& ContactPointA = PositionA + OrientationA * Contacts[i].ContactPointA;
				const FVector& ContactPointB = PositionB + OrientationB * Contacts[i].ContactPointB;

				// �ђʐ[�x�̍X�V�B�v���X�ɂȂ�臒l�𒴂�����R���^�N�g�|�C���g�폜�B
				float PenetrationDepth = Normal | (ContactPointA - ContactPointB);
				if (PenetrationDepth > CONTACT_THRESHOLD_NORMAL)
				{
					RemoveContact(i);
					i--;
					continue;
				}
				Contacts[i].PenetrationDepth = PenetrationDepth;

				// �[�x��������������������臒l�𒴂�����R���^�N�g�|�C���g�폜�B
				float TangentDistance = ((ContactPointA - PenetrationDepth * Normal) - ContactPointB).SizeSquared();
				if (TangentDistance > CONTACT_SQ_THRESHOLD_TANGENT)
				{
					RemoveContact(i);
					i--;
					continue;
				}
			}

			// �X�V���ꂽ�R���^�N�g�������ԍX�V
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
	// AddContact����New�Ȃ̂�Keep�Ȃ̂����肷��

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
			if (Idx < 0) // �����Ƃ݂Ȃ���R���^�N�g�|�C���g���Ȃ������Ƃ�
			{
				if (NumContact < 4) // TODO:�}�W�b�N�i���o�[
				{
					// �Ō�̗v�f�ɒǉ�
					Idx = NumContact;
					NumContact++;
				}
				else
				{
					// �v�f��I��Ō����B
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
		// New�ɂ���̂�AddContact()���݂̂Ȃ̂ł�����New�ɂȂ��Ă���̂͂��肦�Ȃ�
	default:
		check(false);
		break;
	}
}

void ARigidBodiesCustomMesh::FContactPair::RemoveContact(int32 Index)
{
	// �Ō�̗v�f�������Ă���B�\�[�g��AddContact()�ŕʓr�s���B
	Contacts[Index] = Contacts[NumContact - 1];
	NumContact--;
}

int32 ARigidBodiesCustomMesh::FContactPair::FindNearestContact(const FVector& ContactPointA, const FVector& ContactPointB, const FVector& Normal)
{
	static const float CONTACT_SAME_POINT_SQ_THRESHOLD = 1.0f; // TODO:EasyPhysics�ł�MKS�P�ʌn��0.01�ɂ��Ă��邪���ꂾ��2��łȂ�������10cm�Ȃ̂ő傫������Ǝv��
	int32 NearestIdx = -1;

	float MinDiff = CONTACT_SAME_POINT_SQ_THRESHOLD;
	for (int32 i = 0; i < NumContact; ++i)
	{
		float DiffA = (ContactPointA - Contacts[i].ContactPointA).SizeSquared();
		float DiffB = (ContactPointB - Contacts[i].ContactPointB).SizeSquared();
		if (DiffA < MinDiff && DiffB < MinDiff && (Normal | Contacts[i].Normal) > 0.99f)
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
		// 4�_�Ō��܂�ʐς̒��ōő�̂��̂�I��
		float AreqSqA = ((P0 - P1) ^ (P2 - P3)).SizeSquared();
		float AreqSqB = ((P0 - P2) ^ (P1 - P3)).SizeSquared();
		float AreqSqC = ((P0 - P3) ^ (P1 - P2)).SizeSquared();
		return FMath::Max(FMath::Max(AreqSqA, AreqSqB), AreqSqC);
	}
}

int32 ARigidBodiesCustomMesh::FContactPair::ChooseSwapContact(const FVector& NewPointA, float NewPenetrationDepth)
{
	check(NumContact == 4);

	int32 MaxPenetrationIdx = -1;
	float MaxPenetration = NewPenetrationDepth;

	// �ł��[���Փ˓_�͔r���Ώۂ���O��
	for (int32 i = 0; i < NumContact; ++i)
	{
		if (Contacts[i].PenetrationDepth < MaxPenetration)
		{
			MaxPenetrationIdx = i;
			MaxPenetration = Contacts[i].PenetrationDepth;
		}
	}

	// �e�_���������Ƃ��̏Փ˓_�����ʐς̂����ő�ɂȂ���̂�I��
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

