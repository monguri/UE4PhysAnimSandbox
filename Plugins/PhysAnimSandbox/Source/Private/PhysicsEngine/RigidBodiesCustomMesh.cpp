#include "PhysicsEngine/RigidBodiesCustomMesh.h"
#include "UObject/ConstructorHelpers.h"
#include "CustomMeshComponent.h"

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

ARigidBodiesCustomMesh::ARigidBodiesCustomMesh()
{
	PrimaryActorTick.bCanEverTick = true;

	DrawMesh = CreateDefaultSubobject<UCustomMeshComponent>(TEXT("CustomMeshComponent0"));
	RootComponent = DrawMesh;
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
		CubeRigidBody.Position = GetActorLocation() + RandPointInSphere(BoxSphere, InitPosCenter);
		// TODO:�Ƃ肠�������̑��̕����p�����[�^�͏����l�̂܂�
	}

	ContactPairs.Reserve(((NumRigidBodies + 1) * (NumRigidBodies + 2)) / 2); //TODO: �R���^�N�g�y�A�͍ő�ł���������y�A���B�ŏI�I�ɂ͑傫�����邪�Ƃ肠��������ŁB
}

void ARigidBodiesCustomMesh::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSeconds�̒l�̕ϓ��Ɋւ�炸�A�V�~�����[�V�����Ɏg���T�u�X�e�b�v�^�C���͌Œ�Ƃ���
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

	//TODO: �R���^�N�g�y�A�z��Ƀ}���`�X���b�h����A�N�Z�X����̂��댯�Ȃ̂łƂ肠�����V���O���X���b�h
	DetectCollision();

	//TODO: �������̂Ƀ}���`�X���b�h����A�N�Z�X����̂��댯�Ȃ̂łƂ肠�����V���O���X���b�h
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

void ARigidBodiesCustomMesh::DetectCollision()
{
	for (const FRigidBody& RigidBody : RigidBodies)
	{
		;
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
	// TODO:�t���A��Static��������̂��Ƃ肠����Integrate�̃X�L�b�v�ōs��
	if (RBIdx == 0)
	{
		return;
	}

	RigidBodies[RBIdx].LinearVelocity += FVector(0.0f, 0.0f, Gravity) * DeltaSeconds;
	// TODO:���B���U���Ȃ��悤��
	RigidBodies[RBIdx].LinearVelocity.Z = FMath::Max(RigidBodies[RBIdx].LinearVelocity.Z, -1000.0f);

	RigidBodies[RBIdx].Position += RigidBodies[RBIdx].LinearVelocity * DeltaSeconds;
	// TODO:���B���U���Ȃ��悤��
	RigidBodies[RBIdx].Position.Z = FMath::Max(RigidBodies[RBIdx].Position.Z, 25.0f);

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

