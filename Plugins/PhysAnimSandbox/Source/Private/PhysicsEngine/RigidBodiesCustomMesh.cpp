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

	TArray<FVector> BoxVerticesScaled;
	for (const FVector& Vertex : BoxVertices)
	{
		BoxVerticesScaled.Add(Vertex * CubeScale);
	}

	RigidBodies.SetNum(NumRigidBodies);

	// InitPosRadius半径の球内にランダムに配置
	FBoxSphereBounds BoxSphere(InitPosCenter, FVector(InitPosRadius), InitPosRadius);

	for (FRigidBody& RigidBody : RigidBodies)
	{
		RigidBody.CollisionShape.Vertices = BoxVerticesScaled;
		RigidBody.CollisionShape.Edges = BoxEdges;
		RigidBody.CollisionShape.Facets = BoxFacets;

		RigidBody.Position = GetActorLocation() + RandPointInSphere(BoxSphere, InitPosCenter);
		// TODO:とりあえずその他の物理パラメータは初期値のまま
	}

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

void ARigidBodiesCustomMesh::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
}

