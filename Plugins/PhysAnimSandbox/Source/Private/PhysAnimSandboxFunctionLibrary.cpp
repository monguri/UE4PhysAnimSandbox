#include "PhysAnimSandboxFunctionLibrary.h"
#include "Engine/SkeletalMesh.h"

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

	SkeletalMesh->MarkPackageDirty();
	return true;
}

