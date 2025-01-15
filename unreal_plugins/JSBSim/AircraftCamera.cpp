// Fill out your copyright notice in the Description page of Project Settings.

#include "CameraSim.h"
#include "Runtime/Engine/Classes/Engine/TextureRenderTarget2D.h"
#include "Components/SphereComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "AircraftCamera.h"

// Sets default values
AAircraftCamera::AAircraftCamera()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USphereComponent>(TEXT("Root"));
	Camera = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Camera"));

	ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D> RenderTargetAsset(TEXT("/Script/Engine.TextureRenderTarget2D'/Game/StarterContent/Textures/CameraRenderTarget.CameraRenderTarget'"));

	RenderTarget = DuplicateObject(RenderTargetAsset.Object, NULL);
	RenderTarget->InitAutoFormat(1024, 1024);
	Camera->TextureTarget = RenderTarget;

}


// Called when the game starts or when spawned
void AAircraftCamera::BeginPlay()
{
	Super::BeginPlay();

	Camera->TextureTarget = RenderTarget;

	int X = RenderTarget->GetSurfaceHeight();
	int Y = RenderTarget->GetSurfaceWidth();

	Texture2D = RenderTarget->ConstructTexture2D(this, FString("Tex2D"), EObjectFlags::RF_NoFlags);

	int xx = Texture2D->GetSizeX();
	int yy = Texture2D->GetSizeY();

	int size=X*Y;
	PixelData.AddUninitialized(size);
	
}

bool AAircraftCamera::GetCameraPixelData(){
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	bool bReadSuccess = RenderTargetResource->ReadPixels(PixelData);
	return bReadSuccess;
}

// Called every frame
void AAircraftCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if(GetCameraPixelData()) {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Green, FString::Printf(TEXT("Got Pixel Data")));
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 10.0f, FColor::Red, FString::Printf(TEXT("Could not get pixel data")));
	}

}

