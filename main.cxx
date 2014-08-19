#include <windows.h> // WINDOWS MUST BE BEFORE !

// -- Kinect --
#include <NuiApi.h>

// -- STD --
#include <iostream>

// -- VTK --
#include <vtkActor2D.h>
#include <vtkAppendPolyData.h>
#include <vtkDataObject.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkImageReslice.h>
#include <vtkImageWriter.h>
#include <vtkLineSource.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkPolyDataWriter.h>
#include <vtkPNGWriter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkWindowToImageFilter.h>

// -- Bender --
#include "vtkArmatureWidget.h"
#include "vtkBoneWidget.h"
#include "vtkDoubleConeBoneRepresentation.h"

// Places from where I glanced/used/copy some code from:
// http://www.cs.princeton.edu/~edwardz/tutorials/kinect/kinect1.html
// http://code.google.com/p/visual-experiments/source/browse/trunk/Demos/?r=45#Demos%2FOgreKinect%253Fstate%253Dclosed
// http://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=0CC8QFjAA&url=http%3A%2F%2Fdownload.microsoft.com%2Fdownload%2FF%2F9%2F9%2FF99791F2-D5BE-478A-B77A-830AD14950C3%2FSkeletalViewer_Walkthrough.pdf&ei=OqdkUcLLBYGaqQHogIGACw&usg=AFQjCNFwPBiIHqCPoI7MgCarD3FUIfZXLQ&sig2=aml8RU0vnvlbJiUHLu2oxA&bvm=bv.44990110,d.aWM&cad=rja

//-----------------------------------------------------------------------------
// STATIC Variables

// Picture size
unsigned int ColorWidth = 640; // resolution could be deduced from kinect frame
unsigned int ColorHeight = 480;

unsigned int DepthWidth = 320; // resolution could be deduced from kinect frame
unsigned int DepthHeight = 240;

unsigned int SkeletonWidth = 640; // resolution could be deduced from kinect frame
unsigned int SkeletonHeight = 480;



// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
HANDLE depthStream;            // The identifier of the Kinect's depth Camera
INuiSensor* sensor;            // The kinect sensor

//-----------------------------------------------------------------------------
bool InitKinect()
{
  // Get a working kinect sensor
  int numSensors;
  if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
    {
    return false;
    }
  if (NuiCreateSensorByIndex(0, &sensor) < 0)
    {
    return false;
    }

  // Initialize sensor
  sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR);
  return sensor;
}

//-----------------------------------------------------------------------------
bool OpenColorStream(HANDLE readyEvent)
{
  if (!sensor)
    {
    return false;
    }

  // Open Stream
  return sensor->NuiImageStreamOpen(
    NUI_IMAGE_TYPE_COLOR, //hmm I guess
    NUI_IMAGE_RESOLUTION_640x480, // resolution
    NUI_IMAGE_DEPTH_NO_VALUE, // looks like this controls how far you want to see
    2, // The doc says "Most application should use 2"
    readyEvent,
    &rgbStream) == EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
bool OpenDepthStream(HANDLE readyEvent)
{
  if (!sensor)
    {
    return false;
    }

  // Open Stream
  return sensor->NuiImageStreamOpen(
    NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, //hmm I guess
   NUI_IMAGE_RESOLUTION_320x240, // resolution, maybe smaller ?
    NUI_IMAGE_DEPTH_NO_VALUE, // looks like this controls how far you want to see
    2, // The doc says "Most application should use 2"
    readyEvent,
    &depthStream) == EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
bool OpenSkeletonStream(HANDLE readyEvent)
{
  if (!sensor || !HasSkeletalEngine(sensor))
    {
    return false;
    }

  return sensor->NuiSkeletonTrackingEnable(readyEvent, 0) == EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
vtkImageData* GrabNextColorFrame()
{
  NUI_IMAGE_FRAME kinectFrame;
  if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &kinectFrame) != EXIT_SUCCESS)
    {
    return 0;
    }

  INuiFrameTexture* texture = kinectFrame.pFrameTexture;
  NUI_LOCKED_RECT lockedRect;
  texture->LockRect(0, &lockedRect, NULL, 0);

  /*std::cout<<"Image Dimension: "
    <<"Number of bytes in a row: "<<lockedRect.Pitch<<std::endl
    <<"Size of PBytes: "<<lockedRect.size<<std::endl;*/

  // Color image is 32-bits-per-pixel RGB

  vtkImageData* image = vtkImageData::New();
  image->SetDimensions(ColorWidth, ColorHeight, 1);
  image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

  const BYTE* kinectImage = const_cast<const BYTE*>(lockedRect.pBits);
  for (unsigned int y = 0; y < ColorHeight; ++y)
    {
    for (unsigned int x = 0; x < ColorWidth; ++x)
      {
      unsigned char* pixel =
        static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
      for (int rgb = 0; rgb < 3; ++rgb)
        {
        pixel[rgb] = *kinectImage;

        ++kinectImage;
        }

      unsigned char c = pixel[0];
      pixel[0] = pixel[2];
      pixel[2] = c;

      ++kinectImage;
      }
    }

  texture->UnlockRect(0);
  sensor->NuiImageStreamReleaseFrame(rgbStream, &kinectFrame);

  return image;
}

//-----------------------------------------------------------------------------
vtkImageData* GrabNextDepthFrame()
{
  NUI_IMAGE_FRAME kinectFrame;
  if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &kinectFrame) != EXIT_SUCCESS)
    {
    return 0;
    }

  INuiFrameTexture* texture = kinectFrame.pFrameTexture;
  NUI_LOCKED_RECT lockedRect;
  texture->LockRect(0, &lockedRect, NULL, 0);

  /*std::cout<<"Image Dimension: "
    <<"Number of bytes in a row: "<<lockedRect.Pitch<<std::endl
    <<"Size of PBytes: "<<lockedRect.size<<std::endl;*/

  // Depth image is 16-bit value in which the low-order 12 bits (bits 0–11)
  // contain the depth value in millimeters

  vtkImageData* image = vtkImageData::New();
  image->SetDimensions(DepthWidth, DepthHeight, 1);
  image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

  const short* kinectImage = reinterpret_cast<short*>(lockedRect.pBits);
  for (unsigned int y = 0; y < DepthHeight; ++y)
    {
    for (unsigned int x = 0; x < DepthWidth; ++x)
      {
      unsigned char* pixel =
        static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));

      short depth = *kinectImage;
      short realDepth = (depth & 0xfff8) >> 3;
      short player = depth & 7;
      char scale = 255 - (BYTE)(256 * realDepth / 0x0fff);

      switch (player)
        {
        case 0:
          {
          pixel[0] = scale / 2;
          pixel[1] = scale / 2;
          pixel[2] = scale / 2;
          break;
          }
        case 1:
          {
          pixel[0] = scale;
          break;
          }
        case 2:
          {
          pixel[1] = scale;
          break;
          }
        case 3:
          {
          pixel[2] = scale;
          break;
          }
        case 4:
          {
          pixel[0] = scale;
          pixel[1] = scale;
          break;
          }
        case 5:
          {
          pixel[0] = scale;
          pixel[2] = scale;
          break;
          }
        case 6:
          {
          pixel[1] = scale;
          pixel[2] = scale;
          break;
          }
        case 7:
          {
          pixel[0] = 255 - scale / 2;
          pixel[1] = 255 - scale / 2;
          pixel[2] = 255 - scale / 2;
          break;
          }
        }

      ++kinectImage;
      }
    }


  texture->UnlockRect(0);
  sensor->NuiImageStreamReleaseFrame(depthStream, &kinectFrame);

  return image;
}

namespace 
{

void ApplyTransformToBone(vtkBoneWidget* bone, double displayPos1[2], double displayPos2[2], double dist)
{
  //std::cout<<"displayPos1: "<<displayPos1[0]<<" "<<displayPos1[1]<<std::endl;
  //std::cout<<"displayPos2: "<<displayPos2[0]<<" "<<displayPos2[1]<<std::endl;

  double lineVect[3];
  lineVect[0] = 0;
  lineVect[1] = displayPos2[0] - displayPos1[0];
  lineVect[2] = displayPos2[1] - displayPos1[1];
  vtkMath::Normalize(lineVect);

  //std::cout<<"Line Vect: "<<lineVect[0]<<" "<<lineVect[1]<<" "<<lineVect[2]<<std::endl;

  double head[3];
  bone->GetWorldHeadRest(head);
  double tail[3];
  
  //std::cout<<"Tail: "<<tail[0]<<" "<<tail[1]<<" "<<tail[2]<<std::endl;

  vtkMath::MultiplyScalar(lineVect, dist);
  vtkMath::Add(head, lineVect, tail);
  
  //std::cout<<"New Tail: "<<tail[0]<<" "<<tail[1]<<" "<<tail[2]<<std::endl;
  bone->SetWorldTailRest(tail);
}
/*
ApplyTransformToBone(armature->GetBoneByName("HipLeft"),
      displayPos[NUI_SKELETON_POSITION_HIP_CENTER],
      displayPos[NUI_SKELETON_POSITION_HIP_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("KneeLeft"),
      displayPos[NUI_SKELETON_POSITION_HIP_LEFT],
      displayPos[NUI_SKELETON_POSITION_KNEE_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("AnkleLeft"),
      displayPos[NUI_SKELETON_POSITION_KNEE_LEFT],
      displayPos[NUI_SKELETON_POSITION_ANKLE_LEFT]);

    ApplyTransformToBone(armature->GetBoneByName("FootLeft"),
      displayPos[NUI_SKELETON_POSITION_ANKLE_LEFT],
      displayPos[NUI_SKELETON_POSITION_FOOT_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("HipRight"),
      displayPos[NUI_SKELETON_POSITION_HIP_CENTER],
      displayPos[NUI_SKELETON_POSITION_HIP_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("KneeRight"),
      displayPos[NUI_SKELETON_POSITION_HIP_RIGHT],
      displayPos[NUI_SKELETON_POSITION_KNEE_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("AnkleRight"),
      displayPos[NUI_SKELETON_POSITION_KNEE_RIGHT],
      displayPos[NUI_SKELETON_POSITION_ANKLE_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("FootRight"),
      displayPos[NUI_SKELETON_POSITION_ANKLE_RIGHT],
      displayPos[NUI_SKELETON_POSITION_FOOT_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("Spine"),
      displayPos[NUI_SKELETON_POSITION_HIP_CENTER],
      displayPos[NUI_SKELETON_POSITION_SPINE]);
    ApplyTransformToBone(armature->GetBoneByName("ShoulderCenter"),
      displayPos[NUI_SKELETON_POSITION_SPINE],
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER]);
    ApplyTransformToBone(armature->GetBoneByName("Head"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      displayPos[NUI_SKELETON_POSITION_HEAD]);
    ApplyTransformToBone(armature->GetBoneByName("ShoulderLeft"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      displayPos[NUI_SKELETON_POSITION_SHOULDER_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("ElbowLeft"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_LEFT],
      displayPos[NUI_SKELETON_POSITION_ELBOW_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("WristLeft"),
      displayPos[NUI_SKELETON_POSITION_ELBOW_LEFT],
      displayPos[NUI_SKELETON_POSITION_WRIST_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("HandLeft"),
      displayPos[NUI_SKELETON_POSITION_WRIST_LEFT],
      displayPos[NUI_SKELETON_POSITION_HAND_LEFT]);
    ApplyTransformToBone(armature->GetBoneByName("ShoulderRight"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      displayPos[NUI_SKELETON_POSITION_SHOULDER_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("ElbowRight"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
      displayPos[NUI_SKELETON_POSITION_ELBOW_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("WristRight"),
      displayPos[NUI_SKELETON_POSITION_ELBOW_RIGHT],
      displayPos[NUI_SKELETON_POSITION_WRIST_RIGHT]);
    ApplyTransformToBone(armature->GetBoneByName("HandRight"),
      displayPos[NUI_SKELETON_POSITION_WRIST_RIGHT],
      displayPos[NUI_SKELETON_POSITION_HAND_RIGHT]);*/
}

//-----------------------------------------------------------------------------
//vtkPolyData* GrabNextSkeletonFrame(vtkArmatureWidget* armature)
vtkPolyData* GrabNextSkeletonFrame(std::string filename)
{
  NUI_SKELETON_FRAME SkeletonFrame;
  if (NuiSkeletonGetNextFrame(0, &SkeletonFrame) != EXIT_SUCCESS)
    {
    return 0;
    }

  bool foundSkeleton = false;
  for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
    {
    if (SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED)
      {
       foundSkeleton = true;
      }
    }

  if (! foundSkeleton)
    {
      std::cout<<"No Skeleton Found !"<<std::endl;
    return 0;
    }

  vtkArmatureWidget* armature = 0;
  NuiTransformSmooth(&SkeletonFrame, NULL);
  for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
    {
    const NUI_SKELETON_DATA& skeletonData = SkeletonFrame.SkeletonData[i];
    if (skeletonData.eTrackingState != NUI_SKELETON_TRACKED)
      {
      continue;
      }

    double displayPos[20][2];
    for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
      {
      float fx, fy;
      NuiTransformSkeletonToDepthImage(
              SkeletonFrame.SkeletonData[i].SkeletonPositions[j], &fx, &fy);
      displayPos[j][0] = (int) (fx * SkeletonWidth + 0.5f);
      displayPos[j][1] = (int) (fy * SkeletonHeight + 0.5f);
      }

    vtkNew<vtkPolyDataReader> reader;
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkPolyData* referenceArmature = reader->GetOutput();
    if (!referenceArmature)
      {
      std::cout<<"No Reference armature !"<<std::endl;
      return 0;
      }

    vtkPoints* points = referenceArmature->GetPoints();
    if (!points)
      {
      std::cerr<<"Cannot create armature from ref,"
        <<" No points !"<<std::endl;
      return 0;
      }

     armature = vtkArmatureWidget::New();

    double p[3];
    double p2[3];
    double d[3];
    double dist;
    vtkBoneWidget* Root = armature->CreateBone(NULL, "Root");
    armature->AddBone(Root, NULL);
    points->GetPoint(0, p);
    Root->SetWorldHeadRest(p);
    points->GetPoint(1, p);
    Root->SetWorldTailRest(p);

    // Left leg
    vtkBoneWidget* HipLeft = armature->CreateBone(Root, "HipLeft");
    armature->AddBone(HipLeft, Root);
    points->GetPoint(3, p);
    points->GetPoint(2, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    HipLeft->SetWorldTailRest(p);

    ApplyTransformToBone(HipLeft,
      displayPos[NUI_SKELETON_POSITION_HIP_CENTER],
      displayPos[NUI_SKELETON_POSITION_HIP_LEFT],
      dist);

    vtkBoneWidget* KneeLeft = armature->CreateBone(HipLeft, "KneeLeft");
    armature->AddBone(KneeLeft, HipLeft);
    points->GetPoint(5, p);
    points->GetPoint(4, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    KneeLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("KneeLeft"),
      displayPos[NUI_SKELETON_POSITION_HIP_LEFT],
      displayPos[NUI_SKELETON_POSITION_KNEE_LEFT],
      dist);

    vtkBoneWidget* AnkleLeft = armature->CreateBone(KneeLeft, "AnkleLeft");
    armature->AddBone(AnkleLeft, KneeLeft);
    points->GetPoint(7, p);
    points->GetPoint(6, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    AnkleLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("AnkleLeft"),
      displayPos[NUI_SKELETON_POSITION_KNEE_LEFT],
      displayPos[NUI_SKELETON_POSITION_ANKLE_LEFT],
      dist);

    vtkBoneWidget* FootLeft = armature->CreateBone(AnkleLeft, "FootLeft");
    armature->AddBone(FootLeft, AnkleLeft);
    points->GetPoint(9, p);
    points->GetPoint(8, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    FootLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("FootLeft"),
      displayPos[NUI_SKELETON_POSITION_ANKLE_LEFT],
      displayPos[NUI_SKELETON_POSITION_FOOT_LEFT],
      dist);

    // Right leg
    vtkBoneWidget* HipRight = armature->CreateBone(Root, "HipRight");
    armature->AddBone(HipRight, Root);
    points->GetPoint(11, p);
    points->GetPoint(10, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    HipRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("HipRight"),
      displayPos[NUI_SKELETON_POSITION_HIP_CENTER],
      displayPos[NUI_SKELETON_POSITION_HIP_RIGHT],
      dist);

    vtkBoneWidget* KneeRight = armature->CreateBone(HipRight, "KneeRight");
    armature->AddBone(KneeRight, HipRight);
    points->GetPoint(13, p);
    points->GetPoint(12, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    KneeRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("KneeRight"),
      displayPos[NUI_SKELETON_POSITION_HIP_RIGHT],
      displayPos[NUI_SKELETON_POSITION_KNEE_RIGHT],
      dist);

    vtkBoneWidget* AnkleRight = armature->CreateBone(KneeRight, "AnkleRight");
    armature->AddBone(AnkleRight, KneeRight);
    points->GetPoint(15, p);
    points->GetPoint(14, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    AnkleRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("AnkleRight"),
      displayPos[NUI_SKELETON_POSITION_KNEE_RIGHT],
      displayPos[NUI_SKELETON_POSITION_ANKLE_RIGHT],
      dist);

    vtkBoneWidget* FootRight = armature->CreateBone(AnkleRight, "FootRight");
    armature->AddBone(FootRight, AnkleRight);
    points->GetPoint(17, p);
    points->GetPoint(16, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    FootRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("FootRight"),
      displayPos[NUI_SKELETON_POSITION_ANKLE_RIGHT],
      displayPos[NUI_SKELETON_POSITION_FOOT_RIGHT],
      dist);

    // spine
    vtkBoneWidget* Spine = armature->CreateBone(Root, "Spine");
    armature->AddBone(Spine, Root);
    points->GetPoint(19, p);
    points->GetPoint(18, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    Spine->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("Spine"),
      displayPos[NUI_SKELETON_POSITION_HIP_CENTER],
      displayPos[NUI_SKELETON_POSITION_SPINE],
      dist);

    vtkBoneWidget* ShoulderCenter = armature->CreateBone(Spine, "ShoulderCenter");
    armature->AddBone(ShoulderCenter, Spine);
    points->GetPoint(21, p);
    points->GetPoint(20, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    ShoulderCenter->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("ShoulderCenter"),
      displayPos[NUI_SKELETON_POSITION_SPINE],
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      dist);

    vtkBoneWidget* Head = armature->CreateBone(ShoulderCenter, "Head");
    armature->AddBone(Head, ShoulderCenter);
    points->GetPoint(23, p);
    points->GetPoint(22, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    Head->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("Head"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      displayPos[NUI_SKELETON_POSITION_HEAD],
      dist);

    // Left Arm
    vtkBoneWidget* ShoulderLeft = armature->CreateBone(ShoulderCenter, "ShoulderLeft");
    armature->AddBone(ShoulderLeft, ShoulderCenter);
    points->GetPoint(25, p);
    points->GetPoint(24, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    ShoulderLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("ShoulderLeft"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      displayPos[NUI_SKELETON_POSITION_SHOULDER_LEFT],
      dist);

    vtkBoneWidget* ElbowLeft = armature->CreateBone(ShoulderLeft, "ElbowLeft");
    armature->AddBone(ElbowLeft, ShoulderLeft);
    points->GetPoint(27, p);
    points->GetPoint(26, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    ElbowLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("ElbowLeft"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_LEFT],
      displayPos[NUI_SKELETON_POSITION_ELBOW_LEFT],
      dist);

    vtkBoneWidget* WristLeft = armature->CreateBone(ElbowLeft, "WristLeft");
    armature->AddBone(WristLeft, ElbowLeft);
    points->GetPoint(29, p);
    points->GetPoint(28, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    WristLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("WristLeft"),
      displayPos[NUI_SKELETON_POSITION_ELBOW_LEFT],
      displayPos[NUI_SKELETON_POSITION_WRIST_LEFT],
      dist);

    vtkBoneWidget* HandLeft = armature->CreateBone(WristLeft, "HandLeft");
    armature->AddBone(HandLeft, WristLeft);
    points->GetPoint(31, p);
    points->GetPoint(30, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    HandLeft->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("HandLeft"),
      displayPos[NUI_SKELETON_POSITION_WRIST_LEFT],
      displayPos[NUI_SKELETON_POSITION_HAND_LEFT],
      dist);

    // Right Arm
    vtkBoneWidget* ShoulderRight = armature->CreateBone(ShoulderCenter, "ShoulderRight");
    armature->AddBone(ShoulderRight, ShoulderCenter);
    points->GetPoint(33, p);
    points->GetPoint(32, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    ShoulderRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("ShoulderRight"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_CENTER],
      displayPos[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
      dist);

    vtkBoneWidget* ElbowRight = armature->CreateBone(ShoulderRight, "ElbowRight");
    armature->AddBone(ElbowRight, ShoulderRight);
    points->GetPoint(35, p);
    points->GetPoint(34, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    ElbowRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("ElbowRight"),
      displayPos[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
      displayPos[NUI_SKELETON_POSITION_ELBOW_RIGHT],
      dist);

    vtkBoneWidget* WristRight = armature->CreateBone(ElbowRight, "WristRight");
    armature->AddBone(WristRight, ElbowRight);
    points->GetPoint(37, p);
    points->GetPoint(36, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    WristRight->SetWorldTailRest(p);
   ApplyTransformToBone(armature->GetBoneByName("WristRight"),
      displayPos[NUI_SKELETON_POSITION_ELBOW_RIGHT],
      displayPos[NUI_SKELETON_POSITION_WRIST_RIGHT],
      dist);

    vtkBoneWidget* HandRight = armature->CreateBone(WristRight, "HandRight");
    armature->AddBone(HandRight, WristRight);
    points->GetPoint(39, p);
    points->GetPoint(38, p2);
    vtkMath::Subtract(p2, p, d);
    dist = vtkMath::Norm(d);
    HandRight->SetWorldTailRest(p);
    ApplyTransformToBone(armature->GetBoneByName("HandRight"),
      displayPos[NUI_SKELETON_POSITION_WRIST_RIGHT],
      displayPos[NUI_SKELETON_POSITION_HAND_RIGHT],
      dist);


    // Init pose mode
    armature->SetWidgetState(vtkArmatureWidget::Pose);
    armature->SetWidgetState(vtkArmatureWidget::Rest);

    break;
    }

  return armature->GetPolyData();
}

//-----------------------------------------------------------------------------
vtkArmatureWidget* CreateArmature(std::string& filename)
{
  vtkNew<vtkPolyDataReader> reader;
  reader->SetFileName(filename.c_str());
  reader->Update();

  vtkPolyData* referenceArmature = reader->GetOutput();
  if (!referenceArmature)
    {
    std::cout<<"No Reference armature !"<<std::endl;
    return 0;
    }

  vtkPoints* points = referenceArmature->GetPoints();
  if (!points)
    {
    std::cerr<<"Cannot create armature from ref,"
      <<" No points !"<<std::endl;
    return 0;
    }

  vtkArmatureWidget* armature = vtkArmatureWidget::New();

  double p[3];

  vtkBoneWidget* Root = armature->CreateBone(NULL, "Root");
  armature->AddBone(Root, NULL);
  points->GetPoint(0, p);
  Root->SetWorldHeadRest(p);
  points->GetPoint(1, p);
  Root->SetWorldTailRest(p);

  // Left leg
  vtkBoneWidget* HipLeft = armature->CreateBone(Root, "HipLeft");
  armature->AddBone(HipLeft, Root);
  points->GetPoint(3, p);
  HipLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(HipLeft, false);

  vtkBoneWidget* KneeLeft = armature->CreateBone(HipLeft, "KneeLeft");
  armature->AddBone(KneeLeft, HipLeft);
  points->GetPoint(5, p);
  KneeLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(KneeLeft, false);

  vtkBoneWidget* AnkleLeft = armature->CreateBone(KneeLeft, "AnkleLeft");
  armature->AddBone(AnkleLeft, KneeLeft);
  points->GetPoint(7, p);
  AnkleLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(AnkleLeft, false);

  vtkBoneWidget* FootLeft = armature->CreateBone(AnkleLeft, "FootLeft");
  armature->AddBone(FootLeft, AnkleLeft);
  points->GetPoint(9, p);
  FootLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(FootLeft, false);

  // Right leg
  vtkBoneWidget* HipRight = armature->CreateBone(Root, "HipRight");
  armature->AddBone(HipRight, Root);
  points->GetPoint(11, p);
  HipRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(HipRight, false);

  vtkBoneWidget* KneeRight = armature->CreateBone(HipRight, "KneeRight");
  armature->AddBone(KneeRight, HipRight);
  points->GetPoint(13, p);
  KneeRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(KneeRight, false);

  vtkBoneWidget* AnkleRight = armature->CreateBone(KneeRight, "AnkleRight");
  armature->AddBone(AnkleRight, KneeRight);
  points->GetPoint(15, p);
  AnkleRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(AnkleRight, false);

  vtkBoneWidget* FootRight = armature->CreateBone(AnkleRight, "FootRight");
  armature->AddBone(FootRight, AnkleRight);
  points->GetPoint(17, p);
  FootRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(FootRight, false);

  // spine
  vtkBoneWidget* Spine = armature->CreateBone(Root, "Spine");
  armature->AddBone(Spine, Root);
  points->GetPoint(19, p);
  Spine->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(Spine, false);

  vtkBoneWidget* ShoulderCenter = armature->CreateBone(Spine, "ShoulderCenter");
  armature->AddBone(ShoulderCenter, Spine);
  points->GetPoint(21, p);
  ShoulderCenter->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(ShoulderCenter, false);

  vtkBoneWidget* Head = armature->CreateBone(ShoulderCenter, "Head");
  armature->AddBone(Head, ShoulderCenter);
  points->GetPoint(23, p);
  Head->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(Head, false);

  // Left Arm
  vtkBoneWidget* ShoulderLeft = armature->CreateBone(ShoulderCenter, "ShoulderLeft");
  armature->AddBone(ShoulderLeft, ShoulderCenter);
  points->GetPoint(25, p);
  ShoulderLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(ShoulderLeft, false);

  vtkBoneWidget* ElbowLeft = armature->CreateBone(ShoulderLeft, "ElbowLeft");
  armature->AddBone(ElbowLeft, ShoulderLeft);
  points->GetPoint(27, p);
  ElbowLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(ElbowLeft, false);

  vtkBoneWidget* WristLeft = armature->CreateBone(ElbowLeft, "WristLeft");
  armature->AddBone(WristLeft, ElbowLeft);
  points->GetPoint(29, p);
  WristLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(WristLeft, false);

  vtkBoneWidget* HandLeft = armature->CreateBone(WristLeft, "HandLeft");
  armature->AddBone(HandLeft, WristLeft);
  points->GetPoint(31, p);
  HandLeft->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(HandLeft, false);

  // Left Arm
  vtkBoneWidget* ShoulderRight = armature->CreateBone(ShoulderCenter, "ShoulderRight");
  armature->AddBone(ShoulderRight, ShoulderCenter);
  points->GetPoint(33, p);
  ShoulderRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(ShoulderRight, false);

  vtkBoneWidget* ElbowRight = armature->CreateBone(ShoulderRight, "ElbowRight");
  armature->AddBone(ElbowRight, ShoulderRight);
  points->GetPoint(35, p);
  ElbowRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(ElbowRight, false);

  vtkBoneWidget* WristRight = armature->CreateBone(ElbowRight, "WristRight");
  armature->AddBone(WristRight, ElbowRight);
  points->GetPoint(37, p);
  WristRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(WristRight, false);

  vtkBoneWidget* HandRight = armature->CreateBone(WristRight, "HandRight");
  armature->AddBone(HandRight, WristRight);
  points->GetPoint(39, p);
  HandRight->SetWorldTailRest(p);
  armature->SetBoneLinkedWithParent(HandRight, false);

  // Init pose mode
  armature->SetWidgetState(vtkArmatureWidget::Pose);
  armature->SetWidgetState(vtkArmatureWidget::Rest);

  return armature;
}

//-----------------------------------------------------------------------------
 int main (int argc, char* argv)
 {
  // 1- init
  if (!InitKinect())
    {
    std::cout<<"Could not Init Kinect"<<std::endl;
    return EXIT_FAILURE;
    }

  // 2- Create Events
  const int numberOfEvents = 3;
  HANDLE nextColorFrameEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
  HANDLE nextDepthFrameEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
  HANDLE nextSkeletonFrameEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
  HANDLE events[numberOfEvents] = {nextColorFrameEvent,
                                   nextDepthFrameEvent,
                                   nextSkeletonFrameEvent};

  // 3- Open RGB stream
  if (!OpenColorStream(nextColorFrameEvent))
    {
    std::cout<<"Could not Init Color Stream"<<std::endl;
    return EXIT_FAILURE;
    }

  if (!OpenDepthStream(nextDepthFrameEvent))
    {
    std::cout<<"Could not Init depth Stream"<<std::endl;
    return EXIT_FAILURE;
    }

  if (!OpenSkeletonStream(nextSkeletonFrameEvent))
    {
    std::cout<<"Could not Init skeleton Stream "<< HasSkeletalEngine(sensor) <<std::endl;
    return EXIT_FAILURE;
    }

  std::string filename = "W:/Jungle/Kinect/K_Bender/Armature.vtk";
  vtkSmartPointer<vtkArmatureWidget> armature = CreateArmature(filename);

  vtkNew<vtkPolyDataWriter> w;
  w->SetFileName("W:/Jungle/Kinect/ArmatureTest.vtk");
  w->SetInputData(armature->GetPolyData());
  w->Write();

  // -- VTK Rendering --

  // 1- Color
  vtkSmartPointer<vtkImageMapper> colorImageMapper =
    vtkSmartPointer<vtkImageMapper>::New();
  colorImageMapper->SetColorWindow(255);
  colorImageMapper->SetColorLevel(127.5);

  vtkSmartPointer<vtkActor2D> colorImageActor =
    vtkSmartPointer<vtkActor2D>::New();
  colorImageActor->SetMapper(colorImageMapper);
  colorImageActor->SetPosition(0,0);

  // 2 -Depth
  vtkSmartPointer<vtkImageMapper> depthImageMapper =
    vtkSmartPointer<vtkImageMapper>::New();
  depthImageMapper->SetColorWindow(255);
  depthImageMapper->SetColorLevel(127.5);

  vtkSmartPointer<vtkActor2D> depthImageActor =
    vtkSmartPointer<vtkActor2D>::New();
  depthImageActor->SetMapper(depthImageMapper);
  depthImageActor->SetPosition(ColorWidth,0);

  // 2 -Depth
  vtkSmartPointer<vtkImageMapper> skeletonImageMapper =
    vtkSmartPointer<vtkImageMapper>::New();
  skeletonImageMapper->SetColorWindow(255);
  skeletonImageMapper->SetColorLevel(127.5);

  /**/
  vtkSmartPointer<vtkPolyDataMapper> sMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();

  vtkSmartPointer<vtkActor> sActor =
    vtkSmartPointer<vtkActor>::New();
  sActor->SetMapper(sMapper);
  /**/

  // Setup renderers
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(ColorWidth + DepthWidth, ColorHeight);

  renderer->AddActor2D(colorImageActor);
  renderer->AddActor2D(depthImageActor);
  renderer->AddActor(sActor);

  vtkNew<vtkImageFlip> flipColor;
  flipColor->SetFilteredAxis(1);

  vtkNew<vtkImageFlip> flipDepth;
  flipDepth->SetFilteredAxis(1);

  // -- Armature Rendering --
  w->SetFileName("W:/Jungle/Kinect/Pose.vtk");



  // -- Event Loop --

  bool capture = true;
  while (capture)
    {
    if ( WAIT_OBJECT_0 == WaitForSingleObject(nextColorFrameEvent, 0) )
      {
      flipColor->SetInputData(GrabNextColorFrame());
      flipColor->Update();
      colorImageMapper->SetInputData(flipColor->GetOutput());

      renderWindow->Render();
      }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(nextDepthFrameEvent, 0) )
      {
      flipDepth->SetInputData(GrabNextDepthFrame());
      flipDepth->Update();
      depthImageMapper->SetInputData(flipDepth->GetOutput());

      renderWindow->Render();

      vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = 
        vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
        windowToImageFilter->SetInput(renderWindow);

        vtkSmartPointer<vtkPNGWriter> writer = 
          vtkSmartPointer<vtkPNGWriter>::New();
        windowToImageFilter->Update();
        writer->SetInputConnection(windowToImageFilter->GetOutputPort());
        writer->SetFileName("W:/Jungle/Kinect/NotDetectedBob.png");
        writer->Write();
        
      Sleep(10000);
      }

    /*if ( WAIT_OBJECT_0 == WaitForSingleObject(nextSkeletonFrameEvent, 0) )
      {
      std::cout<<"Grabbing armature"<<std::endl;
      vtkPolyData* newArmature = GrabNextSkeletonFrame( filename );
      if (newArmature)
        {
        w->SetInputData(newArmature);
        w->Write();

        std::cout<<"Wrote !"<<std::endl;

        renderWindow->Render();

        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = 
        vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
        windowToImageFilter->SetInput(renderWindow);

        vtkSmartPointer<vtkPNGWriter> writer = 
          vtkSmartPointer<vtkPNGWriter>::New();
        windowToImageFilter->Update();
        writer->SetInputConnection(windowToImageFilter->GetOutputPort());
        writer->SetFileName("W:/Jungle/Kinect/KinectVovythevov.png");
        writer->Write();

      Sleep(1000);
        }
      }*/

    }

  return EXIT_SUCCESS;
 }