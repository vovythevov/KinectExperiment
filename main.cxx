#include <windows.h> // WINDOWS MUST BE BEFORE !

// -- Kinect --
#include <NuiApi.h>

// -- STD --
#include <iostream>

// -- VTK --
#include <vtkActor2D.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkImageReslice.h>
#include <vtkImageWriter.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>

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
  if (!sensor)
    {
    return false;
    }

  return sensor->NuiSkeletonTrackingEnable(readyEvent, 0) != EXIT_SUCCESS;
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
    std::cout<<"Could not Init skeleton Stream"<<std::endl;
    return EXIT_FAILURE;
    }


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

  vtkSmartPointer<vtkActor2D> skeletonImageActor =
    vtkSmartPointer<vtkActor2D>::New();
  skeletonImageActor->SetMapper(depthImageMapper);
  skeletonImageActor->SetPosition(SkeletonWidth, 0);

  // Setup renderers
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(ColorWidth + DepthWidth + SkeletonWidth, ColorHeight);

  renderer->AddActor2D(colorImageActor);
  renderer->AddActor2D(depthImageActor);
  renderer->AddActor2D(skeletonImageActor);

  vtkNew<vtkImageFlip> flipColor;
  flipColor->SetFilteredAxis(1);

  vtkNew<vtkImageFlip> flipDepth;
  flipDepth->SetFilteredAxis(1);

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
      }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(nextDepthFrameEvent, 0) )
      {
      flipDepth->SetInputData(GrabNextDepthFrame());
      flipDepth->Update();
      depthImageMapper->SetInputData(flipDepth->GetOutput());

      renderWindow->Render();
      }

    }

  return EXIT_SUCCESS;
 }