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

//-----------------------------------------------------------------------------
// STATIC Variables

// Picture size
unsigned int width = 640; // resolution could be deduced from kinect frame
unsigned int height = 480;

// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
//HANDLE depthStream;            // The identifier of the Kinect's depth Camera
INuiSensor* sensor;            // The kinect sensor

//-----------------------------------------------------------------------------
bool InitKinect() // source: http://www.cs.princeton.edu/~edwardz/tutorials/kinect/kinect1.html
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
  sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
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
    readyEvent,  // Next frame event. Maybe I could grab this ???
    &rgbStream) == EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
bool GrabNextFrame(HANDLE handle, int numberOfTry, NUI_IMAGE_FRAME& kinectFrame)
{
  if (!sensor)
    {
    return false;
    }

  // Grab frame
  HRESULT result = E_NUI_FRAME_NO_DATA;
  while (numberOfTry >= 0 && result != EXIT_SUCCESS)
    {
    HRESULT result = sensor->NuiImageStreamGetNextFrame(handle, 0, &kinectFrame);
    if (result == EXIT_FAILURE)
      {
      std::cout<<"Could not grab frame ! Error "<<result<<std::endl;
      return false;
      }

    --numberOfTry;
    Sleep(1);
    }

  return true;
}

//-----------------------------------------------------------------------------
vtkImageData* GrabNextColorFrame(int numberOfTry)
{
  NUI_IMAGE_FRAME kinectFrame;
  if (! GrabNextFrame(rgbStream, numberOfTry, kinectFrame))
    {
    return 0;
    }

  INuiFrameTexture* texture = kinectFrame.pFrameTexture;
  NUI_LOCKED_RECT lockedRect;
  texture->LockRect(0, &lockedRect, NULL, 0);

  /*std::cout<<"Image Dimension: "
    <<"Number of bytes in a row: "<<lockedRect.Pitch<<std::endl
    <<"Size of PBytes: "<<lockedRect.size<<std::endl;*/

  vtkImageData* image = vtkImageData::New();
  image->SetDimensions(width, height, 1);
  image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

  const BYTE* kinectImage = const_cast<const BYTE*>(lockedRect.pBits);
  for (unsigned int y = 0; y < height; ++y)
    {
    for (unsigned int x = 0; x < width; ++x)
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
 int main (int argc, char* argv)
 {
  // 1- init
  if (!InitKinect())
    {
    std::cout<<"Could not Init Kinect"<<std::endl;
    return EXIT_FAILURE;
    }

  // 2- Create Events
  HANDLE nextColorFrameEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

  // 2 - Open RGB stream
  if (!OpenColorStream(nextColorFrameEvent))
    {
    std::cout<<"Could not Init Color Stream"<<std::endl;
    return EXIT_FAILURE;
    }

  // -- VTK Rendering --

  // 1- Color
  vtkSmartPointer<vtkImageMapper> colorImageMapper =
    vtkSmartPointer<vtkImageMapper>::New();
  colorImageMapper->SetColorWindow(255);
  colorImageMapper->SetColorLevel(127.5);

  vtkSmartPointer<vtkActor2D> colorImageActor = vtkSmartPointer<vtkActor2D>::New();
  colorImageActor->SetMapper(colorImageMapper);
  colorImageActor->SetPosition(0,0);

  // 2 -Depth
  //vtkSmartPointer<vtkImageMapper> depthImageMapper =
  //  vtkSmartPointer<vtkImageMapper>::New();

  //vtkSmartPointer<vtkActor2D> depthImageActor = vtkSmartPointer<vtkActor2D>::New();
  //depthImageActor->SetMapper(depthImageMapper);
  //depthImageActor->SetPosition(width,0);

  // Setup renderers
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(width * 2, height);

  //renderer->AddViewProp(imageActor);
  renderer->AddActor2D(colorImageActor);
  //renderer->AddActor2D(depthImageActor);

  vtkNew<vtkImageFlip> flip;
  flip->SetFilteredAxis(1);

  bool capture = true;
  while (capture)
    {
    if (WAIT_OBJECT_0 == WaitForSingleObject(nextColorFrameEvent, 1))
      {
      flip->SetInputData(GrabNextColorFrame(5));
      flip->Update();
      colorImageMapper->SetInputData(flip->GetOutput());

      renderWindow->Render();
      }

    Sleep(1);
    }

  return EXIT_SUCCESS;
 }