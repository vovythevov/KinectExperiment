#include <windows.h> // WINDOWS MUST BE BEFORE !

// -- Kinect --
#include <NuiApi.h>

// -- STD --
#include <iostream>

// -- VTK --
#include <vtkActor2D.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkImageWriter.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>

//-----------------------------------------------------------------------------
// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
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
bool OpenStream()
{
  if (!sensor)
    {
    return false;
    }

  // Open Stream
  return sensor->NuiImageStreamOpen(
    NUI_IMAGE_TYPE_DEPTH, //hmm I guess
    NUI_IMAGE_RESOLUTION_640x480, // resolution
    NUI_IMAGE_DEPTH_NO_VALUE, // looks like this controls how far you want to see
    2, // The doc says "Most application should use 2"
    NULL,  // Next frame event. Maybe I could grab this ???
    &rgbStream) == EXIT_SUCCESS;
}

//-----------------------------------------------------------------------------
 void GrabNextFrame(int numberOfTry, vtkImageData* image)
{
  if (!sensor)
    {
    return;
    }

  // Grab frame
  NUI_IMAGE_FRAME kinectFrame;
  HRESULT result = E_NUI_FRAME_NO_DATA;
  while (numberOfTry >= 0 && result != EXIT_SUCCESS)
    {
    HRESULT result = sensor->NuiImageStreamGetNextFrame(rgbStream, 10, &kinectFrame);
    if (result == EXIT_FAILURE)
      {
      std::cout<<"Could not grab frame ! Error "<<result<<std::endl;
      return;
      }

    --numberOfTry;
    Sleep(10);
    }

  INuiFrameTexture* texture = kinectFrame.pFrameTexture;
  NUI_LOCKED_RECT lockedRect;
  texture->LockRect(0, &lockedRect, NULL, 0);

  int width = 640;
  int height = 480;

  vtkNew<vtkImageImport> importer;
  importer->SetDataExtent(0, width, 0, height, 0, 0);
  importer->SetWholeExtent(0, width, 0, height, 0, 0);
  importer->SetImportVoidPointer(lockedRect.pBits);
  importer->Update();

  image->DeepCopy(importer->GetOutput());

  texture->UnlockRect(0);
  sensor->NuiImageStreamReleaseFrame(rgbStream, &kinectFrame);
}


//-----------------------------------------------------------------------------
 int main (int argc, char* argv)
 {
  std::cout<<"I'm alive, aliiiive ! "<<S_OK<<std::endl;
 
  // 1- init
  std::cout<<"Is there anybody out there ?"<<std::endl;
  if (InitKinect())
    {
    std::cout<<"Yeah ! Found a kinect ! How are you doing ?"<<std::endl;
    std::cout<<sensor->NuiStatus()<<std::endl;
    }
  else
    {
    std::cout<<"Well, I guess I'd better go :("<<std::endl;
    return EXIT_FAILURE;
    }

  // 2 - Open stream
  if (OpenStream())
    {
    std::cout<<"Getting the kinect ready"<<std::endl;
    std::cout<<sensor->NuiStatus()<<std::endl;
    }
  else
    {
    std::cout<<"NOooooooooooooo :("<<std::endl;
    return EXIT_FAILURE;
    }


  // -- VTK Rendering --
  vtkSmartPointer<vtkImageMapper> imageMapper = vtkSmartPointer<vtkImageMapper>::New();
  imageMapper->SetColorWindow(255);
  imageMapper->SetColorLevel(127.5);

  vtkSmartPointer<vtkActor2D> imageActor = vtkSmartPointer<vtkActor2D>::New();
  imageActor->SetMapper(imageMapper);

  // Setup renderers
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
 
  //renderer->AddViewProp(imageActor);
  renderer->AddActor2D(imageActor);

  bool capture = true;
  while (capture)
    {
    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
    GrabNextFrame(100, image);

    imageMapper->SetInputData( image );
 
    renderWindow->Render();
    }

  return EXIT_SUCCESS;
 }