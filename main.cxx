#include <windows.h> // WINDOWS MUST BE BEFORE !

// -- Kinect --
#include <NuiApi.h>

// -- STD --
#include <iostream>

// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
INuiSensor* sensor;            // The kinect sensor

bool initKinect() // source: http://www.cs.princeton.edu/~edwardz/tutorials/kinect/kinect1.html
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

 int main (int argc, char* argv)
 {
  std::cout<<"I'm alive, aliiiive !"<<std::endl;
 
  std::cout<<"Is there anybody out there ?"<<std::endl;
  if (initKinect())
    {
    std::cout<<"Yeah ! Found a kinect ! How are you doing ?"<<std::endl;
    std::cout<<sensor->NuiStatus()<<std::endl;
    }
  else
    {
    return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
 }