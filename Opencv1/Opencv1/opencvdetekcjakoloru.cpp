#include <iostream> 
#include <stdlib.h>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include "serial_devices.h"
#include "serial_port.h"
#include <string>



using namespace cv;
using namespace std;




// Driver code 
int main()
{

    SERIAL_PORT serial;
    INTERFACE_CLASS enum_ports;
    wstring port_name = L"COM3";
    int sres = serial.open_port(port_name, 9600, 500);

    if (sres < 0)
        return 0;

    int hmin = 0, smin = 73, vmin = 147;
    int hmax = 179, smax = 255, vmax = 255;
    int posX_tymczas = 0;
    int posY_tymczas = 0;

    Mat image;
    Mat image1;
    
    Mat image2, image3, imagewykryty;

    namedWindow("paski_zmian", (640, 200));
    createTrackbar("Hue Min", "paski_zmian", &hmin, 179);
    createTrackbar("saturation Min", "paski_zmian", &smin, 255);
    createTrackbar("value Min", "paski_zmian", &vmin, 255);
    createTrackbar("Hue Max", "paski_zmian", &hmax, 179);
    createTrackbar("saturation Max", "paski_zmian", &smax, 255);
    createTrackbar("value Max", "paski_zmian", &vmax, 255);
    
    
   
    
    VideoCapture Cam(0);
    // Show Image inside a window with
    // the name provided
    namedWindow("camera");
    
    while (true) {



        Cam.read(image);
        cvtColor(image, image1,COLOR_BGR2HSV);
        Scalar dolna(hmin, smin, vmin);
        Scalar gorna(hmax, smax, vmax);

        inRange(image1, dolna, gorna, imagewykryty);
        erode(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        //morphological closing (removes small holes from the foreground)
        dilate(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        erode(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        



        Moments oMoments = moments(imagewykryty);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        if (dArea > 20000)
        {
            //calculate the position of the ball
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;


            ellipse(image, Point(posX, posY), Size(100, 100), 0, 0, 360, Scalar(255, 0, 0), 6);


            if (posX - posX_tymczas > 2 || posY - posY_tymczas > 2);         
            {
                cout << "obracaj w lewo" << endl;
                
                string pozycja = to_string(posX)+","+to_string(posY)+"\n";
                
                int nout = serial.swrite(pozycja);
                
                std::cout << "Wrote: " << posX << " - " << posY;
                
                
            
            }
            if (posX_tymczas - posX > 2 || posY_tymczas - posY > 2) {
                cout << "obracaj w prawo" << endl;
                
                string pozycja2 = to_string(posX) + "," + to_string(posY) + "\n";
                
                int nout2 = serial.swrite(pozycja2);
                
                std::cout << "Wrote: " << posX << " - " << posY;
                
            }
            posX_tymczas = posX;
            posY_tymczas = posY;


          
            

        }
        //Canny(image1,image3,20,80);
        imshow("camera", image);
        imshow("camera1", imagewykryty);
        if (waitKey(1) == 27) break;
    }
    
    waitKey(0);
    // Wait for any keystroke
    return 0;
    

}