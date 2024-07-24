#include <iostream> 
#include <stdlib.h>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>



using namespace cv;
using namespace std;




// Driver code 
int main()
{
    int serial_port;
    
    if ((serial_port = serialOpen("/dev/ttyACM0", 9600)) < 0)
{
    cerr << "nie da rady kolego\n";
    return 1;
}

    

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
    namedWindow("camera",WINDOW_NORMAL);
    namedWindow("camera1",WINDOW_NORMAL);
    resizeWindow("camera",800,600);
    resizeWindow("camera1",800,600);
    
    
    while (true) {


        
        Cam.read(image);
        waitKey(30); // 30 ms opóźnienia
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
            line(image, Point(posX-100, posY), Point(posX+100,posY), Scalar(255,0,0),3);
            line(image, Point(posX, posY-100), Point(posX,posY+100), Scalar(255,0,0),3);

            if (posX - posX_tymczas > 2 || posY - posY_tymczas > 2)        
            {
                cout << "obracaj w lewo" << endl;
                
                string pozycja = to_string(posX)+","+to_string(posY)+"\n";
                
                serialPuts(serial_port, pozycja.c_str());
                
                std::cout << "Wrote: " << pozycja.c_str() ;
                
                
            
            }
            if (posX_tymczas - posX > 2|| posY_tymczas - posY > 2) 
            {
                cout << "obracaj w prawo" << endl;
                
                string pozycja2 = to_string(posX) + "," + to_string(posY) + "\n";
                
                serialPuts(serial_port, pozycja2.c_str());
                
                std::cout << "Wrote: " << pozycja2.c_str() ;
                
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
    serialClose(serial_port);
    return 0;
    

}
