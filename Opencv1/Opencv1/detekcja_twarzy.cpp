#include <iostream> 
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;
using namespace std;

// Driver code 
int main()
{
    

    Mat image;
    Mat image1;

    Mat image2, image3, imagewykryty;
    Point Srodek_tymczas;

    int hmin = 0, smin = 14, vmin = 169;
    int hmax = 179, smax = 50, vmax = 215;

    namedWindow("paski_zmian", (640, 200));
    createTrackbar("Hue Min", "paski_zmian", &hmin, 179);
    createTrackbar("saturation Min", "paski_zmian", &smin, 255);
    createTrackbar("value Min", "paski_zmian", &vmin, 255);
    createTrackbar("Hue Max", "paski_zmian", &hmax, 179);
    createTrackbar("saturation Max", "paski_zmian", &smax, 255);
    createTrackbar("value Max", "paski_zmian", &vmax, 255);
    


    VideoCapture Cam(0);
    CascadeClassifier faceCascade;
    faceCascade.load("C:/Users/48609/Downloads/haarcascade_frontalface_default.xml");
    if (faceCascade.empty()) { cout << "XML file nie ma xD" << endl; }
    vector<Rect> twarze;
    // Show Image inside a window with
    // the name provided
    namedWindow("camera");
    while (true) {
        Cam.read(image);

        cvtColor(image, image1, COLOR_BGR2HSV);
        Scalar dolna(hmin, smin, vmin);
        Scalar gorna(hmax, smax, vmax);

        inRange(image1, dolna, gorna, imagewykryty);

       

        faceCascade.detectMultiScale(image, twarze, 1.1, 4);
        for (int i = 0; i < twarze.size(); i++) {
            rectangle(image, twarze[i].tl(), twarze[i].br(), Scalar(255, 0, 0), 1);
            Point srodek(twarze[i].x + twarze[i].width / 2, twarze[i].y + twarze[i].height / 2);
            
            if (srodek.x - Srodek_tymczas.x > 10) {
                cout << "obracaj w lewo" << endl;
            }
            if (Srodek_tymczas.x - srodek.x > 10) {
                cout << "obracaj w prawo" << endl;
            }
            Srodek_tymczas = srodek;
        }

        //Canny(image1,image3,20,80);
        imshow("camera", image);
        if (waitKey(1) == 27) break;
    }

    waitKey(0);
    // Wait for any keystroke
    return 0;


}