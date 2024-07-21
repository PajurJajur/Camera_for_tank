#include <iostream> 
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;
using namespace std;

// Driver code 
int main()
{

    Mat image = imread("C:/Users/48609/Downloads/karty.jpg");
    Mat image1;
    vector<Point2f> punkty_pocz = { {62,84},{254,70},{290,294},{75,315} };
    vector<Point2f> punkty_po = { {0,0},{150,0},{150,250},{0,250} };

    Mat H = findHomography(punkty_pocz, punkty_po);
    warpPerspective(image, image1, H, Size(250,300));
    imshow("normalny", image);
    imshow("z lotu ptaka", image1);
    waitKey(0);
    //Mat image2, image3;

    /*
    VideoCapture Cam(0);
    // Show Image inside a window with 
    // the name provided 
    namedWindow("camera");
    while (true) {
        Cam.read(image);
        cvtColor(image, image1, COLOR_BGR2GRAY);
       // bilateralFilter(image1, image2,9,75,75);
        Canny(image1,image3,20,80);
        imshow("camera", image3);
        if (waitKey(1) == 27) break;
    }
    cout << image.size() << endl;
    waitKey(0);
    // Wait for any keystroke 
    return 0;
    */

}