package org.firstinspires.ftc.teamcode;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class ImageCheck {

    Mat mat = new Mat();

    Point offset = new Point(0, 0);




    public Mat processFrame(Mat input){

        //Processing input frame and converting to HSV matrix format
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);



        //Setting low and high matrix values for post processing
        Scalar lowHsv = new Scalar(0, 89, 126);
        Scalar highHsv = new Scalar(58, 255, 255);

        //Change source frame to binary format for shape post processing
        Core.inRange(input, lowHsv, highHsv, mat);


        //Output list of coordinates
        List<MatOfPoint> contour = new ArrayList<MatOfPoint>();


        //Throw away variable
        Mat hierarchy = new Mat();


        //Finding the coordinates of each of the vertices in source image
        //simplifies image into a list of important coordinates
        //TODO: Play around with the Apprx and Retrieval functions to see which gives best results
        Imgproc.findContours(input, contour, hierarchy, Imgproc.CHAIN_APPROX_SIMPLE, Imgproc.RETR_EXTERNAL);



        //Outputs post processed image
        return input;



    }


}
