package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.sun.tools.javac.util.ArrayUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Vector;


public class ConeOrientate extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();

    Point l = new Point(0,0);
    Size s = new Size(0,0);
    RotatedRect p = new RotatedRect(l, s, 0);


    OpenCvCamera cam;

    //For some reason this variable is returning a List error
    List<MatOfPoint> cont = new ArrayList<MatOfPoint>();


    MatOfPoint sortedCont;
    Scalar color = new Scalar(100,129,75);
    double[] rectArea = {0};
    RotatedRect[] rectPoint = {p};
    Mat output = new Mat();
    Mat hierarchy = new Mat();



    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        //Converting RGB image to HSV in order to easily look for color ranges
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Low Color End
        Scalar lowHsv = new Scalar(0, 49, 75);

        //High Color End
        Scalar highHsv = new Scalar(75, 141, 198);


        //Converting HSV image to Binary by checking if a pixel is in the specified range
        Core.inRange(mat, lowHsv, highHsv, mat);


        Imgproc.findContours(mat, cont, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);





        return mat;
    }

    public void sortCont(){
        int i = 0;
        for (MatOfPoint contour : cont) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);

            double rectangleArea = boundingRect.size.area();

            rectArea = Arrays.copyOf(rectArea, rectArea.length+1);
            rectArea[i] = rectangleArea;

            rectPoint = Arrays.copyOf(rectPoint, rectPoint.length+1);
            rectPoint[i] = boundingRect;


            i++;



        }

        for(int a = 0; a < rectArea.length; a++){
            for(int b = a+1; b < rectArea.length; b++){
                if(rectArea[a] > rectArea[b]){
                    double tempArea = rectArea[a];
                    RotatedRect tempRect = rectPoint[a];

                    rectArea[a] = rectArea[b];
                    rectPoint[a] = rectPoint[b];

                    rectArea[b] = tempArea;
                    rectPoint[b] = tempRect;
                }
            }

        }

    }
    public double coneArea(){
        return rectArea[0];
    }

    public Point conePos(){
        return rectPoint[0].center;
    }
}
