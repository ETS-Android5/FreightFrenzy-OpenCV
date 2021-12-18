/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Imgproc.*;
import org.opencv.imgproc.Moments;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    static int i = 0;
    static int red_lower = 0;
    static int red_upper = 255;
    static int blue_lower = 0;
    static int blue_upper = 255;
    static int green_lower = 0;
    static int green_upper = 255;
    static int area_lower = 10000;
    static int area_upper = 250000;
    private WebcamName webcam1;
    private WebcamName webcam2;
    private SwitchableCamera switchableCamera;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AcuULCD/////AAABmT3WggAKcUf6ujl4RXFuE69bIn9w0BkcGcPeVjpZDBjTFgl6Dso8WLn3H+A2b7SlxWuv13sZb0+zRq505bguhB0t0IWJvUFUirziTiectN5zMeJm4aluqNOljkwju1ESmz/ckXZ4XQqa21xxhc5pV+imIdR/m/7B3ovzu5ElpDYHTBBS4O365Aac3veTpj2yqPalexQM0O5T+W5uknxmHIiuUYFKSQt/OAET5gvycbEdsrgeqrQvRhRP9LcepuuCieCW49VUsc3hft73eUHRmu5M0dOICqTOuuX3JYRekdn9fSA45yjF/N3oFuu7snxURvaHR+A8lNJHCYV5LBo51dFM8dJYfjW8oDMNYO6haPsW";





    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initVuforia2Cameras();

//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//        VideoCapture capture = new VideoCapture(0);
////        VideoCapture capture2 = new VideoCapture(1);
//        Mat matrix1 = new Mat();
//        Mat matrix2 = new Mat();
//        Mat newMatrix1 = new Mat();
//        Mat newMatrix2 = new Mat();
//        Mat newMatrix1Gray = new Mat();
//        Mat newMatrix2Blur = new Mat();
//        Mat newMatrix1Canny = new Mat();
//        List<MatOfPoint> contours = new ArrayList<>();
//
//
//        capture.read(matrix1);

        waitForStart();




        /**
         * The below code takes the given image in the filepath and essentially just runs a lot of filters on the same image
         * and ALSO finds and draws the contours of each image, before plotting the centroid of each contour with an ellipse.
         */





        /**
         * End of contour code
         */


        /**
         * This block of code runs for 10,000 frames of images collected from the local webcam of the computer.
         * The variables and intialization of the webcam is at the beginning of the main method.
         */
//        while (i < 10000) {
//            System.out.println(i);
//            capture.read(matrix1);
////            Imgproc.resize(matrix1, matrix1, new Size(50, 30));
////            Imgproc.resize(matrix1, matrix1, new Size(1280, 720));
//
//            Scalar lowerHSV = new Scalar(105, 90, 20);
//            Scalar upperHSV = new Scalar(125, 255, 230);
//            Mat matrixHSV = new Mat();
//            Mat Mask = new Mat();
//            Mat coloredMask = new Mat();
//            Imgproc.cvtColor(matrix1, matrixHSV, Imgproc.COLOR_RGB2HSV);
//            Core.inRange(matrixHSV, lowerHSV, upperHSV, Mask);
//            Core.bitwise_and(matrix1, matrix1, coloredMask, Mask);
//            Mat binaryMask = new Mat();
//            Imgproc.threshold(Mask, binaryMask, 200, 255, Imgproc.THRESH_BINARY_INV);
//            //Finding Contours
//            List<MatOfPoint> contoursMask = new ArrayList<>();
//            Mat hierarcheyMask = new Mat();
//            Imgproc.findContours(binaryMask, contoursMask, hierarcheyMask, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            Iterator<MatOfPoint> itMask = contours.iterator();
//
//
//            RotatedRect box2 = new RotatedRect(new Point(316, 170), new Size(5, 5), 180);
//
//            Mat drawMask = Mat.zeros(binaryMask.size(), CvType.CV_8UC3);
//            Mat centroidsMask;
//            for (int i = 0; i < contoursMask.size(); i++) {
//                int area = (int) Imgproc.contourArea(contoursMask.get(i));
//                if (area > area_lower && area < area_upper) {
//                    centroidsMask = new Mat();
//                    System.out.println(contoursMask);
//                    System.out.println(contoursMask.get(i));
//
//                    Scalar color = new Scalar(0, 255, 0);
//
//                    //Drawing Contours
//                    Imgproc.drawContours(drawMask, contoursMask, i, color, 2, Imgproc.LINE_8, hierarcheyMask, 2, new Point());
//                } else {
//
//                }
//            }
//
//            for (int contIdx = 0; contIdx < contoursMask.size(); contIdx++) {
//                int area = (int) Imgproc.contourArea(contoursMask.get(contIdx));
//                if (area > area_lower && area < area_upper) {
//                    System.out.println("Contour Area: " + area);
//                    Moments Mmask = Imgproc.moments(contoursMask.get(contIdx));
//                    int cx = (int) (Mmask.get_m10() / Mmask.get_m00());
//                    int cy = (int) (Mmask.get_m01() / Mmask.get_m00());
//                    Imgproc.ellipse(drawMask, new RotatedRect(new Point(cx, cy), new Size(10, 10), 180), new Scalar(0, 0, 255));
//                    System.out.println(cx + ", " + cy);
//
//                }
//            }
//
//            i++;
//        }
        }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initVuforia2Cameras() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    }

