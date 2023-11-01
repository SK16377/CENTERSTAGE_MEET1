package org.firstinspires.ftc.teamcode.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.opencv.CenterstageDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Centerstage auto", group="Auto")
public class meet_1_auto extends LinearOpMode {
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        CenterstageDetector detector = new CenterstageDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                                     {
                                         @Override
                                         public void onOpened()
                                         { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }
                                         @Override
                                         public void onError(int errorCode)
                                         {
                                             //This will be called if the camera could not be opened
                                         }
                                     }
        );
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        waitForStart();
        switch (detector.getLocation()) {
            case LEFT://right
                dropPurplePixelPose = new Pose2d(-26, 8, Math.toRadians(0));
                break;
            case RIGHT://middle
                dropPurplePixelPose = new Pose2d(-30, 3, Math.toRadians(0));
                break;
            case NOT_FOUND://left
                dropPurplePixelPose = new Pose2d(-30, -9, Math.toRadians(-45));
        }
        midwayPose1 = new Pose2d(-14, 13, Math.toRadians(-45));
        waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
        parkPose = new Pose2d(-8, 30, Math.toRadians(-90));
        //break;
        webcam.stopStreaming();
    }
}




//package org.firstinspires.ftc.teamcode.Comp_code;
///* Copyright (c) 2019 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//
//
//import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.opencv.CenterstageDetector;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import org.firstinspires.ftc.teamcode.opencv.CenterstageDetector;
//import java.util.List;
//
///**
// * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
// */
//@Autonomous(name = "meet 1", group = "00-Autonomous", preselectTeleOp = "pixel_tele")
//public class meet_1_auto extends LinearOpMode {
//    CenterstageDetector CenterstageDetector;
//
//    //botmap robot = new botmap();
//    //webcam = hwMap.get(WebcamName.class, "Webcam 1");
//    public static String TEAM_NAME = "Spicy Ketchup";
//    public static int TEAM_NUMBER = 16377;
//    OpenCvCamera webcam;
//    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    //Vision parameters
//    //private TfodProcessor tfod;
//    // private VisionPortal visionPortal;
//
//    //Define and declare Robot Starting Locations
//    public enum START_POSITION {
//        BLUE_LEFT,
//        BLUE_RIGHT,
//        RED_LEFT,
//        RED_RIGHT
//    }
//
//    public static START_POSITION startPosition;
//
//    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
//        LEFT,
//        RIGHT,
//        MIDDLE
//    }
//
//    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //Key Pay inputs to selecting Starting Position of robot
//        selectStartingPosition();
//        telemetry.addData("Selected Starting Position", startPosition);
//
//        //Activate Camera Vision that uses TensorFlow for pixel detection
//        //initTfod();
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        //waitForStart();
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            telemetry.addData("Selected Starting Position", startPosition);
//
//            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
//            //runTfodTensorFlow();
//            //telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
//            telemetry.update();
//        }
//
//        //Game Play Button  is pressed
//
//
//        int cameraMonitorViewId = hardwareMap.appContext
//                .getResources().getIdentifier("cameraMonitorViewId",
//                        "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        CenterstageDetector detector = new CenterstageDetector(telemetry);
//        webcam.setPipeline(detector);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                                         @Override
//                                         public void onOpened() {
//                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                                         }
//
//                                         @Override
//                                         public void onError(int errorCode) {
//                                             //This will be called if the camera could not be opened
//                                         }
//                                     }
//        );
//
//        waitForStart();
//        switch (detector.getLocation()) {
//            case LEFT://right
//                // ...
//                break;
//            case RIGHT://middle
//                // ...
//                break;
//            case NOT_FOUND://left
//                // ...
//        }
//        webcam.stopStreaming();
//        if (opModeIsActive() && !isStopRequested()) {
//            //Build parking trajectory based on last detected target by vision
//            runAutonoumousMode();
//        }
//    }
//
//    public void runAutonoumousMode() {
//        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
//        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
//        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
//        Pose2d midwayPose1 = new Pose2d(0,0,0);
//        Pose2d midwayPose1a = new Pose2d(0,0,0);
//        Pose2d intakeStack = new Pose2d(0,0,0);
//        Pose2d midwayPose2 = new Pose2d(0,0,0);
//        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
//        Pose2d parkPose = new Pose2d(0,0, 0);
//        double waitSecondsBeforeDrop = 0;
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
//        int cameraMonitorViewId = hardwareMap.appContext
//                .getResources().getIdentifier("cameraMonitorViewId",
//                        "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        CenterstageDetector detector = new CenterstageDetector(telemetry);
//        webcam.setPipeline(detector);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                                         @Override
//                                         public void onOpened() {
//                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                                         }
//
//                                         @Override
//                                         public void onError(int errorCode) {
//                                             //This will be called if the camera could not be opened
//                                         }
//                                     }
//        );
//        waitForStart();
////            switch (startPosition){
////                switch (detector.getLocation()) {
////             case LEFT://right
////                // ...
////                 break;
////                case RIGHT://middle
////                // ...
////                break;
////                case NOT_FOUND://left
////                // ...
////                 }
////            }
//        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
//        moveBeyondTrussPose = new Pose2d(-15,0,0);
//
//        switch (startPosition) {
//            case BLUE_LEFT:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                switch(identifiedSpikeMarkLocation){
//                    case LEFT:
//                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
//                        dropYellowPixelPose = new Pose2d(23, 36, Math.toRadians(-90));
//                        break;
//                    case MIDDLE:
//                        dropPurplePixelPose = new Pose2d(30, 3, Math.toRadians(0));
//                        dropYellowPixelPose = new Pose2d(30, 36,  Math.toRadians(-90));
//                        break;
//                    case RIGHT:
//                        dropPurplePixelPose = new Pose2d(30, -9, Math.toRadians(-45));
//                        dropYellowPixelPose = new Pose2d(37, 36, Math.toRadians(-90));
//                        break;
//                }
//                midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));
//                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
//                parkPose = new Pose2d(8, 30, Math.toRadians(-90));
//                break;
//
//            case RED_RIGHT:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                switch(identifiedSpikeMarkLocation){
//                    case LEFT:
//                        dropPurplePixelPose = new Pose2d(-30, -9, Math.toRadians(-45));
//                        // dropYellowPixelPose = new Pose2d(21, -36, Math.toRadians(90));
//                        break;
//                    case MIDDLE:
//                        dropPurplePixelPose = new Pose2d(-30, 3, Math.toRadians(0));
//                        //dropYellowPixelPose = new Pose2d(29, -36,  Math.toRadians(90));
//                        break;
//                    case RIGHT:
//                        dropPurplePixelPose = new Pose2d(-26, 8, Math.toRadians(0));
//                        // dropYellowPixelPose = new Pose2d(37, -36, Math.toRadians(90));
//                        break;
//                }
//                midwayPose1 = new Pose2d(-14, 13, Math.toRadians(-45));
//                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
//                parkPose = new Pose2d(-8, 30, Math.toRadians(-90));
//                break;
//
//            case BLUE_RIGHT:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                switch(identifiedSpikeMarkLocation){
//                    case LEFT:
//                        dropPurplePixelPose = new Pose2d(27, 9, Math.toRadians(45));
//                        dropYellowPixelPose = new Pose2d(27, 86, Math.toRadians(-90));
//                        break;
//                    case MIDDLE:
//                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
//                        dropYellowPixelPose = new Pose2d(34, 86, Math.toRadians(-90));
//                        break;
//                    case RIGHT:
//                        dropPurplePixelPose = new Pose2d(26, -8, Math.toRadians(0));
//                        dropYellowPixelPose = new Pose2d(43, 86, Math.toRadians(-90));
//                        break;
//                }
//                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
//                midwayPose1a = new Pose2d(18, -18, Math.toRadians(-90));
//                intakeStack = new Pose2d(52, -19,Math.toRadians(-90));
//                midwayPose2 = new Pose2d(52, 62, Math.toRadians(-90));
//                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
//                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
//                break;
//
//            case RED_LEFT:
//                drive = new MecanumDrive(hardwareMap, initPose);
//                switch(identifiedSpikeMarkLocation){
//                    case LEFT:
//                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
//                        dropYellowPixelPose = new Pose2d(37, -86, Math.toRadians(90));
//                        break;
//                    case MIDDLE:
//                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
//                        dropYellowPixelPose = new Pose2d(29, -86, Math.toRadians(90));
//                        break;
//                    case RIGHT:
//                        dropPurplePixelPose = new Pose2d(27, -9, Math.toRadians(-45));
//                        dropYellowPixelPose = new Pose2d(21, -86, Math.toRadians(90));
//                        break;
//                }
//                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
//                midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
//                intakeStack = new Pose2d(52, 19,Math.toRadians(90));
//                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
//                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
//                parkPose = new Pose2d(50, -84, Math.toRadians(90));
//                break;
//        }
//    }
//
//
//
//    public void selectStartingPosition() {
//        telemetry.setAutoClear(true);
//        telemetry.clearAll();
//        //******select start pose*****
//        while(!isStopRequested()){
//            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
//                    TEAM_NAME, " ", TEAM_NUMBER);
//            telemetry.addData("---------------------------------------","");
//            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
//            telemetry.addData("    Blue Left   ", "(X / ▢)");
//            telemetry.addData("    Blue Right ", "(Y / Δ)");
//            telemetry.addData("    Red Left    ", "(B / O)");
//            telemetry.addData("    Red Right  ", "(A / X)");
//            if(gamepad1.x){
//                startPosition = START_POSITION.BLUE_LEFT;
//                break;
//            }
//            if(gamepad1.y){
//                startPosition = START_POSITION.BLUE_RIGHT;
//                break;
//            }
//            if(gamepad1.b){
//                startPosition = START_POSITION.RED_LEFT;
//                break;
//            }
//            if(gamepad1.a){
//                startPosition = START_POSITION.RED_RIGHT;
//                break;
//            }
//            telemetry.update();
//        }
//        telemetry.clearAll();
//    }
//}