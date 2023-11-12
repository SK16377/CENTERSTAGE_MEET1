package opencv.opencv;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="centerstage auto", group="Auto")
public class meet_1_auto extends LinearOpMode {
    OpenCvCamera webcam;
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(14.40, -68.62, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector_2_ranges detector = new detector_2_ranges(telemetry);
        //START_POSITION position = new CenterstageDetector(telemetry);
        webcam.setPipeline(detector);
        //private detector.getLocation location = detector.getLocation().LEFT;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

            @Override
                                         public void onError(int errorCode) {
                                             //This will be called if the camera could not be opened
                                         }
                                     }

        );


        Trajectory left = TrajectoryGenerator.generateTrajectory(startPose)
                .splineTo(new Vector2d(22.93, -36.44), Math.toRadians(90.00))
                .build();



        waitForStart();
        //drive.setPoseEstimate(startPose);

        switch (detector.getLocation()) {
            case LEFT:

                break;
            case NOT_FOUND:

                break;
            case RIGHT:

                break;
        }




        webcam.stopStreaming();
    }
}



