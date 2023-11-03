package org.firstinspires.ftc.teamcode.Comp_code;/*
 * Some declarations that are boilerplate are
 * skipped for the sake of brevity.
 * Since there are no real values to use, named constants will be used.
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Comp_code.bot_map;

//import Testing.teleptest;


@TeleOp(name="pixel_tele")
public class pixel_tele extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    //TELEmap robot   = new TELEmap();
    bot_map robot = new bot_map();


    // used with the dump servo, this will get covered in a bit
    ElapsedTime toggleTimer = new ElapsedTime();
    //double toggleTime = .25;

    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .002;  // prevents arm from falling from gravity


    public static int LiftTarget = 0; // target position

    public static int START_POS = 0;
    public static int LOW = 2100; //1208 = LOW
    public static int MID = 2530; //2078 = MID
    // public static int HIGH = 500; //2900 = HIGH
    private DcMotorEx llift;
    private DcMotorEx rlift;
    double SpeedAdjust = 1;
    int lift_speed = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested()) {
            robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

            while (gamepad2.left_trigger == 1) {
                robot.climb.setPower(2);
            }
            if (gamepad2.left_trigger == 0) {
                robot.climb.setPower(0);
            }
            while (gamepad2.right_trigger == 1) {
                robot.climb.setPower(-4);
            }
//            if (gamepad2.dpad_up) {
//                LiftTarget = HIGH;
//            }
            if (gamepad2.dpad_down) {
                lift_speed = 1;
                lift_speed = 1;
               // LiftTarget = LOW;
            }
            else if (gamepad2.dpad_right) {
                LiftTarget = MID;
            }
            else if (gamepad2.dpad_left) {
                LiftTarget = MID;
            }
            if (gamepad2.cross) {
                LiftTarget = 0;
            }
            if (gamepad2.right_trigger == 0) {
                robot.climb.setPower(0);
//            }
                while (gamepad1.right_trigger == 1) {
                    robot.intake.setPower(-4);
                }
                if (gamepad1.right_trigger == 0) {
                    robot.intake.setPower(0);
                }
                while (gamepad1.left_trigger == 1) {
                    robot.intake.setPower(2);
                }
                if (gamepad1.left_trigger == 0) {
                    robot.intake.setPower(0);
                }
                if (gamepad1.left_bumper) {
                    SpeedAdjust = 4;
                } else if (gamepad1.right_bumper) {
                    SpeedAdjust = 1;
                }
            }
            llift.setPower(lift_speed);
            rlift.setPower(lift_speed);
        }

    }
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            llift = hardwareMap.get(DcMotorEx.class,"Llift");
            rlift = hardwareMap.get(DcMotorEx.class,"Rlift");

            llift.setDirection(DcMotorEx.Direction.FORWARD);
            rlift.setDirection(DcMotorEx.Direction.REVERSE);

            llift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            llift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void update() {
            controller.setPID(p, i, d);

            int larmPos = llift.getCurrentPosition();
            int rarmPos = rlift.getCurrentPosition();

            double Lpid = controller.calculate(larmPos, LiftTarget);
            double Rpid = controller.calculate(rarmPos, LiftTarget);

            // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
            // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

            double Lpower = Lpid + f;
            double Rpower = Rpid + f;

            llift.setPower(Lpower);
            rlift.setPower(Rpower);

//            telemetry.addData("pos", larmPos);
//            telemetry.addData("pos", rarmPos);
//            telemetry.addData("target", LiftTarget);
//            telemetry.addData("target", LiftTarget);
//            telemetry.update();
        }
    }
}