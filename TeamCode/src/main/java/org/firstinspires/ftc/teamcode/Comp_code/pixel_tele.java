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

//import Comp_code.bot_map;

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

    public PIDController controller;
    public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .002;  // prevents arm from falling from gravity


    public static int LiftTarget = 0; // target position

    public static int START_POS = 0;
    public static int LOW = 2100; //1208 = LOW
    public static int MID = 2530; //2078 = MID
    // public static int HIGH = 500; //2900 = HIGH
    public DcMotorEx larm;
    public DcMotorEx rarm;

    double SpeedAdjust = 1;
    double lift_speed = 0;

    enum intake {
        START,
        TUBE,
        WHEEL,

    }
    enum wheel {
        START,
       // TUBE,
        WHEEL,

    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        waitForStart();
        robot.raxon.setPosition(0);
        robot.laxon.setPosition(0);
        if (isStopRequested()) return;

        intake INTAKE = intake.START;
        wheel SPIN = wheel.START;

        while (opModeIsActive() && !isStopRequested()) {

//            switch (INTAKE) {
//                case START:
//                    while (gamepad1.left_trigger == 1) {
//                        robot.intake.setPower(2);
//                        INTAKE = intake.WHEEL;
//                    }
//                    break;
//                case WHEEL:
//
//                        robot.wheel.setPosition(1);
//
//                    break;
//                case LIFT_TWIST:
//                    if (waitTimer2.seconds() >= waitTime2) {
//                        robot.grab.setPosition(OPEN);
//                        elbowDown = elbowDownState.START;
//                    }
                    //break;
           // }



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
                //lift_speed = 0.2;
                // lift_speed = 1;
                LiftTarget = LOW;
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
                    robot.wheel.setPosition(1);
                    robot.intake.setPower(2);

                }
                if(gamepad2.circle){
                    robot.raxon.setPosition(.4);
                    robot.laxon.setPosition(.4);
                }
//                if(gamepad1.left_trigger == 1){
//                    robot.wheel.setPosition(1);
//                }
                if (gamepad1.left_trigger == 0) {
                    robot.intake.setPower(0);

                }
                if (gamepad1.left_bumper) {
                    SpeedAdjust = 4;
                } else if (gamepad1.right_bumper) {
                    SpeedAdjust = 1;
                }

            }
            //llift.setPower(lift_speed);
             //rlift.setPower(lift_speed);
            lift.update();
            telemetry.update();
        }

    }
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            larm = hardwareMap.get(DcMotorEx.class,"Llift");
            rarm = hardwareMap.get(DcMotorEx.class,"Rlift");


            larm.setDirection(DcMotorEx.Direction.FORWARD);
            rarm.setDirection(DcMotorEx.Direction.REVERSE);

            larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift

            controller.setPID(p, i, d);

            int larmPos = larm.getCurrentPosition();
            int rarmPos = rarm.getCurrentPosition();

            double Lpid = controller.calculate(larmPos, LiftTarget);
            double Rpid = controller.calculate(rarmPos, LiftTarget);

            // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
            // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

            double Lpower = Lpid + f;
            double Rpower = Rpid + f;

            larm.setPower(Lpower);
            rarm.setPower(Rpower);

//            telemetry.addData("Lpos", larmPos);
//            telemetry.addData("Rpos", rarmPos);
//            telemetry.addData("Ltarget", LiftTarget);
//            telemetry.addData("Rtarget", LiftTarget);
           // telemetry.addData("cone", cone);
            //telemetry.addData("SSVar", SSVar);
            telemetry.update();
        }
    }
}