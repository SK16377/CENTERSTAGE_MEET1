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


    double SpeedAdjust = 1;

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
        }
    }
}