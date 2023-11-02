package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class pid_testing extends OpMode {
    private PIDController controller;
    public static double p = .006, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .05;  // prevents arm from falling from gravity


    public static int LiftTarget = 0; // target position

    //public static int START_POS = 230;
    public static int LOW = 0; //1208 = LOW
    public static int MID = 180; //2078 = MID
   // public static int HIGH = 500; //2900 = HIGH
   private DcMotorEx llift;
    private DcMotorEx rlift;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        llift = hardwareMap.get(DcMotorEx.class,"Llift");
        rlift = hardwareMap.get(DcMotorEx.class,"Rlift");

        llift.setDirection(DcMotorEx.Direction.REVERSE);
        rlift.setDirection(DcMotorEx.Direction.FORWARD);

        llift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        llift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
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

        telemetry.addData("pos", larmPos);
        telemetry.addData("pos", rarmPos);
        telemetry.update();
    }
}
