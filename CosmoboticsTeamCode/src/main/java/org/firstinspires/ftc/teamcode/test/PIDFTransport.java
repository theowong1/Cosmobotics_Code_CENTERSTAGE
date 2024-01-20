package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@TeleOp
public class PIDFTransport extends OpMode {
    private PIDController slidesController;

    public static double slidesp = .022, slidesi = 0, slidesd = 0.00038;
    public static int slidesTarget = 0;

    private DcMotorEx slidesMotor;
    private PIDController armController;

    public static double armp = 0.006, armi = 0, armd = 0.0004; //TODO: Tune in Relation to Slides
    public static double f = 0.06;
    public static int armTarget = 0;

    private final double arm_ticks_in_degrees = 1425.1 / 1800;

    private DcMotorEx armMotor;

    private ServoImplEx intakeRotation;
    private TouchSensor zeroLimit;

    public static double servoPos;

    @Override
    public void init() {
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");
        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");

        slidesController = new PIDController(slidesp, slidesi, slidesd);

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armController = new PIDController(armp, armi, armd);

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        intakeRotation.setPosition(servoPos);
        if (zeroLimit.isPressed()) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        armController.setPID(armp, armi, armd);
        int armPos = armMotor.getCurrentPosition();
        double armpid = armController.calculate(armPos, armTarget);
        double armff = Math.cos(Math.toRadians(armTarget / arm_ticks_in_degrees)) * f;

        double armPower = armpid + armff;



        slidesController.setPID(slidesp, slidesi, slidesd);
        int slidesPos = slidesMotor.getCurrentPosition();
        double slidespid = slidesController.calculate(slidesPos, slidesTarget);

        double slidespower = slidespid;

        if (!gamepad1.b) {
            slidesMotor.setPower(slidespower);
            armMotor.setPower(armPower);
        }

        telemetry.addData("ispressed", zeroLimit.isPressed());
        telemetry.addData("pos", armPos);
        telemetry.addData("target", armTarget);
        telemetry.update();
    }
}