package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
@TeleOp(group = "Testing")
public class TestSlides extends OpMode {
    private DcMotorEx slidesMotor;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;


    // TODO: Maybe set beginning to zero
    //Safe, Close, Med, Far
    public static final int[] slidesPositions = {0, 1000, 2000, 3000};

    public static int state = 0;
    @Override
    public void init() {
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setTargetPosition(state);
        slidesMotor.setPower(1);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kP, kI, kD, kF));
    }

    @Override
    public void loop() {
        telemetry.addLine("Directions: Press each button to move slide to its respective position");
        if (gamepad1.dpad_down) state = 0; // Safe
        if (gamepad1.dpad_left) state = 1; // Close
        if (gamepad1.dpad_up) state = 2; // Med
        if (gamepad1.dpad_right) state = 3; // Far

        slidesMotor.setTargetPosition(state);
        telemetry.addData("target-pos: ", state);

        telemetry.addData("pos: ", slidesMotor.getCurrentPosition());
    }
}


