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
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
@TeleOp(group = "Testing")
public class GetTransportPositions extends OpMode {
    public static DcMotorEx armMotor;
    public static DcMotorEx slidesMotor;

    public static ServoImplEx rightIntake;

    public static ServoImplEx leftIntake;
    public static ServoImplEx intakeRotation;;
    public static double rightIntakePos = 0;
    public static double leftIntakePos = 0;
    public static double intakeRotationPos = 0;
    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");
        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake");
        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake");

        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
        telemetry.addData("SlidesPos: ", slidesMotor.getCurrentPosition());
        telemetry.addData("LeftIntakePos: ", leftIntakePos);
        telemetry.addData("RightIntakePos: ", rightIntakePos);
        telemetry.addData("RightIntakePos: ", intakeRotationPos);
    }
}

