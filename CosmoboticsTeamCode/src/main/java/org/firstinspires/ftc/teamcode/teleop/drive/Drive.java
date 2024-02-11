package org.firstinspires.ftc.teamcode.teleop.drive;

import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Controls;

public class Drive {
    public Telemetry telemetry;
    public MotorEx frontRight, frontLeft, backRight, backLeft;

    public MecanumDrive drive;

    public double botHeading;
    public IMU imu;

    private PIDFController turnController = new PIDFController(new PIDCoefficients(1.25, 0, 0.0002));

    public double IMUOffset;

    public double x;
    public double y; // Remember, Y stick value is reversed
    public static double rx;
    public double RedOffset = 270;
    public double BlueOffset = 90;

    public double slowModeOffset = 1;

    public static final boolean robotCentric = false;
    public static final boolean brakeMode = true;
    public Drive(GamepadEx gamepadEx, HardwareMap hardwareMap) {
        frontRight = new MotorEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        frontLeft = new MotorEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        backRight = new MotorEx(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);
        backLeft = new MotorEx(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);

        if (brakeMode) {
            frontRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            frontRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
            frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
        }

        frontRight.setRunMode(MotorEx.RunMode.RawPower);
        frontLeft.setRunMode(MotorEx.RunMode.RawPower);
        backRight.setRunMode(MotorEx.RunMode.RawPower);
        backLeft.setRunMode(MotorEx.RunMode.RawPower);

        frontRight.setInverted(true);
        frontLeft.setInverted(false);
        backRight.setInverted(true);
        backLeft.setInverted(false);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        if (isRed) {
            IMUOffset = BlueOffset;
        } else {
            IMUOffset = RedOffset;
        }

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + IMUOffset;

        x = gamepadEx.getLeftX() / slowModeOffset;
        y = gamepadEx.getLeftY() / slowModeOffset;
        rx = gamepadEx.getRightX() / slowModeOffset;
    }

    public void normalTurn(GamepadEx gamepadEx) {
        rx = gamepadEx.getRightX() / slowModeOffset;
    }
    public void autoTurnLeft() {
        if (isRed) {
            rx = calcRotBasedOnIdeal(botHeading, Math.toRadians(270));
        } else {
            rx = calcRotBasedOnIdeal(botHeading, Math.toRadians(45));
        }
    }

    public void autoTurnRight() {
        if (isRed) {
            rx = calcRotBasedOnIdeal(botHeading, Math.toRadians(315));
        } else {
            rx = calcRotBasedOnIdeal(botHeading, Math.toRadians(90));
        }
    }

    public void activateSlowMode() {
        slowModeOffset = 2;
    }

    public void deactivateSlowMode() {
        slowModeOffset = 1;
    }
    public void resetImu(){
        IMUOffset = 0;
        imu.resetYaw();
    }

    private double calcRotBasedOnIdeal(double heading, double idealHeading) {
        // Error in rotations (should always be between (-0.5,0.5))
        double err = angleWrap(idealHeading - heading);
        turnController.setTargetPosition(0);
        double correction = turnController.update(err);
        return correction;
    }

    private double angleWrap(double angle) {
        // Changes any angle between [-180,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI)
            angle -= 2 * Math.PI * (angle > 0? 1 : -1);
        return angle;
    }

    private double normalizeAngle(double angle) {
        // Normalizes angle in [0,360] range
        while (angle > 2 * Math.PI) angle -= 2 * Math.PI;
        while (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    public void update(GamepadEx gamepadEx) {
        x = gamepadEx.getLeftX() / slowModeOffset;
        y = gamepadEx.getLeftY() / slowModeOffset;
        if (robotCentric) {
            drive.driveRobotCentric(x, y, rx);
        } else {
            drive.driveFieldCentric(x, y, rx, botHeading);
        }
        telemetry.addData("strafeSpeed", x);
        telemetry.addData("forwardSpeed", y);
        telemetry.addData("turnSpeed/turn", rx);
        telemetry.addData("botHeading", botHeading);
        telemetry.addData("frontLeft", frontLeft.getVelocity());
        telemetry.addData("backLeft", backLeft.getVelocity());
        telemetry.addData("frontRight", frontRight.getVelocity());
        telemetry.addData("backRight", backRight.getVelocity());
    }
}