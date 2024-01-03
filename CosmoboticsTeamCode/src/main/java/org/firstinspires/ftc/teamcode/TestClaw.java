package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.Robot;

@TeleOp
public class TestClaw extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        boolean rightIntakeOpen = gamepad1.right_bumper;
        boolean leftIntakeOpen = gamepad1.left_bumper;
        boolean rightIntakeClose = gamepad1.start;
        boolean leftIntakeClose = gamepad1.back;
        ElapsedTime gamepadBackTime = new ElapsedTime();
        ElapsedTime gamepadStartTime = new ElapsedTime();
        ElapsedTime gamepad1leftBumperTime = new ElapsedTime();
        ElapsedTime gamepad1rightBumperTime = new ElapsedTime();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (leftIntakeOpen && gamepad1leftBumperTime.milliseconds() > 500) {
                robot.leftIntake.setPosition(1);
                gamepad1leftBumperTime.reset();
            }
            if (rightIntakeOpen && gamepad1rightBumperTime.milliseconds() > 500) {
                robot.rightIntake.setPosition(1);
                gamepad1rightBumperTime.reset();
            }
            if (leftIntakeClose && gamepadBackTime.milliseconds() > 500) {
                robot.leftIntake.setPosition(0);
                gamepadBackTime.reset();
            }
            if (rightIntakeClose && gamepadStartTime.milliseconds() > 500) {
                robot.leftIntake.setPosition(0);
                gamepadStartTime.reset();
            }
        }
    }
}
