package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class CosmoboticsTeleOp extends LinearOpMode {
    Robot robot;
    public double autoTurn(double x, double y){
        //initialize robot target angle
        double target = robot.botHeading;

        //calculate target angle using joystick values
        if (x != 0 || y != 0) {
            target = Math.toDegrees(Math.atan2(y, -x));
        }

        //make target angle range 0 ~ 359.999...
        if (target < 0){
            target += 360;
        }

        double angleError = target - robot.botHeading;

        if (angleError > 0){

            //fine turning
            if (Math.abs(angleError) < 8){
                return 0.2;
            }

            return 0.7;

        }else if (angleError < 0){
            //fine turning
            if (Math.abs(angleError) < 8){
                return 0.2;
            }

            return -0.7;

        } else {
            return 0;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        ElapsedTime gamepadBackTime = new ElapsedTime();
        ElapsedTime droneTime = new ElapsedTime();
        ElapsedTime gamepadStartTime = new ElapsedTime();
        ElapsedTime gamepadYTime = new ElapsedTime();
        ElapsedTime gamepadATime = new ElapsedTime();
        ElapsedTime gamepadBTime = new ElapsedTime();
        ElapsedTime gamepad2leftBumperTime = new ElapsedTime();
        ElapsedTime gamepad2rightBumperTime = new ElapsedTime();
        ElapsedTime gamepad1leftBumperTime = new ElapsedTime();
        ElapsedTime gamepad1rightBumperTime = new ElapsedTime();
        ElapsedTime gamepadDpadRightTime = new ElapsedTime();
        ElapsedTime gamepadDpadLeftTime = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //Field Centric Drive:
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            //this version is robot centric
            /*double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;*/
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.dpad_up) {
                robot.resetImu();
            }
            //field centric ***/))
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-robot.botHeading) - y * Math.sin(-robot.botHeading);
            double rotY = x * Math.sin(-robot.botHeading) + y * Math.cos(-robot.botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (-rotY + rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            //robot centric (there is different logic)
            /*double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);*/

            //Game-Pad Booleans and Doubles:
            boolean droneButton = gamepad2.back;
            boolean intakeRotationButton = gamepad2.y;
            boolean rightIntake = gamepad2.right_bumper;
            boolean leftIntake = gamepad2.left_bumper;
            boolean reset = gamepad1.a;
            //Auto-Turn Buttons:
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean backButton = gamepad1.back;
            boolean startButton = gamepad1.start;

            //Drone Initialization:
            if (droneButton && droneTime.milliseconds() > 500) {
                robot.droneServo.setPosition(1);
                droneTime.reset();
            }

            //Reset Arm Initialization:
            if (reset && gamepadATime.milliseconds() > 500) {
                robot.armTarget = 0;
                robot.slidesTarget = 0;
                robot.intakeRotation.setPosition(robot.intakeRotationPosStart);
                robot.leftIntake.setPosition(0);
                robot.rightIntake.setPosition(0);
                gamepadATime.reset();
            }

            //Slides Initialization:
            robot.slidesTarget += gamepad1.right_trigger * 25 - gamepad1.left_trigger * 15;

            if (robot.slidesTarget < 0) {
                robot.slidesTarget = 0;
            } else if (robot.slidesTarget > 2500) {
                robot.slidesTarget = 2500;
            }

            //Arm Initialization:
            robot.armTarget += gamepad2.right_trigger * 25 - gamepad2.left_trigger * 15;

            if (robot.armTarget < 0) {
                robot.armTarget = 0;
            } else if (robot.armTarget > 2500) {
                robot.armTarget = 2500;
            }

            //Intake Rotation Initialization:
            if (intakeRotationButton && gamepadYTime.milliseconds() > 500) {
                if (robot.intakeRotationPos == robot.intakeRotationPosEnd) {
                    robot.intakeRotation.setPosition(robot.intakeRotationPosStart);
                } else {
                    robot.intakeRotation.setPosition(robot.intakeRotationPosEnd);
                }
                gamepadYTime.reset();
            }

            //Claw Initializations:
            if (rightIntake && gamepad1rightBumperTime.milliseconds() > 500) {
                if (robot.rightIntakePos == 0) {
                    robot.rightIntake.setPosition(1);
                } else {
                    robot.rightIntake.setPosition(0);
                }
                gamepad1rightBumperTime.reset();
            }

            if (leftIntake && gamepad1leftBumperTime.milliseconds() > 500) {
                if (robot.leftIntakePos == 0) {
                    robot.leftIntake.setPosition(1);
                } else {
                    robot.leftIntake.setPosition(0);
                }
                gamepad1leftBumperTime.reset();
            }

            //Auto-turn Initializations:
            if (dpadLeft && gamepadDpadLeftTime.milliseconds() > 500) {
                autoTurn(-.5, .5);
                gamepadDpadLeftTime.reset();
            }
            if (dpadRight && gamepadDpadRightTime.milliseconds() > 500) {
                autoTurn(.5, .5);
                gamepadDpadRightTime.reset();
            }
            if (startButton && gamepadStartTime.milliseconds() > 500) {
                autoTurn(1, 0);
                gamepadStartTime.reset();
            }
            if (backButton && gamepadBackTime.milliseconds() > 500) {
                autoTurn(-1, 0);
                gamepadBackTime.reset();
            }


            //Telemetry:
            telemetry.addData("ArmPos", robot.armPos);
            telemetry.addData("ArmTarget", robot.armTarget);
            telemetry.addData("SlidesPos", robot.slidesPos);
            telemetry.addData("SlidesTarget", robot.slidesTarget);
            telemetry.addData("IntakeRotationPos", robot.intakeRotationPos);
            telemetry.addData("LeftIntakePos", robot.leftIntakePos);
            telemetry.addData("RightIntakePos", robot.rightIntakePos);
            telemetry.addData("FrontLeftPos", robot.frontLeftPos);
            telemetry.addData("FrontRightPos", robot.frontRightPos);
            telemetry.addData("BackLeftPos", robot.backLeftPos);
            telemetry.addData("BackRightPos", robot.backRightPos);
            telemetry.addData("Robot Heading", robot.botHeading);
            telemetry.addData("Y", -gamepad1.left_stick_y);
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("RX", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}