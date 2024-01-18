package org.firstinspires.ftc.teamcode.teleop.transport;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class Transport {
    public Toggle toggle;
    public static DcMotorEx slidesMotor;

    public static DcMotorEx armMotor;

    public static ServoImplEx rightIntake;

    public static ServoImplEx leftIntake;
    public static ServoImplEx intakeRotation;
    public static TouchSensor touchSensorClaw;
    public static TouchSensor zeroLimit;
    public static TouchSensor maxLimit;

    public static int mode = 0;
    public static boolean automode;

    //Safe, Intaking Ground, Intaking Med, Intaking Top, Parallels (.5, 1st, 1.5, 2nd, 2.5, 3rd, 3.5), Hang
    public static final int[] armPositions = {-30, 3630, 3590, 3550, 196, 351, 447, 552, 686, 784, 871, 1780};
    //Safe, Full
    public static final int[] slidesPositions = {0, 2935};

    //Safe, Deploy, Intaking, Parallels (.5, 1st, 1.5, 2nd, 2.5 - 3.5), Hang
    public static final double[] intakeRotPositions = {0, .9, .38, .85, .875, .9, .925, 1, .35};

    //Idx
    public static int leftClawPos = 0;
    public static int rightClawPos = 0;

    //Closed, Inter, Open
    public static final double[] clawPositions = {1, .95, .375};

    //Neutral, In-taking, Out-taking
    public enum TPos {
        //Reset:
        RESET("RESET", slidesPositions[0], armPositions[0], intakeRotPositions[0]),

        //Deploy:
        DEPLOY("DEPLOY", slidesPositions[0], armPositions[1], intakeRotPositions[1]),

        //Intaking Positions:
        INTAKING_GROUND("INTAKING_GROUND",  slidesPositions[1], armPositions[1], intakeRotPositions[2]),
        INTAKING_MED("INTAKING_MED",  slidesPositions[1], armPositions[2], intakeRotPositions[2]),
        INTAKING_TOP("INTAKING_STACK",  slidesPositions[1], armPositions[3], intakeRotPositions[2]),
        //Outtaking Positions:
        OUTTAKING_1("OUTTAKING_1",  slidesPositions[1], armPositions[4], intakeRotPositions[3]),

        OUTTAKING_2("OUTTAKING_2",  slidesPositions[1], armPositions[5], intakeRotPositions[4]),

        OUTTAKING_3("OUTTAKING_3",  slidesPositions[1], armPositions[6], intakeRotPositions[5]),

        OUTTAKING_4("OUTTAKING_4", slidesPositions[1], armPositions[7], intakeRotPositions[6]),
        OUTTAKING_5("OUTTAKING_4", slidesPositions[1], armPositions[8], intakeRotPositions[7]),
        OUTTAKING_6("OUTTAKING_4", slidesPositions[1], armPositions[9], intakeRotPositions[7]),
        OUTTAKING_7("OUTTAKING_4", slidesPositions[1], armPositions[10], intakeRotPositions[7]),

        HANG("HANG", slidesPositions[1], armPositions[11], intakeRotPositions[8])
        ;

        private final String debug;
        private final int slidesPosition;
        private final int armPosition;

        private final double intakeRotPosition;

        TPos(String debug, int slidePosition, int armPosition, double intakeRotPosition) {
            this.debug = debug;
            this.slidesPosition = slidePosition;
            this.armPosition = armPosition;
            this.intakeRotPosition = intakeRotPosition;
        }

        public String toString() { return debug; }
        public int slidesPos() { return slidesPosition; }
        public int armPos() { return armPosition; }
        public double intakeRotPos() { return intakeRotPosition; }
    }

    public TPos transportPos = TPos.RESET;

    public Transport (HardwareMap hardwareMap) {

        touchSensorClaw = hardwareMap.get(TouchSensor.class, "touchSensorClaw");
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");
        maxLimit = hardwareMap.get(TouchSensor.class, "maxLimit");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");
        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake");
        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake");

        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);
    }

    public void setTPos() {
        armMotor.setTargetPosition(transportPos.armPos());
        slidesMotor.setTargetPosition(transportPos.slidesPos());
        intakeRotation.setPosition(transportPos.intakeRotPos());
        leftIntake.setPosition(clawPositions[leftClawPos]);
        rightIntake.setPosition(clawPositions[rightClawPos]);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.left_trigger != 0) {
            leftClawPos = 2;
        } else {
            leftClawPos = 0;
        }
        if (gamepad1.right_trigger != 0) {
            rightClawPos = 2;
        } else {
            rightClawPos = 0;
        }
        if (gamepad1.back) { //TODO: USE TOGGLE LOGIC
            if (automode) {
                automode = false;
            } else {
                automode = true;
            }
        }
        if (gamepad2.dpad_right) {
            transportPos = TPos.RESET;
            mode = 0;
        }
        if (gamepad2.dpad_down) {
            transportPos = TPos.INTAKING_GROUND;
            mode = 1;
        }
        if (gamepad2.dpad_left) {
            transportPos = TPos.INTAKING_MED;
            mode = 1;
        }
        if (gamepad2.dpad_up) {
            transportPos = TPos.INTAKING_TOP;
            mode = 1;
        }
        if (gamepad2.b) {
            transportPos = TPos.OUTTAKING_1;
            mode = 2;
        }
        if (gamepad2.a) {
            transportPos = TPos.OUTTAKING_2;
            mode = 2;
        }
        if (gamepad2.x) {
            transportPos = TPos.OUTTAKING_3;
            mode = 2;
        }
        if (gamepad2.y) {
            transportPos = TPos.OUTTAKING_4;
            mode = 2;
        }
        if (gamepad2.left_bumper) {
            transportPos = TPos.OUTTAKING_5;
            mode = 2;
        }
        if (gamepad2.right_bumper) {
            transportPos = TPos.OUTTAKING_6;
            mode = 2;
        }
        if (gamepad2.left_stick_button) {
            transportPos = TPos.OUTTAKING_7;
            mode = 2;
        }
        if (gamepad2.right_stick_button) {
            transportPos = TPos.HANG;
            mode = 2;
        }
        if (automode) {
            if (mode == 0) {
                leftClawPos = 0;
                rightClawPos = 0;
            }
            else if (mode == 1) {
                leftClawPos = 2;
                rightClawPos = 2;
            }
            else {
                if (touchSensorClaw.isPressed()) {
                    leftClawPos = 2;
                    rightClawPos = 2;
                } else {
                    leftClawPos = 0;
                    rightClawPos = 0;
                }
            }
        }
        setTPos();
    }
}
