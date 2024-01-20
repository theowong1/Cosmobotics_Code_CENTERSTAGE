package org.firstinspires.ftc.teamcode.teleop.transport;

import com.arcrobotics.ftclib.controller.PIDController;
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
    private Toggle automode;
    private Toggle intake;
    private DcMotorEx slidesMotor;

    private DcMotorEx armMotor;

    private ServoImplEx rightIntake;

    private ServoImplEx leftIntake;
    private ServoImplEx intakeRotation;
    private TouchSensor clawSensor;
    private TouchSensor zeroLimit;
    private PIDController slidesController;

    private final double slidesp = .022, slidesi = 0, slidesd = 0.00038;
    private int slidesTarget = 0;
    private PIDController armController;

    private final double armp = 0.006, armi = 0, armd = 0.0004; //TODO: Tune in Relation to Slides
    private double f = 0.06;
    private int armTarget = 0;

    private final double arm_ticks_in_degrees = 1425.1 / 1800;

    private int mode = 0;

    //Safe, Intaking Ground, Intaking Med, Intaking Top, Parallels (.5, 1st, 1.5, 2nd, 2.5, 3rd, 3.5), Hang
    public static final int[] armPositions = {0, 3500, 3400, 3300, 300, 450, 600, 750, 900, 1050, 1200, 1780};

    //Safe, Extended
    public static final int[] slidesPositions = {0, 3000};
    //Safe, Deploy, Intaking, Intaking Off-Ground, Parallels (.5, 1st, 1.5, 2 - 3.5), Hang
    public static final double[] intakeRotPositions = {.05, .9, .38, .42, .85, .9, .95, 1, .35};

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
        INTAKING_GROUND("INTAKING_GROUND", slidesPositions[1], armPositions[1], intakeRotPositions[2]),
        INTAKING_MED("INTAKING_MED", slidesPositions[1], armPositions[2], intakeRotPositions[3]),
        INTAKING_TOP("INTAKING_STACK", slidesPositions[1], armPositions[3], intakeRotPositions[3]),
        //Outtaking Positions:
        OUTTAKING_1("OUTTAKING_1", slidesPositions[1], armPositions[4], intakeRotPositions[4]),

        OUTTAKING_2("OUTTAKING_2", slidesPositions[1], armPositions[5], intakeRotPositions[5]),

        OUTTAKING_3("OUTTAKING_3", slidesPositions[1], armPositions[6], intakeRotPositions[6]),

        OUTTAKING_4("OUTTAKING_4", slidesPositions[1], armPositions[7], intakeRotPositions[7]),
        OUTTAKING_5("OUTTAKING_4", slidesPositions[1], armPositions[8], intakeRotPositions[7]),
        OUTTAKING_6("OUTTAKING_4", slidesPositions[1], armPositions[9], intakeRotPositions[7]),
        OUTTAKING_7("OUTTAKING_4", slidesPositions[1], armPositions[10], intakeRotPositions[7]),

        HANG("HANG", slidesPositions[1], armPositions[11], intakeRotPositions[8])
        ;

        private final String debug;
        private final int armPosition;

        private final int slidesPosition;
        private final double intakeRotPosition;

        TPos(String debug, int slidesPosition, int armPosition, double intakeRotPosition) {
            this.debug = debug;
            this.slidesPosition = slidesPosition;
            this.armPosition = armPosition;
            this.intakeRotPosition = intakeRotPosition;
        }

        public String toString() { return debug; }
        public int armPos() { return armPosition; }

        public int slidesPos() {return slidesPosition; }
        public double intakeRotPos() { return intakeRotPosition; }
    }

    public TPos transportPos = TPos.RESET;

    public Transport (HardwareMap hardwareMap) {
        automode = new Toggle(false);
        intake = new Toggle(false);

        clawSensor = hardwareMap.get(TouchSensor.class, "clawSensor");
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");

        slidesController = new PIDController(slidesp, slidesi, slidesd);

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesController.setPID(slidesp, slidesi, slidesd);


        armController = new PIDController(armp, armi, armd);

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController.setPID(armp, armi, armd);


        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");
        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake");
        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake");

        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);
    }


    public void setTPos() {
        intakeRotation.setPosition(transportPos.intakeRotPos());
        if (slidesMotor.getCurrentPosition() < 10) {
            armTarget = transportPos.armPos();
            while (Math.abs(armTarget - armMotor.getCurrentPosition()) > 50) { //TODO: Finite State Mechanics

            }
            slidesTarget = transportPos.slidesPos();
        } else { //TODO: FINITE STATE MECHANICS
            slidesTarget = 0;
            while (Math.abs(slidesTarget - slidesMotor.getCurrentPosition()) > 10) {

            }
            armTarget = transportPos.armPos();
            while (Math.abs(armTarget - armMotor.getCurrentPosition()) > 50) {

            }
            slidesTarget = transportPos.slidesPos();
        }
        leftIntake.setPosition(clawPositions[leftClawPos]);
        rightIntake.setPosition(clawPositions[rightClawPos]);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        int armPos = armMotor.getCurrentPosition();
        double armpid = armController.calculate(armPos, armTarget);
        double armff = Math.cos(Math.toRadians(armTarget / arm_ticks_in_degrees)) * f;

        double armPower = armpid + armff;

        armMotor.setPower(armPower);

        int slidesPos = slidesMotor.getCurrentPosition();
        double slidespid = slidesController.calculate(slidesPos, slidesTarget);

        double slidesPower = slidespid;

        slidesMotor.setPower(slidesPower);

//        automode.update(gamepad1.back);
        intake.update(gamepad1.dpad_up);
        if (zeroLimit.isPressed()) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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

        if (gamepad1.dpad_right) {
            transportPos = TPos.RESET;
            mode = 0;
        }
        if (gamepad1.dpad_down) {
            transportPos = TPos.INTAKING_GROUND;
            mode = 1;
        }
        if (gamepad1.dpad_left) {
            transportPos = TPos.INTAKING_MED;
            mode = 1;
        }
        if (gamepad1.dpad_up) {
            transportPos = TPos.INTAKING_TOP;
            mode = 1;
        }
        if (gamepad1.b) {
            transportPos = TPos.OUTTAKING_1;
            mode = 2;
        }
        if (gamepad1.a) {
            transportPos = TPos.OUTTAKING_2;
            mode = 2;
        }
        if (gamepad1.x) {
            transportPos = TPos.OUTTAKING_3;
            mode = 2;
        }
        if (gamepad1.y) {
            transportPos = TPos.OUTTAKING_4;
            mode = 2;
        }
        if (gamepad1.left_bumper) {
            transportPos = TPos.OUTTAKING_5;
            mode = 2;
        }
        if (gamepad1.right_bumper) {
            transportPos = TPos.OUTTAKING_6;
            mode = 2;
        }
//        if (gamepad2.left_stick_button) {
//            transportPos = TPos.OUTTAKING_7;
//            mode = 2;
//        }
//        if (gamepad2.right_stick_button) {
//            transportPos = TPos.HANG;
//            mode = 2;
//        }
        if (intake.value() == true) {
            if (transportPos == TPos.DEPLOY) {
                transportPos = TPos.INTAKING_GROUND;
                mode = 1;
            } else {
                transportPos = TPos.DEPLOY;
                mode = 1;
            }
        }
//        if (automode.value() == true) {
//            if (mode == 0) {
//                leftClawPos = 0;
//                rightClawPos = 0;
//            }
//            else if (mode == 1) {
//                leftClawPos = 2;
//                rightClawPos = 2;
//            }
//            else {
//                if (clawSensor.isPressed()) {
//                    leftClawPos = 2;
//                    rightClawPos = 2;
//                } else {
//                    leftClawPos = 0;
//                    rightClawPos = 0;
//                }
//            }
//        }
        setTPos();
    }
}
