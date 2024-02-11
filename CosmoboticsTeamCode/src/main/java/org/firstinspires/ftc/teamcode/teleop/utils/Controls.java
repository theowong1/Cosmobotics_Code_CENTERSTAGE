package org.firstinspires.ftc.teamcode.teleop.utils;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;

public class Controls {
    public Telemetry telemetry;
    private Drive drive;
    private Transport transport;
    private Misc misc;
    private GamepadEx gamepad1;
    private GamepadEx gamepad2;
    private TriggerReader autoTurnLeft;
    private TriggerReader autoTurnRight;
    private ToggleButtonReader resetIMU;
    private ToggleButtonReader slowmode;

    private ToggleButtonReader signalLeftYellowPixel;
    private ToggleButtonReader signalLeftPurplePixel;
    private ToggleButtonReader signalLeftGreenPixel;
    private ToggleButtonReader signalLeftWhitePixel;
    private ToggleButtonReader signalRightYellowPixel;
    private ToggleButtonReader signalRightPurplePixel;
    private ToggleButtonReader signalRightGreenPixel;
    private ToggleButtonReader signalRightWhitePixel;
    private ToggleButtonReader aimDrone;
    private ToggleButtonReader fireDrone;
    private ToggleButtonReader reset;
    private ToggleButtonReader half;
    private ToggleButtonReader one;
    private ToggleButtonReader oneHalf;
    private ToggleButtonReader two;
    private ToggleButtonReader twoHalf;
    private TriggerReader three;

    private TriggerReader threeHalf;
    private ToggleButtonReader deploy;
    private ToggleButtonReader medIntakingGround;
    private ToggleButtonReader farIntakingGround;
    private ToggleButtonReader closeIntakingMed;
//    private ToggleButtonReader farIntakingMed;
    private ToggleButtonReader closeIntakingTop;
//    private ToggleButtonReader farIntakingTop;
    private ToggleButtonReader leftClaw;

    private ToggleButtonReader rightClaw;

    private ToggleButtonReader stackOuttake;

    private ToggleButtonReader autoMode;
    private double leftClawMacro = 0;

    private double rightClawMacro = 0;

    public Controls(GamepadEx gamepad1, GamepadEx gamepad2, Drive drive, Transport transport, Misc misc) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.drive = drive;
        this.transport = transport;
        this.misc = misc;

        //TODO: Possibly Implement Manual Transport Mode
        //-------------------------------
        // *** GAMEPAD 1 CONTROLS ***
        //-------------------------------
        //Auto-turn:
        autoTurnLeft = new TriggerReader(gamepad1, GamepadKeys.Trigger.LEFT_TRIGGER);
        autoTurnRight = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        //Claw:
        leftClaw = new ToggleButtonReader(gamepad1, GamepadKeys.Button.LEFT_BUMPER);
        rightClaw = new ToggleButtonReader(gamepad1, GamepadKeys.Button.RIGHT_BUMPER);
        //Signal Left Pixels:
        signalLeftYellowPixel = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_UP);
        signalLeftPurplePixel = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_LEFT);
        signalLeftGreenPixel = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_DOWN);
        signalLeftWhitePixel = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_RIGHT);
        //IMU Reset & Far Intaking:
        resetIMU = new ToggleButtonReader(gamepad1, GamepadKeys.Button.BACK);
        slowmode = new ToggleButtonReader(gamepad1, GamepadKeys.Button.START);
        //Intaking & Reset:
        reset = new ToggleButtonReader(gamepad1, GamepadKeys.Button.A);
        deploy = new ToggleButtonReader(gamepad1, GamepadKeys.Button.X);
        medIntakingGround = new ToggleButtonReader(gamepad1, GamepadKeys.Button.B);
        farIntakingGround = new ToggleButtonReader(gamepad1, GamepadKeys.Button.Y);
        //Slowmode:
        // = new ToggleButtonReader(gamepad1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        fireDrone = new ToggleButtonReader(gamepad1, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        //-------------------------------
        // *** GAMEPAD 2 CONTROLS ***
        //-------------------------------
        //Tall Outtaking:
        three = new TriggerReader(gamepad2, GamepadKeys.Trigger.LEFT_TRIGGER);
        threeHalf = new TriggerReader(gamepad2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        //Stack Intaking:
        closeIntakingMed = new ToggleButtonReader(gamepad2, GamepadKeys.Button.LEFT_BUMPER);
        closeIntakingTop = new ToggleButtonReader(gamepad2, GamepadKeys.Button.RIGHT_BUMPER);
        //Semi-Tall Outtaking & Auto-Mode:
        twoHalf = new ToggleButtonReader(gamepad2, GamepadKeys.Button.START);
        autoMode = new ToggleButtonReader(gamepad2, GamepadKeys.Button.BACK);
        //Signal Right Pixels:
        signalRightYellowPixel = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_UP);
        signalRightPurplePixel = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_LEFT);
        signalRightGreenPixel = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_DOWN);
        signalRightWhitePixel = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_RIGHT);
        //Outtaking:
        half = new ToggleButtonReader(gamepad2, GamepadKeys.Button.A);
        one = new ToggleButtonReader(gamepad2, GamepadKeys.Button.X);
        oneHalf = new ToggleButtonReader(gamepad2, GamepadKeys.Button.B);
        two = new ToggleButtonReader(gamepad2, GamepadKeys.Button.Y);
        //Drone:
        aimDrone = new ToggleButtonReader(gamepad2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        stackOuttake = new ToggleButtonReader(gamepad2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        //= gamepad2.getLeftX();
        //= gamepad2.getLeftY();
        //= gamepad2.getRightX();
        //= gamepad2.getRightY();
    }

    public void update() {
        //*** DRIVE FUNCTIONS ***
        //Autoturn:
        if (autoTurnLeft.isDown()) {
            drive.autoTurnLeft();
        } else if (autoTurnRight.isDown()) {
            drive.autoTurnRight();
        } else {
            drive.normalTurn(gamepad1);
        }
        //Reset IMU:
        if (resetIMU.wasJustPressed()) {
            drive.resetImu();
        }
        //Slowmode:
        if (slowmode.getState()) {
            drive.activateSlowMode();
        } else {
            drive.deactivateSlowMode();
        }

        //*** TRANSPORT FUNCTIONS ***
        //Auto-mode:
        if (autoMode.getState()) {
            transport.autoModeOn();
        } else {
            transport.autoModeOff();
        }
        //Claw:
        if (stackOuttake.wasJustPressed()) {
            transport.medLeftClaw();
        }
        else if (leftClaw.getState()) {
            transport.fullLeftClaw();
        } else {
            transport.closeLeftClaw();
        }
        if (stackOuttake.wasJustPressed()) {
            transport.medRightClaw();
        }
        else if (rightClaw.getState()) {
            transport.fullRightClaw();
        } else {
            transport.closeRightClaw();
        }
        //Intaking:
        if (reset.wasJustPressed()) { transport.reset(); }
        if (deploy.wasJustPressed()) { transport.closeIntaking(); }
        if (medIntakingGround.wasJustPressed()) { transport.medIntaking(); }
        if (farIntakingGround.wasJustPressed()) { transport.farIntaking(); }
        if (closeIntakingMed.wasJustPressed()) { transport.closeMedStack(); }
        if (closeIntakingTop.wasJustPressed()) { transport.closeTopStack(); }
        //Outtaking:
        if (one.wasJustPressed()) { transport.one(); }
        if (oneHalf.wasJustPressed()) { transport.oneHalf(); }
        if (two.wasJustPressed()) { transport.two(); }
        if (twoHalf.wasJustPressed()) { transport.twoHalf(); }
        if (three.wasJustPressed()) { transport.three(); }
        if (threeHalf.wasJustPressed()) { transport.threeHalf(); }

        //*** MISC FUNCTIONS ***
        //Drone:
        if (aimDrone.getState()) {
            misc.prepareDrone();
        } else {
            misc.resetDrone();
        }
        if (fireDrone.wasJustPressed()) {
            misc.fireDrone();;
        }
        //Pixel Signaling:
        if (signalLeftYellowPixel.wasJustPressed()) {
            misc.signalLeftYellowPixel();
        }
        if (signalLeftPurplePixel.wasJustPressed()) {
            misc.signalLeftPurplePixel();
        }
        if (signalLeftGreenPixel.wasJustPressed()) {
            misc.signalLeftGreenPixel();
        }
        if (signalLeftWhitePixel.wasJustPressed()) {
            misc.signalLeftWhitePixel();
        }
        if (signalRightYellowPixel.wasJustPressed()) {
            misc.signalRightYellowPixel();
        }
        if (signalRightPurplePixel.wasJustPressed()) {
            misc.signalRightPurplePixel();
        }
        if (signalRightGreenPixel.wasJustPressed()) {
            misc.signalRightGreenPixel();
        }
        if (signalRightWhitePixel.wasJustPressed()) {
            misc.signalRightWhitePixel();
        }
        //Telemetry:
        telemetry.addData("Slowmode", slowmode.getState());
        telemetry.addData("Reset IMU", resetIMU.isDown());
        telemetry.addData("Autoturn Left", autoTurnLeft.isDown());
        telemetry.addData("Autoturn Right", autoTurnRight.isDown());
        telemetry.addData("Drone Ready", aimDrone.getState());
        telemetry.addData("Drone Fired", fireDrone.getState());
        telemetry.addData("leftClaw", leftClaw.getState());
        telemetry.addData("rightClaw", rightClaw.getState());
        telemetry.addData("Automode", autoMode.getState());
    }
}
