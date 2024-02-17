package org.firstinspires.ftc.teamcode.teleop.utils;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;

public class Controls {
    public Telemetry telemetry;
    private Drive drive;
    private Transport transport;
    private Misc misc;
    private TriggerReader autoTurnLeft;
    private TriggerReader autoTurnRight;
    private ToggleButtonReader resetIMU;
    private ToggleButtonReader slowmode;

    private double leftYGPixel;
    private double leftPWPixel;
    private double rightYGPixel;
    private double rightPWPixel;
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
    private double x;
    private double y;
    private double rx;
    private GamepadEx gamepadEx1;

    private GamepadEx gamepadEx2;

    public Controls(Gamepad gamepad1, Gamepad gamepad2, Drive drive, Transport transport, Misc misc) {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        this.drive = drive;
        this.transport = transport;
        this.misc = misc;

        //TODO: Possibly Implement Manual Transport Mode
        //-------------------------------
        // *** GAMEPAD 1 CONTROLS ***
        //-------------------------------
        //Auto-turn:
        autoTurnLeft = new TriggerReader(gamepadEx1, GamepadKeys.Trigger.LEFT_TRIGGER);
        autoTurnRight = new TriggerReader(gamepadEx1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        //Claw:
        leftClaw = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER);
        rightClaw = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER);
        //Signal Left Pixels:
//       = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_UP);
//       = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_LEFT);
//       = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_DOWN);
//       = new ToggleButtonReader(gamepad1, GamepadKeys.Button.DPAD_RIGHT);
        //IMU Reset & Far Intaking:
        resetIMU = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.BACK);
        slowmode = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.START);
        //Intaking & Reset:
        reset = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.A);
        deploy = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.X);
        medIntakingGround = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.B);
        farIntakingGround = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.Y);
        //Slowmode:
        // = new ToggleButtonReader(gamepad1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        autoMode = new ToggleButtonReader(gamepadEx1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        //Drive:
        x = 0;
        y = 0;
        rx = 0;
        // = gamepadEx1.getRightY();

        //-------------------------------
        // *** GAMEPAD 2 CONTROLS ***
        //-------------------------------
        //Tall Outtaking:
        three = new TriggerReader(gamepadEx2, GamepadKeys.Trigger.LEFT_TRIGGER);
        threeHalf = new TriggerReader(gamepadEx2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        //Stack Intaking:
        closeIntakingMed = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER);
        closeIntakingTop = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER);
        //Semi-Tall Outtaking & Auto-Mode:
        stackOuttake = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.START);
        twoHalf = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.BACK);
        //Signal Right Pixels:
//      = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_UP);
//      = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_LEFT);
//      = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_DOWN);
//      = new ToggleButtonReader(gamepad2, GamepadKeys.Button.DPAD_RIGHT);
        //Outtaking:
        half = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.A);
        one = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.X);
        oneHalf = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.B);
        two = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.Y);
        //Drone:
        aimDrone = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        fireDrone = new ToggleButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        //Signal Pixels:
        leftPWPixel = 0;
        leftYGPixel = 0;
        rightPWPixel = 0;
        rightYGPixel = 0;
    }

    public void update(Telemetry telemetry) {
        //TODO: ORGANIZE BY GAMEPAD, SWITCH TO GAMEPADEXRUN READERS
        //*** DRIVE FUNCTIONS ***
        drive.drive(x, y);
        //TODO: Switching to and from Drive to Sample Mecanum Drive and AutoLocalizing?
        //Autoturn:
        if (autoTurnLeft.isDown()) { drive.autoTurnLeft(); }
        else if (autoTurnRight.isDown()) { drive.autoTurnRight(); }
        else { drive.normalTurn(rx); }
        //Reset IMU:
        if (resetIMU.wasJustPressed()) { drive.resetImu(); }
        //Slowmode:
        if (slowmode.getState()) { drive.activateSlowMode(); }
        else { drive.deactivateSlowMode(); }

        //*** TRANSPORT FUNCTIONS ***
        //Auto-mode:
        if (autoMode.getState()) { transport.autoModeOn(); }
        else { transport.autoModeOff(); }
        //Left-Claw:
        if (stackOuttake.wasJustPressed()) { transport.medLeftClaw(); }
        else if (leftClaw.getState()) { transport.fullLeftClaw(); }
        else { transport.closeLeftClaw(); }
        //Right-Claw:
        if (stackOuttake.wasJustPressed()) { transport.medRightClaw(); }
        else if (rightClaw.getState()) { transport.fullRightClaw(); }
        else { transport.closeRightClaw(); }
        //Intaking:
        if (reset.isDown()) { transport.reset(); }
        if (deploy.isDown()) { transport.closeIntaking(); }
        if (medIntakingGround.isDown()) { transport.medIntaking(); }
        if (farIntakingGround.isDown()) { transport.farIntaking(); }
        if (closeIntakingMed.isDown()) { transport.closeMedStack(); }
        if (closeIntakingTop.isDown()) { transport.closeTopStack(); }
        //Outtaking:
        if (one.isDown()) { transport.one(); }
        if (oneHalf.isDown()) { transport.oneHalf(); }
        if (two.isDown()) { transport.two(); }
        if (twoHalf.isDown()) { transport.twoHalf(); }
        if (three.isDown()) { transport.three(); }
        if (threeHalf.isDown()) { transport.threeHalf(); }

        //*** MISC FUNCTIONS ***
        //Drone:
        if (aimDrone.isDown()) { misc.prepareDrone(); }
        else { misc.resetDrone(); }
        if (fireDrone.isDown()) { misc.fireDrone(); }
        //Pixel Signaling:
        if (leftYGPixel > .9) { misc.signalLeftYellowPixel(); }
        if (leftPWPixel < -.9) { misc.signalLeftPurplePixel(); }
        if (leftYGPixel < -.9) { misc.signalLeftGreenPixel(); }
        if (leftPWPixel > .9) { misc.signalLeftWhitePixel(); }
        if (rightYGPixel > .9) { misc.signalRightYellowPixel(); }
        if (rightPWPixel < -.9) { misc.signalRightPurplePixel(); }
        if (rightYGPixel < -.9) { misc.signalRightGreenPixel();}
        if (rightPWPixel > .9) { misc.signalRightWhitePixel(); }

        //*** Update Readers & Joysticks ***
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();
        x = gamepadEx1.getLeftX();
        y = gamepadEx1.getLeftY();
        rx = gamepadEx1.getRightX();
        leftPWPixel = gamepadEx2.getLeftX();
        leftYGPixel = gamepadEx2.getLeftY();
        rightPWPixel = gamepadEx2.getRightX();
        rightYGPixel = -gamepadEx2.getRightY();
        //Telemetry:
        telemetry.addData("strafeSpeed", x);
        telemetry.addData("forwardSpeed", y);
        telemetry.addData("turnSpeed/turn", rx);
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
