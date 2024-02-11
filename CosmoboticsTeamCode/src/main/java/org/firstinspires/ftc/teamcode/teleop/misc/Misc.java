package org.firstinspires.ftc.teamcode.teleop.misc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Misc {
    private Telemetry telemetry;
    public static ServoImplEx droneServo;
    public static ServoImplEx droneRot;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;

    public static final double launchingPosition = .7;
    public static final double fired = .15;
    public static final double standby = 0;
    public String lefthubcolor = "green";
    public String righthubcolor = "green";

    //Yellow, Purple, Green, White
    public int[] Colors = {Integer.parseInt("F2AF13", 16), Integer.parseInt("800080", 16), Integer.parseInt("00FF00", 16), Integer.parseInt("FFFFFF", 16)};

    public Misc (HardwareMap hardwareMap) {
        lefthubcolor = "green";
        righthubcolor = "green";
        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);
        droneServo = hardwareMap.get(ServoImplEx.class, "droneLauncher");
        droneRot = hardwareMap.get(ServoImplEx.class, "droneRotation");
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        droneServo.setDirection(Servo.Direction.FORWARD);
        droneRot.setDirection(Servo.Direction.REVERSE);
    }

    public void prepareDrone() {
        droneRot.setPosition(launchingPosition);
    }

    public void fireDrone() {
        if (droneRot.getPosition() == launchingPosition) {
            droneServo.setPosition(fired);
        }
    }

    public void resetDrone() {
        droneRot.setPosition(standby);
        droneRot.setPosition(standby);
    }

    public void signalLeftYellowPixel() {
        if (isRed) {
            CtrlHub.setConstant(Colors[0]);
            lefthubcolor = "yellow";
        } else {
            ExpHub.setConstant(Colors[0]);
            righthubcolor = "yellow";
        }
    }
    public void signalLeftPurplePixel() {
        if (isRed) {
            CtrlHub.setConstant(Colors[1]);
            lefthubcolor = "purple";
        } else {
            ExpHub.setConstant(Colors[1]);
            lefthubcolor = "purple";
        }
    }
    public void signalLeftGreenPixel() {
        if (isRed) {
            CtrlHub.setConstant(Colors[2]);
            lefthubcolor = "green";
        } else {
            ExpHub.setConstant(Colors[2]);
            lefthubcolor = "green";
        }
    }
    public void signalLeftWhitePixel() {
        if (isRed) {
            CtrlHub.setConstant(Colors[3]);
            lefthubcolor = "white";
        } else {
            ExpHub.setConstant(Colors[3]);
            lefthubcolor = "white";
        }
    }

    public void signalRightYellowPixel() {
        if (!isRed) {
            CtrlHub.setConstant(Colors[0]);
            lefthubcolor = "yellow";
        } else {
            ExpHub.setConstant(Colors[0]);
            righthubcolor = "yellow";
        }
    }
    public void signalRightPurplePixel() {
        if (!isRed) {
            CtrlHub.setConstant(Colors[1]);
            lefthubcolor = "purple";
        } else {
            ExpHub.setConstant(Colors[1]);
            lefthubcolor = "purple";
        }
    }
    public void signalRightGreenPixel() {
        if (!isRed) {
            CtrlHub.setConstant(Colors[2]);
            lefthubcolor = "green";
        } else {
            ExpHub.setConstant(Colors[2]);
            lefthubcolor = "green";
        }
    }
    public void signalRightWhitePixel() {
        if (!isRed) {
            CtrlHub.setConstant(Colors[3]);
            lefthubcolor = "white";
        } else {
            ExpHub.setConstant(Colors[3]);
            lefthubcolor = "white";
        }
    }

    public void update() {
        telemetry.addData("IsRedAlliance?", isRed);
        telemetry.addData("leftPixelLED", lefthubcolor);
        telemetry.addData("rightPixelLED", righthubcolor);
        telemetry.addData("DroneRotPos", droneRot.getPosition());
        telemetry.addData("DroneServoPos", droneServo.getPosition());
    }
}
