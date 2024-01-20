package org.firstinspires.ftc.teamcode.teleop.misc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

import java.util.List;

public class Misc {
    public Toggle Drone;
    public static ServoImplEx droneServo;
    public static ServoImplEx droneRot;
    public List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    public LynxModule CtrlHub = allHubs.get(0);

    public LynxModule ExpHub = allHubs.get(1);

    //Yellow, Purple, Green, White
    public int[] Colors = {Integer.parseInt("FFFF00", 16), Integer.parseInt("800080", 16), Integer.parseInt("00FF00", 16), Integer.parseInt("FFFFFF", 16)};

    public Misc (HardwareMap hardwareMap) {
        Drone = new Toggle(false);
        droneServo = hardwareMap.get(ServoImplEx.class, "droneLauncher");
        droneRot = hardwareMap.get(ServoImplEx.class, "droneRotation");

        droneServo.setDirection(Servo.Direction.FORWARD);
        droneRot.setDirection(Servo.Direction.FORWARD);

    }
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        Drone.update(gamepad1.start);
        if (Drone.value() == true) {
            droneRot.setPosition(.8);
            droneServo.setPosition(1);
        } else {
            droneRot.setPosition(0);
            droneRot.setPosition(0);
        }
//        if (gamepad1.dpad_up) {
//            ExpHub.setConstant(Colors[0]);
//        }
//        if (gamepad1.dpad_left) {
//            ExpHub.setConstant(Colors[1]);
//        }
//        if (gamepad1.dpad_down) {
//            ExpHub.setConstant(Colors[2]);
//        }
//        if (gamepad1.dpad_right) {
//            ExpHub.setConstant(Colors[3]);
//        }
//        if (gamepad1.y) {
//            CtrlHub.setConstant(Colors[0]);
//        }
//        if (gamepad1.x) {
//            CtrlHub.setConstant(Colors[1]);
//        }
//        if (gamepad1.a) {
//            CtrlHub.setConstant(Colors[2]);
//        }
//        if (gamepad1.b) {
//            CtrlHub.setConstant(Colors[3]);
//        }
    }
}
