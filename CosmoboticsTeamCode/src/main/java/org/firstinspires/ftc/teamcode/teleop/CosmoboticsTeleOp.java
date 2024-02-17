package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.Controls;

@Config
@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    Drive drive;
    Transport transport;
    Misc misc;

    Controls controls;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        transport = new Transport(hardwareMap);
        misc = new Misc(hardwareMap);
        controls = new Controls(gamepad1, gamepad2, drive, transport, misc);
    }

    @Override
    public void loop() {
        controls.update(telemetry);
        drive.update(telemetry);
        transport.update(telemetry);
        misc.update(telemetry);
        telemetry.update();
    }
}
