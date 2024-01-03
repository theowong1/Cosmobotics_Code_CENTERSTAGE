package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;

@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    Drive drive;
    Transport transport;
    Misc misc;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        transport = new Transport(hardwareMap);
        misc = new Misc(hardwareMap);
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        transport.update(gamepad1, gamepad2);
        misc.update(gamepad1, gamepad2);
    }
}
