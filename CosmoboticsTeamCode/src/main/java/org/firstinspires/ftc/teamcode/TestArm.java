package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestArm extends LinearOpMode {
    DcMotor armMotor;
    public TestArm() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
