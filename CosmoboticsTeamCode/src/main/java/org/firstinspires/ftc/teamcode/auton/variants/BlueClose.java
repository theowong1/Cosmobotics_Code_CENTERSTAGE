package org.firstinspires.ftc.teamcode.auton.variants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.Auton;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.AllianceStorage;

@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
public class BlueClose extends LinearOpMode {
    Auton BlueClose;
    Transport transport;
    SampleMecanumDrive drive;

    public void followTrajectory(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            transport.update(telemetry);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        BlueClose = new Auton(hardwareMap);
        transport = BlueClose.transport();
        drive = BlueClose.drive();
        AllianceStorage.isRed = false;
        BlueClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
        Pose2d startPose = new Pose2d(15.25,63.25, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {transport.two();})
                .lineToSplineHeading(BlueClose.blueLeftBoard)
                .addTemporalMarker(() -> {
//                    if (BlueClose.leftClaw()) {
//                        BlueClose.fullLeftClaw();
//                    } else {
//                        BlueClose.fullRightClaw();
//                    }
                    transport.fullLeftClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()-> {transport.closeIntaking();})
                .waitSeconds(2)
                .lineToSplineHeading(BlueClose.blueRightBoard)
                .addTemporalMarker(()->{
                   transport.fullRightClaw();
                })
                .waitSeconds(.2)
                .build();
        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{transport.closeIntaking();})
                .waitSeconds(4)
                .addTemporalMarker(()->{
                    transport.fullRightClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()->{transport.two();})
                .lineToSplineHeading(BlueClose.blueCenterBoard)
                .addTemporalMarker(()->{
                        transport.fullLeftClaw();
                })
                .waitSeconds(.5)
                .addTemporalMarker(()->{transport.reset();})
                .lineToSplineHeading(BlueClose.blueRightBoard)
                .build();
        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {transport.two();})
                .lineToSplineHeading(BlueClose.blueRightBoard)
                .addTemporalMarker(()->{
                    transport.fullLeftClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()->{transport.farIntaking();})
                .waitSeconds(4)
                .addTemporalMarker(()->{
                    transport.fullRightClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()->{
                    transport.closeIntaking();
                })
                .build();

        while (opModeInInit() && !isStopRequested()) {
            BlueClose.vision();
            telemetry.addData("Spike Pos", BlueClose.spikePos);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive() && !isStopRequested()) {
            switch (BlueClose.spikePos) {
                case LEFT:
                    followTrajectory(leftBoard);
                    followTrajectory(BlueClose.RightBoardPark(false));
                    sleep(30000);
                case CENTER:
                    followTrajectory(centerBoard);
                    followTrajectory(BlueClose.RightBoardPark(false));
                    sleep(30000);
                case RIGHT:
                    followTrajectory(rightBoard);
                    followTrajectory(BlueClose.RightBoardPark(false));
                    sleep(30000);
            }

        }
    }
}
