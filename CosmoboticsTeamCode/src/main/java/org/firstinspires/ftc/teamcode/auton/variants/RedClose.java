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
public class RedClose extends LinearOpMode {
    Auton RedClose;
    Transport transport;
    SampleMecanumDrive drive;

    public void followTrajectory(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
            transport.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        RedClose = new Auton(hardwareMap);
        transport = RedClose.transport();
        drive = RedClose.drive();
        AllianceStorage.isRed = true;
        RedClose.isLeftClaw = true; //YELLOW PIXEL IN THIS CLAW
        Pose2d startPose = new Pose2d(9, -63.25, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Board Auton:
        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {transport.two();})
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    transport.closeRightClaw();
                    transport.closeLeftClaw();
                })
                .lineToSplineHeading(RedClose.redLeftBoard)
                .addTemporalMarker(() -> {
                    transport.fullLeftClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()-> {transport.closeIntaking();})
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    transport.fullRightClaw();
                })
                .waitSeconds(.2)
                .build();
        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{transport.medIntaking();})
                .waitSeconds(4)
                .addTemporalMarker(()->{
                    transport.fullRightClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()->{transport.two();})
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    transport.closeRightClaw();
                    transport.closeLeftClaw();
                })
                .lineToSplineHeading(RedClose.redCenterBoard)
                .addTemporalMarker(()->{
                    transport.fullLeftClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()->{transport.reset();})
                .lineToSplineHeading(RedClose.redLeftBoard)
                .build();
        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    transport.closeRightClaw();
                    transport.closeLeftClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {transport.two();})
                .lineToSplineHeading(RedClose.redRightBoard)
                .addTemporalMarker(() -> {
                    transport.fullLeftClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()-> {transport.farIntaking();})
                .waitSeconds(4)
                .addTemporalMarker(()->{
                    transport.fullRightClaw();
                })
                .waitSeconds(.2)
                .addTemporalMarker(()->{transport.reset();})
                .lineToSplineHeading(RedClose.redLeftBoard)
                .build();

        while (opModeInInit() && !isStopRequested()) {
            RedClose.vision();
            telemetry.addData("Spike Pos", RedClose.spikePos);
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive() && !isStopRequested()) {
            switch (RedClose.spikePos) {
                case LEFT:
                    followTrajectory(leftBoard);
                    followTrajectory(RedClose.LeftBoardPark(true));
                    sleep(30000);
                case CENTER:
                    followTrajectory(centerBoard);
                    followTrajectory(RedClose.LeftBoardPark(true));
                    sleep(30000);
                case RIGHT:
                    followTrajectory(rightBoard);
                    followTrajectory(RedClose.LeftBoardPark(true));
                    sleep(30000);
            }

        }
    }
}

