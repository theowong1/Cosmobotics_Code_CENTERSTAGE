package org.firstinspires.ftc.teamcode.auton;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.pipeline.SpikePosDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class Auton {
    private SpikePosDetector.SPIKE_POS spikePos;
    private SpikePosDetector pipeline;
    private VisionPortal vision;
    private Transport transport;
    private SampleMecanumDrive drive;
    public final Pose2d standardPose = new Pose2d(0, 0, Math.toRadians(0));
    private Pose2d leftSpikePose;
    private Pose2d centerSpikePose;
    private Pose2d rightSpikePose;

    public Auton(HardwareMap hardwareMap, Pose2d leftSpikePose, Pose2d centerSpikePose, Pose2d rightSpikePose) {
        this.leftSpikePose = leftSpikePose;
        this.centerSpikePose = centerSpikePose;
        this.rightSpikePose = rightSpikePose;

        pipeline = new SpikePosDetector();
        spikePos = pipeline.getType();
        vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessors(pipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        drive = new SampleMecanumDrive(hardwareMap);
        transport = new Transport(hardwareMap);
    }

    public SampleMecanumDrive drive() {
        return drive;
    }
    public final void BlueSpikeAuton() {
        drive.setPoseEstimate(standardPose);
        switch (spikePos) {
            case LEFT:
                TrajectorySequence leftSpike = drive.trajectorySequenceBuilder(standardPose)
                        //TODO: Write Trajectory
                        .build();
                drive.followTrajectorySequence(leftSpike);
                drive.setPoseEstimate(leftSpikePose);
            case CENTER:
                TrajectorySequence centerSpike = drive.trajectorySequenceBuilder(standardPose)
                        //TODO: Write Trajectory
                        .build();
                drive.followTrajectorySequence(centerSpike);
                drive.setPoseEstimate(centerSpikePose);
            case RIGHT:
                TrajectorySequence rightSpike = drive.trajectorySequenceBuilder(standardPose)
                        //TODO: Write Trajectory
                        .build();
                drive.setPoseEstimate(rightSpikePose);
                drive.followTrajectorySequence(rightSpike);
        }
    }
    public final void RedSpikeAuton() {
        drive.setPoseEstimate(standardPose);
        switch (spikePos) {
            case LEFT:
                TrajectorySequence leftSpike = drive.trajectorySequenceBuilder(standardPose)
                        //TODO: Write Trajectory
                                .build();
                drive.followTrajectorySequence(leftSpike);
                drive.setPoseEstimate(leftSpikePose);
            case CENTER:
                TrajectorySequence centerSpike = drive.trajectorySequenceBuilder(standardPose)
                        //TODO: Write Trajectory
                                .build();
                drive.followTrajectorySequence(centerSpike);
                drive.setPoseEstimate(centerSpikePose);
            case RIGHT:
                TrajectorySequence rightSpike = drive.trajectorySequenceBuilder(standardPose)
                        //TODO: Write Trajectory
                        .build();
                drive.setPoseEstimate(rightSpikePose);
                drive.followTrajectorySequence(rightSpike);
        }
    }

    public void BoardAuton(TrajectorySequence leftBoard, TrajectorySequence centerBoard, TrajectorySequence rightBoard) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftBoard);
            case CENTER:
                drive.followTrajectorySequence(centerBoard);
            case RIGHT:
                drive.followTrajectorySequence(rightBoard);
        }
    }


    public void spikeToStack(TrajectorySequence leftStack, TrajectorySequence centerStack, TrajectorySequence rightStack) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftStack);
            case CENTER:
                drive.followTrajectorySequence(centerStack);
            case RIGHT:
                drive.followTrajectorySequence(rightStack);
        }
    }

    public void ToStack(TrajectorySequence leftStack, TrajectorySequence centerStack, TrajectorySequence rightStack) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftStack);
            case CENTER:
                drive.followTrajectorySequence(centerStack);
            case RIGHT:
                drive.followTrajectorySequence(rightStack);
        }
    }

    public void FromStack(TrajectorySequence leftBoard, TrajectorySequence centerBoard, TrajectorySequence rightBoard) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftBoard);
            case CENTER:
                drive.followTrajectorySequence(centerBoard);
            case RIGHT:
                drive.followTrajectorySequence(rightBoard);
        }
    }

    public final void BoardPark(TrajectorySequence leftBoardPark, TrajectorySequence centerBoardPark, TrajectorySequence rightBoardPark) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftBoardPark);
            case CENTER:
                drive.followTrajectorySequence(centerBoardPark);
            case RIGHT:
                drive.followTrajectorySequence(rightBoardPark);
        }
    }

    public void Reset() {
        transport.transportPos = Transport.TPos.RESET;
        transport.setTPos();
    }

    public void Outtaking() {
        transport.transportPos = Transport.TPos.OUTTAKING_1;
        transport.setTPos();
    }

    public void IntakingBottom() {
        transport.transportPos = Transport.TPos.INTAKING_GROUND;
        transport.setTPos();
    }

    public void IntakingMed() {
        transport.transportPos = Transport.TPos.INTAKING_MED;
        transport.setTPos();
    }

    public void IntakingTop() {
        transport.transportPos = Transport.TPos.INTAKING_TOP;
    }

    public void PlaceLeftPixel() {
        transport.leftClawPos = 2;
        transport.setTPos();
    }

    public void PlaceRightPixel() {
        transport.rightClawPos = 2;
        transport.setTPos();
    }

    public void PlaceLeftPixelHolding() {
        transport.leftClawPos = 1;
        transport.setTPos();
    }

    public void PlaceRightPixelHolding() {
        transport.rightClawPos = 1;
        transport.setTPos();
    }

    public void IntakeLeftPixel() {
        transport.leftClawPos = 0;
        transport.setTPos();
    }

    public void IntakeRightPixel() {
        transport.rightClawPos = 0;
        transport.setTPos();
    }
}