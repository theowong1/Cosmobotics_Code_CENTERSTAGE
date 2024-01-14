package org.firstinspires.ftc.teamcode.auton;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.pipeline.SpikePosDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

public class Auton {
    private SpikePosDetector.SPIKE_POS spikePos;
    private SpikePosDetector pipeline;
    private VisionPortal vision;
    private Transport transport;
    private SampleMecanumDrive drive;
    private Pose2d startPose;
    public Auton(HardwareMap hardwareMap, Pose2d startPose) {
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
        drive.setPoseEstimate(startPose);
    }

    public void SpikeAuton(TrajectorySequence leftSpike, TrajectorySequence centerSpike, TrajectorySequence rightSpike) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftSpike);
            case CENTER:
                drive.followTrajectorySequence(centerSpike);
            case RIGHT:
                drive.followTrajectorySequence(rightSpike);
        }
    }
    public void SpikePark(TrajectorySequence leftSpikePark, TrajectorySequence centerSpikePark, TrajectorySequence rightSpikePark) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftSpikePark);
            case CENTER:
                drive.followTrajectorySequence(centerSpikePark);
            case RIGHT:
                drive.followTrajectorySequence(rightSpikePark);
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

    public void BoardPark(TrajectorySequence leftBoardPark, TrajectorySequence centerBoardPark, TrajectorySequence rightBoardPark) {
        switch (spikePos) {
            case LEFT:
                drive.followTrajectorySequence(leftBoardPark);
            case CENTER:
                drive.followTrajectorySequence(centerBoardPark);
            case RIGHT:
                drive.followTrajectorySequence(rightBoardPark);
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
}