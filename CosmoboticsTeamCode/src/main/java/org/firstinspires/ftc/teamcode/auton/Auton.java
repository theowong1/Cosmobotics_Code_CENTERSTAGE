package org.firstinspires.ftc.teamcode.auton;
import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.pipeline.SpikePosDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class Auton {
    public SpikePosDetector.SPIKE_POS spikePos;
    private SpikePosDetector pipeline;
    private VisionPortal vision;
    private Transport transport;
    private SampleMecanumDrive drive;
    public Pose2d blueLeftBoard;
    public Pose2d blueCenterBoard;
    public Pose2d blueRightBoard;
    public Pose2d redLeftBoard;
    public Pose2d redCenterBoard;
    public Pose2d redRightBoard;
    public boolean isLeftClaw;

    public Auton(HardwareMap hardwareMap) {

        //TODO: Set These Poses
        blueLeftBoard = new Pose2d(45, 31, Math.toRadians(180));
        blueCenterBoard = new Pose2d(45, 28, Math.toRadians(180));
        blueRightBoard = new Pose2d(45, 25, Math.toRadians(180));
        redLeftBoard = new Pose2d(45, -25, Math.toRadians(190));
        redCenterBoard = new Pose2d(45, -28, Math.toRadians(190));
        redRightBoard = new Pose2d(45, -31, Math.toRadians(190));

        pipeline = new SpikePosDetector();
        spikePos = pipeline.getType();
        vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessors(pipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        drive = new SampleMecanumDrive(hardwareMap);
        transport = new Transport(hardwareMap);
    }
    public void vision() {
        spikePos = pipeline.getType();
    }
    public SampleMecanumDrive drive() {
        return drive;
    }

    public Transport transport() {
        return transport;
    }

    public Boolean leftClaw() {
        return isLeftClaw;
    }

    public TrajectorySequence LeftBoardToLeftStack(Boolean Reverse) {
        if (!isRed) {
            TrajectorySequence blueLeftBoardToLeftStack = drive.trajectorySequenceBuilder(blueLeftBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return blueLeftBoardToLeftStack;
        } else {
            TrajectorySequence redLeftBoardToLeftStack = drive.trajectorySequenceBuilder(redLeftBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return redLeftBoardToLeftStack;
        }
    }

    public TrajectorySequence CenterBoardToLeftStack(Boolean Reverse) {
        if (!isRed) {
            TrajectorySequence blueCenterBoardToLeftStack = drive.trajectorySequenceBuilder(blueCenterBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return blueCenterBoardToLeftStack;
        } else {
            TrajectorySequence redCenterBoardToLeftStack = drive.trajectorySequenceBuilder(redCenterBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return redCenterBoardToLeftStack;
        }
    }

    public TrajectorySequence RightBoardToLeftStack(Boolean Reverse) {
        if (!isRed) {
            TrajectorySequence blueRightBoardToLeftStack = drive.trajectorySequenceBuilder(blueRightBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
           return blueRightBoardToLeftStack;
        } else {
            TrajectorySequence redRightBoardToLeftStack = drive.trajectorySequenceBuilder(redRightBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return redRightBoardToLeftStack;
        }
    }

    public TrajectorySequence LeftBoardToRightStack(Boolean Reverse) {
        if (!isRed) {
            TrajectorySequence blueLeftBoardToRightStack = drive.trajectorySequenceBuilder(blueLeftBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return blueLeftBoardToRightStack;
        } else {
            TrajectorySequence redLeftBoardToRightStack = drive.trajectorySequenceBuilder(redLeftBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return redLeftBoardToRightStack;
        }
    }

    public TrajectorySequence CenterBoardToRightStack(Boolean Reverse) {
        if (!isRed) {
            TrajectorySequence blueCenterBoardToRightStack = drive.trajectorySequenceBuilder(blueCenterBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return blueCenterBoardToRightStack;
        } else {
            TrajectorySequence redCenterBoardToRightStack = drive.trajectorySequenceBuilder(redCenterBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return redCenterBoardToRightStack;
        }
    }

    public TrajectorySequence RightBoardToRightStack(Boolean Reverse) {
        if (!isRed) {
            TrajectorySequence blueRightBoardToRightStack = drive.trajectorySequenceBuilder(blueRightBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return blueRightBoardToRightStack;
        } else {
            TrajectorySequence redRightBoardToRightStack = drive.trajectorySequenceBuilder(redRightBoard)
                    .setReversed(Reverse)
                    //TODO: WRITE TRAJECTORY
                    .build();
            return redRightBoardToRightStack;
        }
    }

    public TrajectorySequence LeftBoardPark(Boolean goLeft) {
        if (goLeft && isRed) {
            TrajectorySequence redLeftBoardParkLeft = drive.trajectorySequenceBuilder(redLeftBoard)
                    .UNSTABLE_addTemporalMarkerOffset(.5, ()->{transport.reset();})
                    .strafeTo(new Vector2d(45, -4))
                    .strafeTo(new Vector2d(63.25, -4))
                    .build();
            return redLeftBoardParkLeft;
        } else if (goLeft && !isRed) {
            TrajectorySequence blueLeftBoardParkLeft = drive.trajectorySequenceBuilder(blueLeftBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, 63.25))
                    .strafeTo(new Vector2d(63.25, 63.25))
                    .build();
            return blueLeftBoardParkLeft;
        } else if (!goLeft && isRed){
            TrajectorySequence redLeftBoardParkRight = drive.trajectorySequenceBuilder(redLeftBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, -63.25))
                    .strafeTo(new Vector2d(63.25, -63.25))
                    .build();
            return redLeftBoardParkRight;
        } else {
            TrajectorySequence blueLeftBoardParkRight = drive.trajectorySequenceBuilder(blueLeftBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, 7))
                    .strafeTo(new Vector2d(63.25, 7))
                    .build();
            return blueLeftBoardParkRight;
        }
    }

    public TrajectorySequence CenterBoardPark(Boolean goLeft) {
        if (goLeft && isRed) {
            TrajectorySequence redCenterBoardParkLeft = drive.trajectorySequenceBuilder(redCenterBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, -7))
                    .strafeTo(new Vector2d(63.25, -7))
                    .build();
            return redCenterBoardParkLeft;
        } else if (goLeft && !isRed) {
            TrajectorySequence blueCenterBoardParkLeft = drive.trajectorySequenceBuilder(blueCenterBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, 63.25))
                    .strafeTo(new Vector2d(63.25, 63.25))
                    .build();
            return blueCenterBoardParkLeft;
        } else if (!goLeft && isRed){
            TrajectorySequence redCenterBoardParkRight = drive.trajectorySequenceBuilder(redCenterBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, -63.25))
                    .strafeTo(new Vector2d(63.25, -63.25))
                    .build();
            return redCenterBoardParkRight;
        } else {
            TrajectorySequence blueCenterBoardParkRight = drive.trajectorySequenceBuilder(blueCenterBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, 7))
                    .strafeTo(new Vector2d(63.25, 7))
                    .build();
            return blueCenterBoardParkRight;
        }
    }

    public TrajectorySequence RightBoardPark(Boolean goLeft) {
        if (goLeft && isRed) {
            TrajectorySequence redRightBoardParkLeft = drive.trajectorySequenceBuilder(redRightBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, -7))
                    .strafeTo(new Vector2d(63.25, -7))
                    .build();
            return redRightBoardParkLeft;
        } else if (goLeft && !isRed) {
            TrajectorySequence blueRightBoardParkLeft = drive.trajectorySequenceBuilder(blueRightBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, 63.25))
                    .strafeTo(new Vector2d(63.25, 63.25))
                    .build();
            return blueRightBoardParkLeft;
        } else if (!goLeft && isRed){
            TrajectorySequence redRightBoardParkRight = drive.trajectorySequenceBuilder(redRightBoard)
                    .addTemporalMarker(()->{transport.reset();})
                    .strafeTo(new Vector2d(45, -63.25))
                    .strafeTo(new Vector2d(63.25, -63.25))
                    .build();
            return redRightBoardParkRight;
        } else {
            TrajectorySequence blueRightBoardParkRight = drive.trajectorySequenceBuilder(blueRightBoard)
                    .UNSTABLE_addTemporalMarkerOffset(.5, ()->{transport.reset();})
                    .strafeTo(new Vector2d(45, 7))
                    .strafeTo(new Vector2d(63.25, 7))
                    .build();
            return blueRightBoardParkRight;
        }
    }
}