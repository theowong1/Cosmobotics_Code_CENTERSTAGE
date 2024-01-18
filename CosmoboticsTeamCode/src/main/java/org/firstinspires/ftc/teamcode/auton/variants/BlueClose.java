package org.firstinspires.ftc.teamcode.auton.variants;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.Auton;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp="CosmoboticsTeleOp")
public class BlueClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d leftSpikePose = new Pose2d(0, 0, Math.toRadians(0)); //TODO: Create Poses
        Pose2d centerSpikePose = new Pose2d(0, 0, Math.toRadians(0)); //TODO: Create Poses
        Pose2d rightSpikePose = new Pose2d(0, 0, Math.toRadians(0)); //TODO: Create Poses

        Auton BlueClose = new Auton(hardwareMap, leftSpikePose, centerSpikePose, rightSpikePose);

        SampleMecanumDrive drive = BlueClose.drive();

        //Board Auton:
        TrajectorySequence leftBoard = drive.trajectorySequenceBuilder(leftSpikePose)
                //TODO: Write Trajectory
                .build();
        TrajectorySequence centerBoard = drive.trajectorySequenceBuilder(centerSpikePose)
                //TODO: Write Trajectory
                .build();
        TrajectorySequence rightBoard = drive.trajectorySequenceBuilder(rightSpikePose)
                //TODO: Write Trajectory
                .build();

        //To Stack:
        TrajectorySequence leftStack = drive.trajectorySequenceBuilder(leftBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence centerStack = drive.trajectorySequenceBuilder(centerBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence rightStack = drive.trajectorySequenceBuilder(rightBoard.end())
                //TODO: Write Trajectory
                .build();

        //From Stack:
        TrajectorySequence leftReturn = drive.trajectorySequenceBuilder(leftStack.end())
                .setReversed(true)
                //TODO: Write Trajectory
                .build();
        TrajectorySequence centerReturn = drive.trajectorySequenceBuilder(centerStack.end())
                //TODO: Write Trajectory
                .setReversed(true)
                .build();
        TrajectorySequence rightReturn = drive.trajectorySequenceBuilder(rightStack.end())
                .setReversed(true)
                //TODO: Write Trajectory
                .build();

        //Board Park:
        TrajectorySequence leftLeftPark = drive.trajectorySequenceBuilder(leftBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence leftRightPark = drive.trajectorySequenceBuilder(leftBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence centerLeftPark = drive.trajectorySequenceBuilder(centerBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence centerRightPark = drive.trajectorySequenceBuilder(centerBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence rightLeftPark = drive.trajectorySequenceBuilder(rightBoard.end())
                //TODO: Write Trajectory
                .build();
        TrajectorySequence rightRightPark = drive.trajectorySequenceBuilder(rightBoard.end())
                //TODO: Write Trajectory
                .build();

        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            BlueClose.BlueSpikeAuton();
            BlueClose.BoardAuton(leftBoard, centerBoard, rightBoard);
            BlueClose.ToStack(leftStack, centerStack, rightStack);
            BlueClose.FromStack(leftReturn, centerReturn, rightReturn);
            BlueClose.BoardPark(leftLeftPark, centerLeftPark, rightLeftPark);
            BlueClose.BoardPark(leftRightPark, centerRightPark, rightRightPark);
        }
    }
}
