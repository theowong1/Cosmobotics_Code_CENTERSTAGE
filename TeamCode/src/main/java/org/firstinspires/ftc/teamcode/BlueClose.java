package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class BlueClose extends LinearOpMode {
    Robot robot;
    public void slides(int slidesDistance) {
        robot.slidesTarget = slidesDistance;
        telemetry.addData("SlidesPos: ", slidesMotor.getCurrentPosition());
        telemetry.addData("SlidesTarget: ", robot.slidesTarget);
        telemetry.update();
    }

    public void armMotor(int armDistance) {
        robot.armTarget = armDistance;
        telemetry.addData("ArmMotorPos: ", armMotor.getCurrentPosition());
        telemetry.addData("ArmMotorTarget: ", robot.armTarget);
        telemetry.update();
    }

    DcMotor slidesMotor;
    DcMotor armMotor;
    ServoImplEx leftIntake;
    ServoImplEx rightIntake;
    ServoImplEx rotationalIntake;
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        //Camera:
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("camera", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //Constants:
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(10, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //Purple Trajectories:
        TrajectorySequence leftPurpTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, 38, Math.toRadians(0)))
                .build();
        TrajectorySequence centerPurpTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(0)))
                .build();
        TrajectorySequence rightPurpTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(0, 38, Math.toRadians(0)))
                .build();


        TrajectorySequence myLeftSideTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(50, 23, Math.toRadians(180)))
                .build();
        TrajectorySequence myCenterSideTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(50, 29, Math.toRadians(180)))
                .build();
        TrajectorySequence myRightSideTraj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(50, 35, Math.toRadians(180)))
                .build();

        TrajectorySequence leftToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, 37, Math.toRadians(5)))
                .strafeTo(new Vector2d(40, 24))
                .splineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(180))
                .build();
        TrajectorySequence centerToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, 35, Math.toRadians(5)))
                .strafeTo(new Vector2d(40, 24))
                .splineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(180))
                .build();
        TrajectorySequence rightToCyclePath = drive.trajectorySequenceBuilder(new Pose2d(40, 30, Math.toRadians(0)))
                .strafeTo(new Vector2d(40, 24))
                .splineToSplineHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(180))
                .build();

        TrajectorySequence parkLefttoLeft = drive.trajectorySequenceBuilder(new Pose2d(52, 23, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(48, 47, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(180))
                .build();
        TrajectorySequence parkCentertoLeft = drive.trajectorySequenceBuilder(new Pose2d(52, 29, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(48, 47, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(180))
                .build();
        TrajectorySequence parkRighttoLeft = drive.trajectorySequenceBuilder(new Pose2d(52, 35, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(48, 47, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(70, 52), Math.toRadians(180))
                .build();

        TrajectorySequence parkLefttoRight = drive.trajectorySequenceBuilder(new Pose2d(40, 37, Math.toRadians(5)))
                .lineToSplineHeading(new Pose2d(48, 14, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 11), Math.toRadians(0))
                .build();
        TrajectorySequence parkCentertoRight = drive.trajectorySequenceBuilder(new Pose2d(40, 35, Math.toRadians(5)))
                .lineToSplineHeading(new Pose2d(48, 14, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 11), Math.toRadians(0))
                .build();
        TrajectorySequence parkRighttoRight = drive.trajectorySequenceBuilder(new Pose2d(40, 30, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(48, 14, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 11), Math.toRadians(0))
                .build();

        while (opModeInInit()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
        }

        if (opModeIsActive()) {
            SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition colorPos = pipeline.getAnalysis();
            if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT) {
                drive.followTrajectorySequence(leftPurpTraj);
                //Extend Slides
                slides(2000);
                long start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                //Place Yellow Pixel
                leftIntake.setPosition(1);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 500 && opModeIsActive()) {

                }
                slides(0);
                leftIntake.setPosition(0);
                rotationalIntake.setPosition(.92);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                armMotor(1500);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                slides(600);
                rightIntake.setPosition(1);
                //Place Purple Pixel
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                slides(0);
                armMotor(0);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                rightIntake.setPosition(0);
                rotationalIntake.setPosition(.31);
                //Park
                drive.followTrajectorySequence(parkLefttoLeft);
            } else if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.CENTER) {
                rotationalIntake.setPosition(.92);
                long start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 200 && opModeIsActive()) {

                }
                armMotor(1500);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                slides(3200);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                //Place Purp Pixel
                leftIntake.setPosition(1);
                slides(0);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                leftIntake.setPosition(0);
                armMotor(0);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                rotationalIntake.setPosition(.31);
                drive.followTrajectorySequence(myCenterSideTraj1);
                //Extend Slides
                slides(2000);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                //Place Yellow Pixel
                rightIntake.setPosition(1);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000 && opModeIsActive()) {

                }
                slides(0);
                rightIntake.setPosition(0);
                //Park
                drive.followTrajectorySequence(parkCentertoLeft);
            } else if (colorPos == SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.RIGHT) {
                drive.followTrajectorySequence(myRightSideTraj1);
                //Extend Slides
                slides(2000);
                long start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000) {

                }
                //Place Yellow Pixel
                rightIntake.setPosition(1);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 500) {

                }
                slides(0);
                rightIntake.setPosition(0);
                rotationalIntake.setPosition(.92);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1000) {

                }
                armMotor(1500);
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1500) {

                }
                slides(2700);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1500) {

                }
                //Place Purple Pixel
                leftIntake.setPosition(1);
                slides(0);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 1500) {

                }
                armMotor(0);
                armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                start_t = System.currentTimeMillis();
                while (System.currentTimeMillis()-start_t < 3000) {

                }
                leftIntake.setPosition(0);
                rotationalIntake.setPosition(.31);
                //Park
                drive.followTrajectorySequence(parkRighttoLeft);
            }
        }
    }
}
