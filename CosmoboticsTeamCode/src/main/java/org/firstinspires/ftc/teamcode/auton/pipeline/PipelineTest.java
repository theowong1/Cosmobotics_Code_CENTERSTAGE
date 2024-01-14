package org.firstinspires.ftc.teamcode.auton.pipeline;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PipelineTest extends LinearOpMode {
    private VisionPortal visionPortal;

    private SpikePosDetector pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new SpikePosDetector();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessors(pipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            SpikePosDetector.SPIKE_POS position = pipeline.getType();
            telemetry.addLine(pipeline.debug());
            telemetry.update();
        }
    }
}

