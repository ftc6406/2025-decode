package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

@TeleOp(name = "ObjectDetection", group = "Test")
public class ObjectDetectionOpMode extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
        );

        // Set the custom pipeline
        webcam.setPipeline(new SamplePipeline());

        // Open the camera device and start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error:", errorCode);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
            telemetry.update();
        }
    }

    static class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV color space for easier color detection
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define range of the color you want to detect
            // Scalar lowerBound = new Scalar(50, 100, 100); // Example for green
            // Scalar upperBound = new Scalar(70, 255, 255);

            // Create mask to filter out the desired color
            Mat mask = new Mat();
            Mat tempMask = new Mat();

            HashSet<Webcam.Color> enumSet = new HashSet<>();
            enumSet.add(Webcam.Color.GREEN);

            Core.inRange(hsv, Webcam.Color.RED.getLowerBound(), Webcam.Color.RED.getUpperBound(), mask);

            // Add each desired color
            for (Webcam.Color color : enumSet) {
                tempMask = new Mat();
                Core.inRange(hsv, color.getLowerBound(), color.getUpperBound(), tempMask);
                Core.bitwise_or(tempMask, mask, mask);
                tempMask.release();
            }

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw contours on the original image
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2); // Green rectangles around objects
            }

            hsv.release();
            mask.release();
            tempMask.release();
            hierarchy.release();
            return input;
        }
    }
}
