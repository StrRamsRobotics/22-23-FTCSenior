package org.firstinspires.ftc.teamcode.system;//package org.firstinspires.ftc.teamcode.system;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera2;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name="hhhhhh")
//public class Vision extends LinearOpMode
//{
//    OpenCvCamera phoneCam;
//    //WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");
//    @Override
//    public void runOpMode()
//    {
//        /*
//         * cameraMonitorViewId is the monitor we want to display to
//         * phoneCam is an openCV camera object
//         */
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
//
//        // no monitor camera view
//        //// Without a live preview
//        //phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
//
//        /*
//         * Image processing pipeline = samplePipeline()
//         */
//        phoneCam.setPipeline(new SamplePipeline());
//
//        /*
//         * Connecting to camera
//         */
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Start streaming images, specify resolution and rotation
//                 */
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//
//        /*
//         * Wait for the user to press start on the Driver Station
//         */
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            /*
//             * Send some stats to the telemetry
//             */
//            telemetry.addData("Frame Count", phoneCam.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
//            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
//            telemetry.update();
//            /*
//             * Hotkey to stop streaming
//             */
//            if(gamepad1.a)
//            {
//                // Stoping camera
//                phoneCam.stopStreaming();
//                // stop pipeline
//                //phoneCam.closeCameraDevice();
//            }
//
//            /*
//             * 10 ticks = 1 hz
//             * stop cpu throttling
//             */
//            sleep(100);
//        }
//    }
//    /*
//     * An example image processing pipeline to be run upon receipt of each frame from the camera.
//     * Note that the processFrame() method is called serially from the frame worker thread -
//     * that is, a new camera frame will not come in while you're still processing a previous one.
//     * In other words, the processFrame() method will never be called multiple times simultaneously.
//     *
//     * However, the rendering of your processed image to the viewport is done in parallel to the
//     * frame worker thread. That is, the amount of time it takes to render the image to the
//     * viewport does NOT impact the amount of frames per second that your pipeline can process.
//     *
//     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
//     * frame worker thread. This should not be a problem in the vast majority of cases. However,
//     * if you're doing something weird where you do need it synchronized with your OpMode thread,
//     * then you will need to account for that accordingly.
//     */
//    public static class SamplePipeline extends OpenCvPipeline
//    {
//        private static final Scalar RED = new Scalar(255, 0, 0);
//        private static final Scalar GREEN = new Scalar(0, 255, 0);
//        private static final Scalar BLUE = new Scalar(0, 0, 255);
//
//        private static final int THRESHOLD = 107;
//
//        Point topLeft = new Point(0, 0);
//        Point bottomRight = new Point(320, 240);
//
//        private volatile int average;
//
//        Mat blurMat = new Mat();
//
//        Mat edges = new Mat();
//        Mat thresh = new Mat();
//        @Override
//        public Mat processFrame(Mat input) {
//            //blur
//            Imgproc.blur(input, blurMat, new Size(320, 240));
//            //blur sub portion
//            blurMat = blurMat.submat(new Rect(topLeft, bottomRight));
//
//            //tune the threshold
//            Imgproc.Canny(thresh, edges, 100, 300);
//
//            //connect contours
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            //morphing morbing
////          Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
////          Imgproc.morphologyEx(blurMat, blurMat, Imgproc.MORPH_CLOSE, kernel);
//
//            //memory cleanup
//            blurMat.release();
//            edges.release();
//            thresh.release();
//
//            return edges;
//        }
//        /*
//         * if you want to save a frame, you need to clone it to another variable
//         */
//    }
//}
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(200, 200, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 130, 255),
            lower_cyan_bounds    = new Scalar(0, 200, 200, 255),
            upper_cyan_bounds    = new Scalar(150, 255, 255, 255),
            lower_magenta_bounds = new Scalar(170, 0, 170, 255),
            upper_magenta_bounds = new Scalar(255, 60, 255, 255);

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Percent and mat definitions
    private double yelPercent, cyaPercent, magPercent;
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), blurredMat = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Gets color specific values
        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        magPercent = Core.countNonZero(magMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == yelPercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == cyaPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (maxPercent == magPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        yelMat.release();
        cyaMat.release();
        magMat.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}