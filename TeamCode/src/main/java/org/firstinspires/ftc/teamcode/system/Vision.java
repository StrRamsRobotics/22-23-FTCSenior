package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="hhhhhh")
public class Vision extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {
        /*
         * cameraMonitorViewId is the monitor we want to display to
         * phoneCam is an openCV camera object
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // no monitor camera view
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Image processing pipeline = samplePipeline()
         */
        phoneCam.setPipeline(new SamplePipeline());

        /*
         * Connecting to camera
         */
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Start streaming images, specify resolution and rotation
                 */
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
            /*
             * Hotkey to stop streaming
             */
            if(gamepad1.a)
            {
                // Stoping camera
                phoneCam.stopStreaming();
                // stop pipeline
                //phoneCam.closeCameraDevice();
            }

            /*
             * 10 ticks = 1 hz
             * stop cpu throttling
             */
            sleep(100);
        }
    }
    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    public static class SamplePipeline extends OpenCvPipeline
    {
        private static final Scalar RED = new Scalar(255, 0, 0);
        private static final Scalar GREEN = new Scalar(0, 255, 0);

        private static final int THRESHOLD = 107;

        Point topLeft = new Point(50, 50);
        Point bottomRight = new Point(100, 100);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private volatile int average;
        private volatile TYPE type = TYPE.BALL;

        private void inputToCb(Mat input) {
            //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Core.extractChannel(YCrCb, Cb, 2);

            //testing image thresholding
            greyscale = Imageproc.cvtColor(img, YCrCb, cv2.COLOR_BGR2GRAY);
            Imgproc.adaptiveThresholding(greyscale, 125,
                    Imgproc.ADAPTIVE_THRESH_MEAN_C,
                    Imgproc.THRESH_BINARY, 11, 12);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            region1_Cb = Cb.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            average = (int) Core.mean(region1_Cb).val[0];

            if (average > THRESHOLD) {
                type = TYPE.BALL;
                Imgproc.circle(input, topLeft, 55, RED, 2);
            } else {
                type = TYPE.CUBE;
                Imgproc.rectangle(input, topLeft, bottomRight, GREEN, 2);
            }

            return input;
        }

        public TYPE getType() {
            return type;
        }

        public int getAverage() {
            return average;
        }

        public enum TYPE {
            BALL, CUBE
        }
            /*
             * if you want to save a frame, you need to clone it to another variable
             */
    }
}
