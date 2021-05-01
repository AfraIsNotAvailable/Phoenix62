

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@Autonomous(name="AutonomDioptriMari", group="Pushbot")
public class AutonomCorigentaLaChimie extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(new SamplePipeline());

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a)
            {

                phoneCam.stopStreaming();

            }

            sleep(100);
        }
    }


    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;
        private double angle(Point pt1, Point pt2, Point pt0) {
            double dx1 = pt1.x - pt0.x;
            double dy1 = pt1.y - pt0.y;
            double dx2 = pt2.x - pt0.x;
            double dy2 = pt2.y - pt0.y;
            return (dx1 * dx2 + dy1 * dy2) / Math.sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
        }

//        private void setLabel(Mat im, String label, MatOfPoint contour) {
//            int fontface = Core.;
//            double scale = 3;//0.4;
//            int thickness = 3;//1;
//            int[] baseline = new int[1];
//            Size text = Imgproc.getTextSize(label, fontface, scale, thickness, baseline);
//            Rect r = Imgproc.boundingRect(contour);
//            Point pt = new Point(r.x + ((r.width - text.width) / 2),r.y + ((r.height + text.height) / 2));
//            Imgproc.putText(im, label, pt, fontface, scale, new Scalar(255, 0, 0), thickness);
//        }
        Mat redMask = new Mat(320,240, CvType.CV_8U,new Scalar(255,0,0));
    //    Mat redMask = new Mat();
        Scalar lower0= new Scalar(0,100,100);
        Scalar upper0= new Scalar(10,255,255);
        Scalar lower1= new Scalar(180-10,100,100);
        Scalar upper1= new Scalar(180,255,255);
        Mat hsv = new Mat();
        Mat yCbCrChan2Mat = new Mat();
        Mat mask = new Mat();
        Mat mask_0 = new Mat();
        Mat mask_1 = new Mat();
        Mat higherarchy = new Mat();
        MatOfPoint2f approxCurve = new MatOfPoint2f();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        int num;
        @Override
        public Mat processFrame(Mat input)
        {

//            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
//            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 90, 98, Imgproc.THRESH_BINARY_INV);
//            Imgproc.findContours(thresholdMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//            num = contours.size();
//            yCbCrChan2Mat.copyTo(contoursOnFrameMat);
//            Imgproc.drawContours(contoursOnFrameMat, contours, -1, new Scalar(0, 0, 255), 3, 8);

              hsv = input.clone();
              Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

              Imgproc.findContours(hsv,contours,new Mat(),Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);

              for(MatOfPoint cnt : contours){
                  Rect rect = Imgproc.boundingRect(cnt);
                  Imgproc.rectangle(hsv,rect,new Scalar(255,0,0),1,1);
              }
              //
//            Core.inRange(hsv, lower0 , upper0,mask_0);
//            Core.inRange(hsv, lower1 , upper1,mask_1 );
//
//            Core.bitwise_or(mask_0, mask_1,mask);
//
//            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//            Imgproc.findContours(mask, contours, higherarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
//            int num = contours.size();
//            Imgproc.drawContours(mask,contours,-1,new Scalar(255,0,0),3,8);
//
//            for (MatOfPoint cnt : contours) {
//
//                MatOfPoint2f curve = new MatOfPoint2f(cnt.toArray());
//
//                Imgproc.approxPolyDP(curve, approxCurve, 0.02 * Imgproc.arcLength(curve, true), true);
//
//                int numberVertices = (int) approxCurve.total();
//
//                double contourArea = Imgproc.contourArea(cnt);
//
//                if (Math.abs(contourArea) < 100) {
//                    continue;
//                }
//
//                //Rectangle detected
//                if (numberVertices >= 4 && numberVertices <= 6) {
//
//                    List<Double> cos = new ArrayList<>();
//
//                    for (int j = 2; j < numberVertices + 1; j++) {
//                        cos.add(angle(approxCurve.toArray()[j % numberVertices], approxCurve.toArray()[j - 2], approxCurve.toArray()[j - 1]));
//                    }
//
//                    Collections.sort(cos);
//
//                    double mincos = cos.get(0);
//                    double maxcos = cos.get(cos.size() - 1);
//
//                    if (numberVertices == 4 && mincos >= -0.1 && maxcos <= 0.3) {
//                        Imgproc.;
//                    }
//
//                }
//            }
            return yCbCrChan2Mat;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                phoneCam.pauseViewport();
            }
            else
            {
                phoneCam.resumeViewport();
            }
        }
    }
}
