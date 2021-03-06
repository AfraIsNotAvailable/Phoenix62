package org.firstinspires.ftc.teamcode;/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name="AutonomCampion", group="Pushbot")
public class AutonomKakegurui extends LinearOpMode implements OpModeAddition{
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public Robot robot;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AbJRubT/////AAABma6KEOwi/UjmpKjNKZN3/NWMSd1P03DjYNZUmfA4zeI/6iDpgj7s7Xvujkc5tKEYP4QwNmtgXUf2kml1Nb0Pozf+iAxWwM+CPmCTFYks5fp0ckAtACUxtCCOluwQCn5NlU8vgBHwBNeVis+2j/26tO8B2Lh1bNz/RZLK9jbIVCQVQRPPAZ2+IpBPQogX3Dc5I4jktld7zcoTEOV1a7y+0sV006TBpV0KnanLQwXyZfDDfjNC1xDsladUdQ35JHU5N2fEwDOnWC7DLAZNU7UgLIPUH1EoHUbbilp4K5HrDqk4SYovwrEeHWccA9tzIE2oT4vsejcEQ99zFVa5+MhQhWKJSBnUTWj696jXeCNwrbm/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    public int caz = -1;
    public boolean started = false;
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        ElapsedTime runtime = new ElapsedTime();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        float r;
        while (caz == -1 || !isOpModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made+++++++++++++++++++.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions and display boundary info.
                    if (updatedRecognitions.size() == 1) {
                        for (Recognition recognition : updatedRecognitions) {
                            r = recognition.getHeight() / recognition.getWidth();
                            telemetry.addData("Ratio: ", r);
                            telemetry.update();
                            if (r > 0.50) {
                                caz = 3;
                                // break;
                            } else {
                                caz = 2;
                                // break;
                            }
                        }
                    }
                    else if( updatedRecognitions.size() == 0) {
                        caz = 1;
                    }
                }
            }

        }

        /** Wait for the game to begin */




        waitForStart();
        started = true;
        telemetry.addData(">gasit final pe gioneala maxima ", caz);
        telemetry.update();
        robot = new Robot(hardwareMap, this, telemetry);

        switch (caz) {
            case 1: {
                telemetry.addData("caz", 1);
                telemetry.update();
//                robot.setSpeed(0.7);
//                sleep(2400);
//                robot.setSpeed(-0.71);
//                sleep(600);
//                robot.mortusMotorus();
//                sleep(100);
//                robot.slide("stanga", 0.7);
//                sleep(1000);
//                robot.setSpeed(0.7);
//                sleep(450);
                robot.driveOnCM(190,0.8);
                robot.driveOnCM(-182,0.8);
                robot.slideOnCM(-78,0.6);
                robot.driveOnCM(10,0.25);
                robot.driveOnCM(130,0.9);
                robot.curveNoGyro(90, 0.5);
                robot.driveOnCM(20, 0.5);
                robot.driveOnCM(-30, 0.5);
                robot.slideOnCM(-15,0.4);
                break;
            }
            case 2:{
                telemetry.addData("caz", 2);
                telemetry.update();

//                robot.setSpeed(0.7);
//                sleep( 3350);
//             //   robot.slide(true, 0.4);
//                sleep(1500);
//                //robot.mortusMotorus();
//                //sleep(100);
//                //robot.setSpeed(0.7);
//                //sleep(200);
//                robot.setSpeed(-0.9);
//                sleep(550);
//                robot.mortusMotorus();
//                //robot.driveOnFunction(robot.tatamiToEuler2(-0.7));
//                //robot.goTo(65,232);
                robot.driveOnFunction(robot.CMtoEuler(-240));
                robot.curveNoGyro(-90,0.6);
                // robot.driveOnCM(-45,0.5);
                //robot.turnNoGyro(90, 0.6);
                robot.curveNoGyro(-90,-0.6);
                robot.driveOnCM(-233, 0.8);
                robot.slideOnCM(-78, 0.6);
                robot.driveOnCM(12, 0.25);
                robot.driveOnCM(200,0.8);
                robot.curveNoGyro(30,0.6);
                robot.curveNoGyro(30,-0.6);
                robot.driveOnCM(-10,0.7);

                break;
            }
            case 3: {
                telemetry.addData("caz", 3);
                telemetry.update();
                // robot.setSpeed(0.7);
        /*robot.motorRF.setPower(-0.5);
        robot.motorRB.setPower(-0.5);
        robot.motorLF.setPower(-0.5);
        robot.motorLB.setPower(-0.5);*/
//                sleep(4200);
//                robot.mortusMotorus();
//                robot.setSpeed(-0.7);
//                sleep(1400);
//                robot.mortusMotorus();

                robot.driveOnFn(315);
                robot.driveOnFn(-312);
                robot.slideOnCM(-78,0.6);
                robot.driveOnCM(10,0.25);
                robot.driveOnCM(265,0.9);
                robot.curveNoGyro(90, 0.6);
                robot.driveOnCM(20, 0.7);
                robot.driveOnCM(-30, 0.7);
                robot.slideOnCM(110,0.9);
                break;
            }

            default:{
                telemetry.addData("caz", 2);
                telemetry.update();

//                robot.setSpeed(0.7);
//                sleep( 3350);
//             //   robot.slide(true, 0.4);
//                sleep(1500);
//                //robot.mortusMotorus();
//                //sleep(100);
//                //robot.setSpeed(0.7);
//                //sleep(200);
//                robot.setSpeed(-0.9);
//                sleep(550);
//                robot.mortusMotorus();
//                //robot.driveOnFunction(robot.tatamiToEuler2(-0.7));
//                //robot.goTo(65,232);
                robot.driveOnFunction(robot.CMtoEuler(-240));
                robot.curveNoGyro(-90,0.6);
                // robot.driveOnCM(-45,0.5);
                //robot.turnNoGyro(90, 0.6);
                robot.curveNoGyro(-90,-0.6);
                robot.driveOnCM(-233, 0.8);
                robot.slideOnCM(-78, 0.6);
                robot.driveOnCM(12, 0.25);
                robot.driveOnCM(200,0.8);
                robot.curveNoGyro(30,0.6);
                robot.curveNoGyro(30,-0.6);
                robot.driveOnCM(-10,0.7);

                break;
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.maxNumDetections=1;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SECOND_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
