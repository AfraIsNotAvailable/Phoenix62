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
@Autonomous(name="AutonomTaniaESuperFunny(gluma)", group="Pushbot")
public class AutonomSeen extends LinearOpMode implements OpModeAddition{
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
    public RobotEx robot;
   /*
    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    public int caz = -1;
    public boolean started = false;*/

    @Override
    public void runOpMode() {
    /*
        initVuforia();
        initTfod();
        ElapsedTime runtime = new ElapsedTime();

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
*/
        /** Wait for the game to begin */




        waitForStart();
      //  started = true;
     //   telemetry.addData(">gasit final pe gioneala maxima ", caz);
      //  telemetry.update();
        robot = new RobotEx(hardwareMap, this, telemetry);

        robot.driveOnCM(-155,0.7);
        //robot.grab();
        //robot.patinaj(258,-150,30,3);
        //robot.drop();

        int caz = 1;
        switch (caz) {
            case 1: {
                telemetry.addData("caz", 1);
                telemetry.update();

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
//                robot.driveOnFunction(robot.CMtoEuler(-240));
//                robot.curveNoGyro(-90,0.6);
//                // robot.driveOnCM(-45,0.5);
//                //robot.turnNoGyro(90, 0.6);
//                robot.curveNoGyro(-90,-0.6);
//                robot.driveOnCM(-233, 0.8);
//                robot.slideOnCM(-78, 0.6);
//                robot.driveOnCM(12, 0.25);
//                robot.driveOnCM(200,0.8);
//                robot.curveNoGyro(30,0.6);
//                robot.curveNoGyro(30,-0.6);
//                robot.driveOnCM(-10,0.7);

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
//
//                robot.driveOnFn(315);
//                robot.driveOnFn(-312);
//                robot.slideOnCM(-78,0.6);
//                robot.driveOnCM(10,0.25);
//                robot.driveOnCM(265,0.9);
//                robot.curveNoGyro(90, 0.6);
//                robot.driveOnCM(20, 0.7);
//                robot.driveOnCM(-30, 0.7);
//                robot.slideOnCM(110,0.9);
                break;
            }
        }
/*
        if (tfod != null) {
            tfod.shutdown();
        }*/
    }

    /**
     * Initialize the Vuforia localization engine.
     */
//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     *//*
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.maxNumDetections=1;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SECOND_ELEMENT, LABEL_SECOND_ELEMENT);
    }*/

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
