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
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AbJRubT/////AAABma6KEOwi/UjmpKjNKZN3/NWMSd1P03DjYNZUmfA4zeI/6iDpgj7s7Xvujkc5tKEYP4QwNmtgXUf2kml1Nb0Pozf+iAxWwM+CPmCTFYks5fp0ckAtACUxtCCOluwQCn5NlU8vgBHwBNeVis+2j/26tO8B2Lh1bNz/RZLK9jbIVCQVQRPPAZ2+IpBPQogX3Dc5I4jktld7zcoTEOV1a7y+0sV006TBpV0KnanLQwXyZfDDfjNC1xDsladUdQ35JHU5N2fEwDOnWC7DLAZNU7UgLIPUH1EoHUbbilp4K5HrDqk4SYovwrEeHWccA9tzIE2oT4vsejcEQ99zFVa5+MhQhWKJSBnUTWj696jXeCNwrbm/";


    public RobotEx robot;

    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    public int caz = -1;
    public boolean started = false;


    @Override
    public void runOpMode() {

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

        /** Wait for the game to begin */


        robot = new RobotEx(hardwareMap, this, telemetry);

        waitForStart();
        robot.startLauncher(1 );


          started = true;
          telemetry.addData(">gasit final pe gioneala maxima ", caz);
          telemetry.update();
        robot.servoArm.setPosition(0);
        robot.driveOnFn(-130);
        robot.startLauncher(0.61 );
        robot.slideOnCM(-5,0.3);
        robot.shoot(700);
        robot.slideOnCM(-15,0.3);
        robot.startLauncher(0.66 );
        sleep(1500);
        robot.shoot(1000);
        robot.slideOnCM(20,0.3);
        robot.startLauncher(0 );



        double speed = 80;
       // int caz = 3;
        //int caz -1;
        switch (caz) {
            case 1: {

//                telemetry.addData("caz", 1);
//                telemetry.update();
//                robot.driveOnFn(-155);
//                robot.slideOnCM(4,.4);
//                robot.shoot(1000);
//                robot.slideOnCM(4,.4);
//                robot.shoot(1000);
//                robot.stopIntake();
//                robot.slideOnCM(-8,.4);
                //robot.shoot()

//                robot.patinaj(189,125,60,6.5);
//                robot.arm(-700);
//                robot.drop();
//                robot.arm(100);
//                robot.patinaj(85,120,70,7.3);
//
//                robot.driveOnCM(7,0.4);
//                robot.slideOnCM(19,0.4);
//                robot.arm(-700);
//                robot.grab();
//                robot.arm(100);
//                robot.patinaj(265,135,70,8);
//                robot.arm(-700);
//                robot.drop();
                //robot.arm(-100);

                robot.turnNoGyro(180.0,.5);
                robot.slideOnCM(80,0.6);
                robot.dropArm();
                robot.turnNoGyro(180,.5);
                robot.slideOnCM(80,.5);
                robot.grabArm();
                robot.turnNoGyro(180,.5);
                robot.driveOnCM(80,.5);
                robot.dropArm();
                robot.driveOnCM(-10,.5);
                robot.slideOnCM(-60,.5);
                robot.driveOnCM(70,.5);

                break;
            }
            case 2:{
                telemetry.addData("caz", 2);
                telemetry.update();

//                robot.patinaj(270-15 ,100,60,6);
//                robot.arm(-700);
//                robot.drop();
//                robot.arm(100);
//                robot.patinaj(113,207,65,7.0);
//                robot.turnNoGyro(-90,0.7);
//                robot.drop();
//                robot.driveOnCM(45,0.7);
//                robot.grab();
//                robot.driveOnFn(-155);
//                robot.slideOnCM(4,.4);
//                robot.shoot(1000);
//                robot.slideOnCM(4,.4);
//                robot.shoot(1000);
//                robot.stopIntake();
//                robot.slideOnCM(-8,.4);
                //robot.shoot()

//                //robot.turnNoGyro(10,-0.6);
//                robot.move(83.97+180,132,speed);
//                robot.arm(-700);
//                sleep(500);
//                robot.drop();
//                robot.arm(0);
//                //robot.move(360-26.15+180,70,speed/2);
//                robot.slideOnCM(-55,0.7);
//                robot.driveOnFn(217);
//                //robot.driveOnFn(190);

//                robot.arm(-800);
                //   sleep(500);
                // robot.slideOnCM(15,0.5);
//                robot.driveOnCM(5,0.5);
//                robot.grab();
//                robot.arm(100);
//                robot.move(114.22+180,160,speed);
//                robot.startIntake();
//                robot.driveOnFn(110);
//                robot.stopIntake();
//
//                robot.driveOnCM(-65,0.7);
//                robot.shoot(2000);
//                robot.move(80+180,1.4*87,speed);
//                robot.arm(-700);
//                robot.drop();
//                robot.arm(200);
//                robot.move(180,30,speed);
//                robot.driveOnCM(93,0.5);
/////////////////////////////////////////////////////

                robot.turnNoGyro(180,0.5);// te intorci cu fata
                robot.driveOnCM(90,0.5);//mergi in fata
                robot.dropArm();//lasi wobble
                robot.driveOnFn(-223);//back la perete
                robot.slideOnCM(70,0.6);//sldie wooble 2
                robot.grabArm();//ia wobble
                robot.slideOnCM(-60,0.6);//slide back
                robot.driveOnFn(190);//mergi poz 2
                // robot.move(80,205,60);

                robot.dropArm();//lasi jos wooble 2
                robot.driveOnFn(-15);//posibil on fn, back up pt turn
                robot.turnNoGyro(180,0.7); // turn, ne pregatim de lansare

                //throw
//                robot.slideOnCM(-32/2,0.6);  //pozitionam in fata la discuri
//                robot.startLauncher(0 );
//                robot.startIntake();
//                robot.driveOnCM(57,.7);//luam discuri
//                robot.driveOnCM(-50,.7);//mergem unde trb
//                robot.slideOnCM(-10,0.5);
//                robot.shoot(2000);//tragenm
//                robot.startLauncher(0);//oprim launcher
//                robot.turnNoGyro(180,0.5);//pozitionam pt control
                //      robot.driveOnCM(-20,0.6);//parcare

                break;
            }
            case 3: {
                /*telemetry.addData("caz", 3);
                telemetry.update();
                robot.turnNoGyro(190,0.6);
                robot.move(58.87,151.5,speed);
                robot.arm(-700);
                robot.drop();
                robot.arm(100);
                robot.move(360-86.70,278,speed);
                robot.move(180,17,speed);
                //robot.driveOnCM(5,1);
                robot.arm(-700);
                robot.grab();
                robot.arm(100);
                robot.move(82.05,275,speed);
                robot.arm(-700);
                robot.drop();
                robot.arm(100);
                robot.move(180+67.38,169,speed);
                robot.startIntake();
                robot.turnNoGyro(180,0.6);
                robot.move(270,25,speed);*/
//                robot.driveOnFn(-137);
//                robot.slideOnCM(-15,0.4);
//
//                robot.slideOnCM(5,.4);
//                robot.shoot(1000);
//                robot.slideOnCM(5,.4);
//                robot.shoot(1000);
//                robot.stopIntake();
//                robot.slideOnCM(-10,.4);
//                //robot.shoot()
//
//
//                robot.turnNoGyro(180, 0.5);
//                robot.driveOnCM(148, 0.7);
//                robot.slideOnCM(80, 0.5);
//                robot.arm(-700);
//                sleep(500);
//                robot.drop();
//                robot.arm(0);
//                robot.driveOnFn(-283);
//                robot.slideOnCM(-31,0.5);
//                robot.arm(-700);
//                sleep(500);
//                robot.driveOnCM(5,0.5);
//                robot.grab();
//                robot.arm(0);
//                robot.slideOnCM(31,0.5);
//                robot.driveOnFn(275);
//                robot.arm(-700);
//                sleep(500);
//                robot.drop();
//                robot.arm(0);
//                robot.driveOnCM(-100,0.5);


//                robot.stopIntake();
//                robot.move(90,57,speed);
//                robot.shoot(1000);
//                robot.move(90,15,speed);
                //robot.reorient()
                // robot.setSpeed(0.7);
                /*robot.motorRF.setPower(-0.5);
                robot.motorRB.setPower(-0.5);
                robot.motorLF.setPower(-0.5);
                robot.motorLB.setPower(-0.5);*/
//              sleep(4200);
//              robot.mortusMotorus();
//              robot.setSpeed(-0.7);
//              sleep(1400);
//              robot.mortusMotorus();
//
//              robot.driveOnFn(315);
//              robot.driveOnFn(-312);
//              robot.slideOnCM(-78,0.6);
//              robot.driveOnCM(10,0.25);
//              robot.driveOnCM(265,0.9);
//              robot.curveNoGyro(90, 0.6);
//              robot.driveOnCM(20, 0.7);
//              robot.driveOnCM(-30, 0.7);
//              robot.slideOnCM(110,0.9);
                robot.turnNoGyro(180,0.5);
                robot.driveOnFn(125);
                robot.slideOnCM(110,0.6);
                robot.dropArm();
                robot.driveOnCM(-321,0.8);
                robot.slideOnCM(-25,0.5);
                robot.grabArm();
                robot.driveOnCM(325,0.8);
                robot.dropArm();
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

/*


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
        import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

        import java.util.List;


@Autonomous(name="AutonomTaniaESuperFunny(gluma)", group="Pushbot")
public class AutonomSeen extends LinearOpMode implements OpModeAddition{
    //    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
    public RobotEx robot;


    @Override
    public void runOpMode() {




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

                double speed = 60;

                robot.turnNoGyro(10,0.6);
                robot.move(79.97,132,speed);
                //robot.drop();
                robot.move(360-20.15,58,speed);
                robot.move(270,224,speed);
                //robot.grab();
                robot.move(114.22,115,speed);
                robot.startIntake();
                robot.move(270,70,speed);
                robot.stopIntake();
                robot.move(90,95,speed);
                //robot.shoot();
                robot.move(70.63,87,speed);
                //robot.drop();
                robot.move(360,93,speed);
         robot.driveOnCM(-10,0.7);

                break;
            }
            case 3: {
                telemetry.addData("caz", 3);
                telemetry.update();

                double speed = 60;

                robot.turnNoGyro(190,0.6);
                robot.move(58.87,151.5,speed);
                //robot.drop();
                robot.move(360-86.70,278,speed);
                robot.move(180,17,speed);
                //robot.driveOnCM(5,1);
                //robot.grab();
                robot.move(82.05,275,speed);
                //robot.drop();
                robot.move(180+67.38,169,speed);
                robot.startIntake();
                robot.turnNoGyro(180,0.6);
                robot.move(270,25,speed);
                robot.stopIntake();
                robot.move(90,57,speed);
                //robot.shoot()
                robot.move(90,15,speed);
                //robot.reorient()


                break;
            }
        }


    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
*///ch