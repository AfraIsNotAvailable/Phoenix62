package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.exp;


@Autonomous(name="AutonomDriveOnFunction", group="Pushbot")
//@Disabled

public class AutonomDriveOnFunction extends LinearOpMode implements OpModeAddition{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public boolean isOpModeIsActive(){
        return opModeIsActive();
    }
    /**
     * Returns constant K with respect to the ratio of the motor
     *
     * @param  iRatio the ratio of the motor, such as the 40 from 40:1
     */
    public double k(int iRatio) {
        double reductie = iRatio*1.5;
        double coutPerRev = 28;
        double wheelDiam = 4.0 * 2.54;
        return (reductie * coutPerRev) / (wheelDiam * 3.14);
    }

    /**
     * Returns constant K with respect to the ratio of the motor.
     * Returns an integer instead of a double, becasue I hate casts.
     *
     * @param  iRatio the ratio of the motor, such as the 40 from 40:1
     */
    public int ki(int iRatio) {
        double reductie = iRatio*1.5;
        double coutPerRev = 28;
        double wheelDiam = 4.0 * 2.54;
        return (int)((reductie * coutPerRev) / (wheelDiam * 3.14));
    }

    public double medianDelta = 0;
    public double medianPositionDelta = 0;

    public int deltasCounted = 0;
    public int positionalDeltasCounted = 0;

    public RobotEx robot;

    public void testFunction(int target){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double delta = 0;
        double positionDelta = 0;
        double speed = 0.368; //drq stie dc, nu ne atingem de ancient magic
        target *= ki(40);

        motor.setTargetPosition(motor.getCurrentPosition() + target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Chestie","Chestie"+String.valueOf(motor.getCurrentPosition() + target));
        telemetry.update();
        double Target = (double)target;

        // e^-x^4 is more square, thus we get more max speed time
        speed = exp((-1)* Math.pow(((2/Target)*motor.getCurrentPosition() -  1),4));
        motor.setPower(speed);
        double lastPos = motor.getCurrentPosition();
        while(motor.isBusy() && isOpModeIsActive()){
            speed = exp((-1)* Math.pow(((2/Target)*motor.getCurrentPosition() -  1),4));
            motor.setPower(speed);
            dDistance+=speed; // homemade integrala(GONE WRONG)

            delta = speed - exp((-1)* Math.pow(((2/Target)*motor.getCurrentPosition() -  2),4));

            positionDelta = motor.getCurrentPosition() - lastPos;
            lastPos = motor.getCurrentPosition();

            medianDelta += delta;
            medianPositionDelta += positionDelta;

            ++deltasCounted;
            ++positionalDeltasCounted;

            //sper ca asa se face telemetry ca altfel ma sincuid
            telemetry.addData("Distanta", "Distanta: " + String.valueOf(dDistance));
            telemetry.addData("Delta", "Delta: " + String.valueOf(delta));
            telemetry.addData("PDelta", "Positional Delta: " + String.valueOf(positionDelta));
            telemetry.addData("poz", "Pozitia: " + motor.getCurrentPosition());
            telemetry.addData("Chestie","Chestie: "+target);

            telemetry.update();
        }
        telemetry.addData("HERE","HERE: ");

        telemetry.update();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPower(0);
    }

    private DcMotor motor = null;
    public double dDistance = 0;

    public double eulerTimeMedian = 0;
    public double ETC = 0;

    public void eulerTest(int t){
        testFunction(t);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Distanta", "Distanta: " + String.valueOf(dDistance));
        telemetry.addData("Median Delta", "MD: " + String.valueOf(medianDelta/deltasCounted));
        telemetry.addData("Median PDelta", "MPD: " + String.valueOf(medianPositionDelta/positionalDeltasCounted));

        telemetry.update();
        sleep(5000);
        runtime.reset();
    }

    public void eulerTestMedie(int t){
        testFunction(t);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Distanta", "Distanta: " + String.valueOf(dDistance));
        telemetry.addData("Median Delta", "MD: " + String.valueOf(medianDelta/deltasCounted));
        telemetry.addData("Median PDelta", "MPD: " + String.valueOf(medianPositionDelta/positionalDeltasCounted));

        telemetry.update();
        sleep(10);
        ETC++;
        eulerTimeMedian+=runtime.seconds();
        runtime.reset();
    }

    public void testTimpMediuEuler(int r, int t){
        for(int i = 0; i < r; ++i){
            eulerTestMedie(t);
        }

        telemetry.addData("Median runtime","Median Runtime: " + String.valueOf(eulerTimeMedian/ETC));
        telemetry.update();
        sleep(10000);
    }

    @Override
    public void runOpMode() {
        // motor = hardwareMap.get(DcMotor.class, "motor");
        //  motor.setDirection(DcMotor.Direction.FORWARD);
        robot = new RobotEx(hardwareMap,this,telemetry);
        waitForStart();
        runtime.reset();

        robot.servoArm.setDirection(Servo.Direction.REVERSE);
        robot.servoArm.setPosition(0);
        robot.servoArm.setPosition(100);
//        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //while(opModeIsActive()){
//        robot.motorRB.setVelocity(1, AngleUnit.RADIANS);
//        //sleep(1000);
//        robot.motorLB.setVelocity(1, AngleUnit.RADIANS);
//        //sleep(1000);
//        robot.motorLF.setVelocity(1, AngleUnit.RADIANS);
//        //sleep(1000);
//        robot.motorRF.setVelocity(1, AngleUnit.RADIANS);
        //
        // sleep(1000);


//        robot.motorRB.setPower(1);
//        sleep(1000);
//        robot.motorLB.setPower(1);
//        sleep(1000);
//        robot.motorLF.setPower(1);
//        sleep(1000);
//        robot.motorRF.setPower(1);
//        sleep(1000);

        //sleep(10000);
        //}

//        robot.goToF2(120,170);



//
//        robot.setVelocityW(45,45,0);
//        sleep(1000);
//        robot.setVelocityW(-45,-45,0);
//        sleep(1000);//        double fx = 0, fy = 45;
//        double ang = robot.getAng3();
//        while(opModeIsActive()){
//            double[] rot = robot.vecRotate(fx,fy,-Math.abs(robot.getAng3() - ang));
//            fx = rot[0];
//            fy = rot[1];
//            robot.setVelocityW(fx,fy,Math.PI/5);
//        }

        //robot.turnNoGyro(90,0.5);
        //robot.turnNoGyro(90,0.5);
        //robot.turnNoGyro(360,0.5);

        //robot.turnNoGyro(180,0.5);

//        robot.goTo(0,100);
//        telemetry.addData("X: ",robot.posX);
//        telemetry.addData("Y: ",robot.posY);
//        telemetry.addData("ANG: ",robot.ang);
//        telemetry.update();
//        sleep(1000);
//        //robot.
//        robot.goTo(-100,100);
//        telemetry.addData("X: ",robot.posX);
//        telemetry.addData("Y: ",robot.posY);
//        telemetry.addData("ANG: ",robot.ang);
//        telemetry.update();
//        sleep(1000);
//
//        robot.goTo(-100,200);
//        telemetry.addData("X: ",robot.posX);
//        telemetry.addData("Y: ",robot.posY);
//        telemetry.addData("ANG: ",robot.ang);
//        telemetry.update();
//        sleep(1000);

        // sleep(500);
        //robot.turnNoGyro(180,0.5);
        // sleep(500);

        //robot.turn2(180,0.5);
        //robot.turn2(-200,0.5);
        //robot.drive(100,0.5);
        //sleep(5000);
        // robot.drive(100,1);
//        robot.curve(90,0.5);
        //robot.slideOnCM(20,-0.5);
        //robot.slideOnCM(-20,0.5);
        //robot.slideOnCM(-20,-0.5);
        //robot.slideOnCM(20,0.5);
        //robot.slide(-1,0.5);
        ////sleep(2000);
        // robot.slide(1,0.5);
        //sleep(2000);

        //untilTime();

        //testTimpMediuEuler(1,eulerTime(5));
        /*
        for(int i = 0 ; i < 20; ++i){
            eulerTestDistanta(100);
        }
        telemetry.addData("Median Distance:", medianDistance/distTestCount);
        telemetry.update();
        sleep(10000);*/
        //testTimpMediuEuler(20,100);
        // 50 ~ 3 tatamiuri
        /*runtime.reset();
        robot.driveOnFunction(robot.eulerTime(2));
        telemetry.addData("T1 ",runtime.seconds());
        telemetry.update();
        sleep(2000);
        robot.driveOnFunction(robot.eulerTime(-2));
        telemetry.addData("T2 ",runtime.seconds());
        telemetry.update();
        sleep(2000);
        robot.driveOnFunction(robot.tatamiToEuler(6));
        sleep(2000);*/
/*
        //testTimpMediuEulerFull(20,100);
        //robot.driveOnFunction(robot.tatamiToEuler2(2));


       // robot.setAng(-robot.angleBetween(16,16),0.5);
        /*
        double ux = robot.CMtoUnits(0);
        double uy =  robot.CMtoUnits(100);
        telemetry.addData("HELP: ",         robot.angleBetween(ux,uy));
        telemetry.update();
        sleep(1000);
        robot.goTo(0,100);
        ux = robot.CMtoUnits(100);
        uy =  robot.CMtoUnits(100);
        telemetry.addData("HELP: ",         robot.angleBetween(ux,uy));
        telemetry.update();
        sleep(1000);
        robot.goTo(100,100);
        ux = robot.CMtoUnits(100);
        uy =  robot.CMtoUnits(0);
        telemetry.addData("HELP: ",         robot.angleBetween(ux,uy));
        sleep(1000);
        robot.goTo(100,0);
        ux = robot.CMtoUnits(0);
        uy =  robot.CMtoUnits(0);
        telemetry.addData("HELP: ",         robot.angleBetween(ux,uy));
        sleep(1000);
        robot.goTo(0,0);
        */

        //for (int i = 0; i < 100; i++) {
        //   robot.goTo(random()%160-80,random()%160-80);
        // }
        // robot.driveOnFunction(robot.tatamiToEuler(1));
        //obot.driveOnFunction(robot.tatamiToEuler2(1));


    }
}