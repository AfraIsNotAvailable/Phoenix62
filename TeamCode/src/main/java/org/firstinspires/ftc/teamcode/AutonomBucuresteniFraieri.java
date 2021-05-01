package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;

import static java.lang.Math.sqrt;

@Autonomous(name="AutonomBucuresteniFraieri", group="Pushbot")

public class AutonomBucuresteniFraieri  extends LinearOpMode implements OpModeAddition {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotEx robot;

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }

    @Override
    public void runOpMode() {
        robot = new RobotEx(hardwareMap, this, telemetry);
        waitForStart();

        robot.patinaj(135,100,30, 3);
        //double[] v = robot.vecRotate(100,100,45);
        //robot.goToF2(v[0],v[1]);
        //robot.turnNoGyro(45,0.5);
        //double[] v = robot.vecRotate(0,100,45);
        //50 = 180 de grade wtf
        /*robot.setVelocityFieldCentric(0,100, 50,50);
        sleep(5000);
        try {
            robot.goToF2(100,100,180);
        } catch (IOException e) {
            e.printStackTrace();
        }*/
        // v = robot.normalize(v,robot.maxRads(robot.maxRPM));
        //telemetry.addData("vx ",v[0]);
        //telemetry.addData("vy ",v[1]);
        //telemetry.update();
        //sleep(2000);
        //robot.setVelocity(v[0],v[1],1000);
        //robot.goToF(0,100,0.3);
        //sleep(3000);
        //robot.goToF(0,100);
        // robot.setVelocity(23, 30,1000);
        //  robot.setVelocity(20, 20,1000);
        //  robot.setVelocity(15, 20,1000);
        //  robot.goToF(0,10);
        // robot.goToF(10,0);
        //robot.goToF(10,10);
//        while (isOpModeIsActive())
//        {
//            double actv[] = robot.vecRotate(-45 * 0.3 / 2 ,-45 * 0, robot.getAng3());
//            actv = robot.vecRotate(actv[0] ,actv[1] , -2);
//            telemetry.addData("mag ",sqrt((-45 * 0)*(-45 * 0) + (-45 * 1)*(-45 * 1)));
//            telemetry.addData("ww",-Math.PI/2.5*1);
//            telemetry.addData("w/m",-Math.PI/2.5*sqrt((-45 * 0)*(-45 * 0) + (-45 * 1)*(-45 * 1)));
//            telemetry.addData("x",actv[0]);
//            telemetry.addData("y",actv[1]);
//            robot.setVelocityW(actv[0],actv[1],-Math.PI/10.5/2);
//            telemetry.update();
//        }




        //robot.setVelocityW(-actv[0], -actv[1],-Math.PI/2.5*1);

        //sleep(1000);


    }
}
