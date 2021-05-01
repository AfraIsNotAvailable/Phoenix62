

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="AutonomTurnTest", group="Pushbot")
//@Disabled
public class AutonomTurnTest extends LinearOpMode implements OpModeAddition {

    RobotEx robot = null;

    @Override
    public boolean isOpModeIsActive(){
        return opModeIsActive();
    }

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)

        robot =  new RobotEx(hardwareMap,this,telemetry);
        waitForStart();
        for(int i = 0; i <= 360/15; ++i)
        {
            robot.move(90 + 15 * i,100,40);
            sleep(500);
            robot.move(90 + 15 * i,-100,40);
        }



        sleep(1000);
  //      robot.setVelocity(0,-30*10/34,10000/3);
//        robot.setVelocity( 0,20,5000);

      //  while (opModeIsActive()) {
//            for(int i = 1; i <= 10; ++i)
//            {
//                robot.setVelocity( 0,i*10,1000);
//                sleep(2500);
//                robot.setVelocity( 0,-i*10,1000);
//                sleep(5000);
//            }
//            telemetry.setMsTransmissionInterval(10);
//
//            double[] r = robot.getForce(
//                    robot.motorLF.getVelocity(AngleUnit.RADIANS),
//                    robot.motorLF.getVelocity(AngleUnit.RADIANS),
//                    robot.motorLF.getVelocity(AngleUnit.RADIANS),
//                    robot.motorLF.getVelocity(AngleUnit.RADIANS));
//            telemetry.addData("internal: ", Math.sqrt(r[0]*r[0] + r[1]*r[1]));


            
            telemetry.update();
       // }
        // run until the end of the match
        // (driver presses STOP)

        //for (int i = 0; i <= 12; i++) {
        //    robot.setAng(30*i,0.6);
        //    sleep(1000);
        //}

//        robot.setAng2(0,0.6);
//        sleep(500);
//        robot.setAng2(22.5,0.6);
//        sleep(500);
//        robot.setAng2(45,0.6);
//        sleep(500);
//        robot.setAng2(67.5,0.6);
//        sleep(500);
//        robot.setAng2(90,0.6);
//        sleep(500);
//        robot.setAng2(180,0.6);
//        sleep(500);
//        robot.setAng2(270,0.6);
//        sleep(500);


  //      robot.turnNoGyro(90,0.6);
      /*  telemetry.addData("FNDANG",robot.angleBetween(0,100));
        robot.setAngle(robot.angleBetween(0,100));
        this.robot.posX = 0;
        this.robot.posY = 100;
        sleep(2000);
        telemetry.addData("FNDANG4",robot.angleBetween(-100,100));
        robot.setAngle(robot.angleBetween(-100,100));
        this.robot.posX = -100;
        this.robot.posY = 100;

        sleep(2000);
        telemetry.addData("FNDANG3",robot.angleBetween(-100,200));
        robot.setAngle(robot.angleBetween(-100,200));
        this.robot.posX = -100;
        this.robot.posY = 200;

        sleep(2000);
        telemetry.addData("FNDANG2",robot.angleBetween(-100,0));
        robot.setAngle(robot.angleBetween(-100,0));
        this.robot.posX = -100;
        this.robot.posY = 0;

        sleep(2000);
        telemetry.update();
        sleep(20000);*/

        /*
        robot.turn(90,0.6);
        sleep(500);
        robot.turn(180,0.6);
        sleep(500);
        robot.turn(15,0.6);
        sleep(500);
        robot.turn(0,0.6);
        sleep(500);

        robot.setAng(0,0.6);
        sleep(500);
        robot.setAng(90,0.6);
        sleep(500);
        robot.setAng(180,0.6);
        sleep(500);
        robot.setAng(270,0.6);
        sleep(500);*/

//        robot.motorArm.setPower(0.4);
//        sleep(1000);

//        robot.motorRB.setPower(1);
//        sleep(2000);
//        robot.motorRB.setPower(0);
//
//        robot.motorRF.setPower(1);
//        sleep(2000);
//        robot.motorRF.setPower(0);
//
//        robot.motorLB.setPower(1);
//        sleep(2000);
//        robot.motorLB.setPower(0);
//
//        robot.motorLF.setPower(1);
//        sleep(2000);
//        robot.motorLF.setPower(0);
    }
}
