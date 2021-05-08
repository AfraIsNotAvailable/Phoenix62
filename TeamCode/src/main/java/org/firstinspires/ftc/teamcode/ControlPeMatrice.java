
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LoggerData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpModeAddition;
import org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.sqrt;
import static java.lang.StrictMath.abs;

@TeleOp(name="UltraInstinct", group="Control")
//@Disabled
public class ControlPeMatrice extends LinearOpMode implements OpModeAddition {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public RobotEx robot = null;

  //  public LoggerData wr = new LoggerData(robot);

    private int btoi(boolean b){
        if(b) return  1;
        else return 0;
    }
    public boolean flag = false, mode = false;
    public boolean flagServo = false, modeServo = false;
    private float speed;
    private float direction;
    private float right;
    private float left;
    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        robot = new RobotEx(hardwareMap,this, telemetry);

        //wr.generateLogFile("patinaj_controlat");

        waitForStart();

        runtime.reset();

        robot.motorArm.setTargetPosition(200);

        double rangle = robot.getAng3();
        double angoff = 0;

        double armSpeed = gamepad2.right_trigger - gamepad2.left_trigger;

        boolean in = false;
        boolean out = false;

        /*double left_nigger = gamepad2.left_trigger;
        double right_nigger = gamepad2.right_trigger;*/
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            speed = gamepad1.right_trigger - gamepad1.left_trigger;
            direction = -  gamepad1.left_stick_x;
            right = speed - direction;
            left = speed + direction;

            if (right > 1)
                right = 1;
            if (right < -1)
                right = -1;
            if (left > 1)
                left = 1;
            if (left < -1)
                left = -1;
            // Left-Bumper Activity - Use of the motorIntake (the Intake Motor)
            /*if (gamepad2.left_bumper) {
                in = !in;
                while (gamepad2.left_bumper) {
                }
            }*/
//{            } else if (gamepad2.left_bumper && robot.motorIntake.getPower() == 1) {
//                robot.motorIntake.setPower(0);
//            }
            /*if (gamepad2.right_bumper) {
                out = !out;
                while (gamepad2.right_bumper) {
                }
            }*/
//            robot.setSpeed(speed);
            robot.motorArm.setPower(armSpeed);

            if (gamepad2.dpad_down)
            {
                robot.arm(-700);
            }
            else if (gamepad2.dpad_up)
            {
                robot.arm(0);
            }
            if (gamepad2.dpad_left && !flagServo) {
                flagServo = true;
                modeServo = !modeServo;
                if(modeServo){
                    robot.servoArm.setPosition(1);
                }else{
                    robot.servoArm.setPosition(0);
                }
                telemetry.addData("sflag: ", flagServo);
            } else if (!gamepad2.dpad_left && flagServo) {
//                robot.setSpeed(speed);
                flagServo = false;
                telemetry.addData("sflag: ", flagServo);
            }
            if (gamepad2.left_bumper) {
                robot.motorIntake.setPower(1);
            } else if (gamepad2.right_bumper) {
                robot.motorIntake.setPower(-1);
            } else {
                robot.motorIntake.setPower(0);
            }
            robot.motorLauncher.setPower(gamepad2.right_trigger > 0.55 ? 0.55 : gamepad2.right_trigger);
//            if(gamepad2.right_bumper){
//                robot.motorLauncher.setPower(10);
//            }
//            else
//            {
//                robot.motorLauncher.setPower(0);
//            }
            /*robot.motorLauncher.setPower(gamepad2.right_trigger > 0.6 ? 0.6 : gamepad2.right_trigger);
            robot.motorIntake.setVelocity(btoi(gamepad2.left_bumper)*100,AngleUnit.RADIANS);*/
            // Right-Bumper Activity - Use of the motorLauncher (the Launch Motor)
//            if (gamepad2.right_bumper && robot.motorLauncher.getPower() == 1) {
//                robot.motorLauncher.setPower(1);
//            } else if (gamepad2.right_bumper && robot.motorLauncher.getPower() == 0) {
//                robot.motorLauncher.setPower(0);
//            }

            rangle = robot.getAng3();
            // telemetry.addData("time: ",runtime.toString());
            // telemetry.update();

            // telemetry.addData("px: ",-45* gamepad1.left_stick_x);
            // telemetry.addData("py: ",-45* gamepad1.left_stick_y);

            double[] actv = robot.vecRotate(-45 * gamepad1.left_stick_x,-45 * gamepad1.left_stick_y, (rangle-angoff));
            telemetry.addData("mag ",sqrt((-45 * gamepad1.left_stick_x)*(-45 * gamepad1.left_stick_x) + (-45 * gamepad1.left_stick_y)*(-45 * gamepad1.left_stick_y)));
            telemetry.addData("ww",-Math.PI/2.5*gamepad1.right_stick_x);
            telemetry.addData("w/m",-Math.PI/2.5*gamepad1.right_stick_x/sqrt((-45 * gamepad1.left_stick_x)*(-45 * gamepad1.left_stick_x) + (-45 * gamepad1.left_stick_y)*(-45 * gamepad1.left_stick_y)));
            telemetry.addData("position: ", robot.motorArm.getCurrentPosition());
            telemetry.addData("velocity: ", robot.motorArm.getVelocity());
            telemetry.addData("is at target: ", !robot.motorArm.isBusy());
            telemetry.addData("current",robot.motorLauncher.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("rot",robot.motorLauncher.getVelocity());
          //  telemetry.addData("powah",robot.motorLauncher.getVelocity());

            /*robot.setVelocityW(-actv[0], -actv[1],-Math.PI/2.5*gamepad1.right_stick_x);*/

            if (gamepad1.a && !flag) {
                flag = true;
                mode = !mode;
                telemetry.addData("flag: ", flag);
            } else if (!gamepad1.a && flag) {
//                robot.setSpeed(speed);
                flag = false;
                telemetry.addData("flag: ", flag);
            }
            telemetry.addData("mode: ", mode);
            if (mode) {
                robot.setVelocityW(-actv[0], -actv[1],-Math.PI/2.5*gamepad1.right_stick_x);
            } else {
                if (gamepad1.right_stick_x != 0) {
                    robot.motorLB.setPower(gamepad1.right_stick_x);
                    robot.motorLF.setPower(-gamepad1.right_stick_x);
                    robot.motorRB.setPower(-gamepad1.right_stick_x);
                    robot.motorRF.setPower(gamepad1.right_stick_x);
                }
                else {
                    robot.motorLB.setPower(-left);
                    robot.motorLF.setPower(-left);
                    robot.motorRB.setPower(-right);
                    robot.motorRF.setPower(-right);
                }
                telemetry.addData("gamepad1.rs_x: ", gamepad1.right_stick_x);
            }
            // robot.setVelocity(-22.5* gamepad1.right_stick_x,-22.5*gamepad1.right_stick_y);
            //   robot.motorWobble.setPower(gamepad2.right_trigger);
            //  robot.motorWobble.setPower(-gamepad2.left_trigger);

            //  if(gamepad2.dpad_up)
            //  {
            //  robot.servoWobble.setPosition(0.95);
            //}else if(gamepad2.dpad_down){
            //  robot.servoWobble.setPosition(0.5);
            //}
            /*if (gamepad2.x){

                robot.grab();
            } else if (gamepad2.y) {
                robot.release();
            }
            if (gamepad2.dpad_up) {
                robot.motorBrat.setPower(0.3);
            } else if (gamepad2.dpad_down) {
                robot.motorBrat.setPower(-0.3);
            } else if (gamepad2.dpad_right) {
                robot.motorBrat.setPower(0);
            }*/

            telemetry.addData("a1: ", rangle);
            telemetry.addData("a2: ",-1*(rangle - angoff));
            telemetry.addData("lx: ",gamepad1.left_stick_x);
            telemetry.addData("ly: ",gamepad1.left_stick_y);

            telemetry.update();

         //+   wr.writeLogLine();
        }
    }
    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
