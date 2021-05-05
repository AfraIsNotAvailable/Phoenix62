
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

        boolean in = false;
        boolean out = false;

        double left_nigger = gamepad2.left_trigger;
        double right_nigger = gamepad2.right_trigger;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

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




            if (gamepad2.dpad_up)
            {
                robot.arm(-300);
            }
            else if (gamepad2.dpad_down)
            {
                robot.arm(600);
            }

            if(gamepad2.dpad_right) {
                robot.servoArm.setPosition(0.6);
            }else if(gamepad2.dpad_left){
                robot.servoArm.setPosition(0.3);
            }

            robot.motorLauncher.setVelocity(btoi(gamepad2.right_bumper)*100,AngleUnit.RADIANS);
            robot.motorIntake.setVelocity(btoi(gamepad2.left_bumper)*100,AngleUnit.RADIANS);
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

            robot.setVelocityW(-actv[0], -actv[1],-Math.PI/2.5*gamepad1.right_stick_x);
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
