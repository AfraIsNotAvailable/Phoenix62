
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;

@TeleOp(name="ControlCleanLeanAndMean", group="Control")
//@Disabled
public class ControlClean extends LinearOpMode implements OpModeAddition {
    //maths shit
    public class Point{
        public double x,y;

        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
    }



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
    public boolean flagAimbot = false, modeAimbot = false;

    private float speed;
    private float direction;
    private float right;
    private float left;

    public double dist(double x1, double y1, double x2, double y2) {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    public Point wl = new Point(-50,5);
    public Point wm = new Point(-31,5);
    public Point wr = new Point(-13,5);

    public Position pos;
    public double angleBetween(double x, double y)
    {

        double a = toDegrees(atan2(y - pos.y,x - pos.x) );
        return (a + 360) % 360;
    }
    @Override
    public void runOpMode() {

        robot = new RobotEx(hardwareMap,this, telemetry);

        waitForStart();

        runtime.reset();

        robot.motorArm.setTargetPosition(200);

        double rangle = robot.getAng3();
        double angoff = 0;




        double armSpeed = gamepad2.right_trigger - gamepad2.left_trigger;

        boolean in = false;
        boolean out = false;

        while (opModeIsActive()){

            pos = robot.getPosVuforia();

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

            if (gamepad1.y && !flagAimbot) {
                flagAimbot = true;
                modeAimbot = !modeAimbot;

                telemetry.addData("sflag: ", flagAimbot);
            } else if (!gamepad2.dpad_left && flagAimbot) {
                flagAimbot = false;
                telemetry.addData("sflag: ", flagAimbot);
            }

            if (gamepad2.left_bumper) {
                robot.motorIntake.setPower(1);
            } else if (gamepad2.right_bumper) {
                robot.motorIntake.setPower(-1);
            } else {
                robot.motorIntake.setPower(0);
            }

            robot.motorLauncher.setPower(gamepad2.right_trigger > 0.55 ? 0.55 : gamepad2.right_trigger);

            rangle = robot.getAng3();

            double[] actv = robot.vecRotate(-45 * gamepad1.left_stick_x,-45 * gamepad1.left_stick_y, (rangle-angoff));

            if (gamepad1.a && !flag) {
                flag = true;
                mode = !mode;
                telemetry.addData("flag: ", flag);
            } else if (!gamepad1.a && flag) {
                flag = false;
                telemetry.addData("flag: ", flag);
            }
            telemetry.addData("mode: ", mode);

            if(modeAimbot){
                if(dist(pos.x,pos.y,wm.x,wm.y)<4*4){
                    modeAimbot = false;
                }else{
                    double angbet = angleBetween(wm.x,wm.y);
                    actv = robot.vecRotate(-45 * cos(Math.PI * angbet / 180),-45 * sin(Math.PI * angbet / 180) , (rangle-angoff));
                }
            }
            //daca nu merge gen pula pizda coaiele au inceput razboaiele
            //adauga like 90 si 180 la x si y la pozitia robotului si plm
            //calculezi asa dupa si gata lol

            else if (mode) {
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
            }


            telemetry.update();

        }
    }
    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}


