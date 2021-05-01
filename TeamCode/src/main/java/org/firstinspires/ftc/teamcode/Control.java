
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModeAddition;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Control", group="Control")
//@Disabled
public class Control extends LinearOpMode implements OpModeAddition {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public Robot robot = null;

    public int p1;
    public int p0;

    private float speed;
    private float direction;
    private float right;
    private float left;

    private int btoi(boolean b){
        if(b) return  1;
        else return 0;
    }
    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot = new Robot(hardwareMap,this, telemetry);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("time: ",runtime.toString());
            telemetry.update();
            speed = gamepad1.left_trigger - gamepad1.right_trigger;
            direction = -gamepad1.left_stick_x;

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

            if (gamepad1.right_stick_x != 0) {
                robot.motorLB.setPower(-gamepad1.right_stick_x);
                robot.motorLF.setPower(gamepad1.right_stick_x);
                robot.motorRB.setPower(gamepad1.right_stick_x);
                robot.motorRF.setPower(-gamepad1.right_stick_x);
            }
            else {
                robot.motorLB.setPower(-left);
                robot.motorLF.setPower(-left);
                robot.motorRB.setPower(-right);
                robot.motorRF.setPower(-right);
            }

            robot.motorWobble.setPower(gamepad2.right_trigger);
            robot.motorWobble.setPower(-gamepad2.left_trigger);

            if(gamepad2.dpad_up)
            {
                robot.servoWobble.setPosition(0.95);
            }else if(gamepad2.dpad_down){
                robot.servoWobble.setPosition(0.5);
            }
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
        }
    }
    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
