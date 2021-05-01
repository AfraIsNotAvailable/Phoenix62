
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModeAddition;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="ControlNew", group="Control")
//@Disabled
public class ControlNew extends LinearOpMode implements OpModeAddition {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public Robot robot = null;

    public double speed;
    public double direction;
    public boolean precision;

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
            telemetry.addData("time: ", runtime.toString());
            telemetry.update();

            speed = gamepad1.right_trigger - gamepad1.left_trigger;
            direction = gamepad1.right_stick_x;
            precision = gamepad1.y;

            // Turn fara posibilitate de curve
            /*if ()
            else{
                robot.motorLB.setPower(robot.motorLimit(direction, speed));
                robot.motorLF.setPower(robot.motorLimit(direction, speed));
                robot.motorRB.setPower(robot.motorLimit(-direction, speed));
                robot.motorRF.setPower(robot.motorLimit(-direction, speed));
            }*/
        }
    }
    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
