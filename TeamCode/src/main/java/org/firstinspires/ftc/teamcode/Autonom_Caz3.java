package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonom_Caz3", group="Autonom_v1")
public class Autonom_Caz3 extends LinearOpMode implements OpModeAddition {

    //Declare OpMode members
    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        robot = new Robot(hardwareMap, this, telemetry);
        runtime.reset();

        robot.setSpeed(0.7);
        /*robot.motorRF.setPower(-0.5);
        robot.motorRB.setPower(-0.5);
        robot.motorLF.setPower(-0.5);
        robot.motorLB.setPower(-0.5);*/
        sleep(4500);
        robot.mortusMotorus();
        robot.setSpeed(-1);
        sleep(1200);
        robot.mortusMotorus();
    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
