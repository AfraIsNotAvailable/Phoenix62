package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonom_Caz1", group="Autonom_v1")
//@Disabled
public class Autonom_Caz1 extends LinearOpMode implements  OpModeAddition {

    Robot robot;
    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot = new Robot(hardwareMap, this, telemetry);
        runtime = new ElapsedTime();
        runtime.reset();

        robot.setSpeed(0.7);
        sleep(2400);
        robot.setSpeed(-1);
        sleep(600);
        robot.mortusMotorus();
        sleep(100);
       // robot.slide(true, 0.7);
        sleep(1000);
        robot.setSpeed(1);
        sleep(700);
    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
