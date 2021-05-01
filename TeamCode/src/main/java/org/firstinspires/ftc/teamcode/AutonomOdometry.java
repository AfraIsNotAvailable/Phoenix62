

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutonomOdometry", group="Pushbot")
//@Disabled
public class AutonomOdometry extends LinearOpMode implements OpModeAddition {

    Robot robot = null;

    @Override
    public boolean isOpModeIsActive(){
        return opModeIsActive();
    }

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot =  new Robot(hardwareMap,this,telemetry);;
        // run until the end of the match (driver presses STOP)

        robot.driveOnFunction(robot.eulerTime(1));
        sleep(3000);
        robot.driveOnFunction(robot.eulerTime(1));
        sleep((15000));
    }
}
