package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ControlMewTwoStrikesAgain", group="Control")
//@Disabled
public class ControlNewNew extends LinearOpMode implements OpModeAddition {

    // Declare OpMode members
    public RobotEx robot = null;
    private ElapsedTime runtime = new ElapsedTime();

    public double speed;

    @Override
    public void runOpMode() throws InterruptedException {
        // Waity waity time
        waitForStart();
        robot = new RobotEx(hardwareMap, this, telemetry);
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("time: ", runtime.toString());
            telemetry.update();

            double y = -gamepad1.left_stick_y; // Reversed
            double x = gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing - change later
            double rx = gamepad1.right_stick_x;

            double backRightPow = y + x - rx;
            double frontRightPow = y - x - rx;
            double backLeftPow = y - x + rx;
            double frontLeftPow = y + x + rx;

            // Power limiter to -1 and 1
            if (Math.abs(frontLeftPow) > 1 || Math.abs(backLeftPow) > 1 || Math.abs(frontRightPow) > 1 || Math.abs(backRightPow) > 1) {

                double max = 0;

                max = Math.max(Math.abs(frontLeftPow), Math.abs(backLeftPow));
                max = Math.max(Math.abs(frontRightPow), max);
                max = Math.max(Math.abs(backRightPow), max);

                backRightPow /= max;
                frontRightPow /= max;
                backLeftPow /= max;
                frontLeftPow /= max;
            }

            robot.motorRB.setPower(backRightPow);
            robot.motorRF.setPower(frontRightPow);
            robot.motorLB.setPower(backLeftPow);
            robot.motorLF.setPower(frontLeftPow);
        }
    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
