package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TraianTestThrow", group="Control")
//@Disabled
public class TraianTestThrow extends LinearOpMode implements OpModeAddition {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor motor;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        motor = hardwareMap.get(DcMotor.class,"motor");
        motor.setDirection(DcMotor.Direction.REVERSE);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            motor.setPower(gamepad1.right_trigger);
        }
    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}
