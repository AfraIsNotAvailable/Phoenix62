/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the 
 * Modern Robotics and Hitechnic DC Motor Controllers) do not.


@Autonomous(name="Inolove", group = "Pushbot")
public class AutonomPantaVlad extends LinearOpMode implements OpModeAddition{
    // our DC motor.


    public static final double NEW_P = 0;
    public static final double NEW_I = 0;
    public static final double NEW_D = 0;
    //Robot robot;
    public void runOpMode() {

        // get reference to DC motor.
        robot = new Robot(hardwareMap,this,telemetry);

        // wait for start command.
        waitForStart();

       // robot.driveOnCM(100,1);
        sleep(2000);

        for(int motorIndex = 0; motorIndex < 4; ++motorIndex)
        {
            DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)robot.motorRB.getController();
            PIDFCoefficients pidOrig = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D,0);
            motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
            PIDFCoefficients pidModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //robot.driveOnCM(100,1);
        //ssleep(2000);

        robot.driveOnCM(100,1);

    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}*/