
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonom", group="Pushbot")


//@Disabled
public class Autonom extends LinearOpMode implements OpModeAddition {

    Robot robot;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public boolean isOpModeIsActive(){
        return opModeIsActive();
    }

    class RunnableDemo implements Runnable {
        private Thread t;
        private String threadName;

        RunnableDemo( String name) {
            threadName = name;
            telemetry.addLine("Creating " +  threadName );
        }

        public void run() {
            telemetry.addLine("Running " +  threadName );
            try {
                for(int i = 4; i > 0; i--) {
                    telemetry.addLine("Thread: " + threadName + ", " + i);

                    // Let the thread sleep for a while.l9
                    Thread.sleep(50);
                }
            } catch (InterruptedException e) {
                telemetry.addLine("Thread " +  threadName + " interrupted.");


            }
            telemetry.addLine("Thread " +  threadName + " exiting.");


        }

        public void start () {
            telemetry.addLine("Starting " +  threadName );

            if (t == null) {
                t = new Thread (this, threadName);
                t.start ();
            }
        }
    }

    @Override
    public void runOpMode() {
        waitForStart();
//        RunnableDemo R1 = new RunnableDemo( "Thread-1");
//        R1.start();
//
//        RunnableDemo R2 = new RunnableDemo( "Thread-2");
//        R2.start();
//
//        telemetry.update();
//
//        sleep(10000);


    }
}
