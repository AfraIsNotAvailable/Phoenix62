package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.exp;
import static java.lang.Math.round;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.time.Duration.ofSeconds;


public class Robot {
    //public HardwareMap h;

    public double fieldLimX = CMtoUnits(179);
    public double fieldLimY = CMtoUnits(358);

    public DcMotor motorRB = null;
    public DcMotor motorRF = null;
    public DcMotor motorLB = null;
    public DcMotor motorLF = null;
    public DcMotor motorWobble = null;

    public Servo servoWobble = null;

    public BNO055IMU gyro = null;

    public OpModeAddition opMode = null;
    public Telemetry debug = null;

    public double posX = 0, posY = 0;
    public double ang = 90;

    public Orientation lastAngle;
    public double GlobalAngle;
    public double rAngle;

    public Robot()
    {


    }//for polymorphism

    public Robot(HardwareMap h, OpModeAddition om,Telemetry t)
    {

        this.opMode = om;
        this.debug = t;

        motorRB = h.get(DcMotor.class, "motorRB");
        motorRF = h.get(DcMotor.class, "motorRF");
        motorLB = h.get(DcMotor.class, "motorLB");
        motorLF = h.get(DcMotor.class, "motorLF");
        motorWobble = h.get(DcMotor.class, "motorArm");

        gyro = h.get(BNO055IMU.class , "gyro" );

        // De aici in jos puteti sa ignorati ce scrie dar e pentru good practise
        // Motor run mode using the encoders

        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        gyro.initialize(parameters);

        while(opMode.isOpModeIsActive() && !gyro.isGyroCalibrated()){ }
    }

    public Robot(HardwareMap h, OpModeAddition om,Telemetry t, boolean nogyro)
    {

        this.opMode = om;
        this.debug = t;

        motorRB = h.get(DcMotor.class, "motorRB");
        motorRF = h.get(DcMotor.class, "motorRF");
        motorLB = h.get(DcMotor.class, "motorLB");
        motorLF = h.get(DcMotor.class, "motorLF");
        motorWobble = h.get(DcMotor.class, "motorWobble");

//        motorBrat = h.get(DcMotor.class, "motorBrat");

//        servoCleste1 = h.get(Servo.class, "servo1");
//        servoCleste2 = h.get(Servo.class, "servo2");
//        servoWobble = h.get(Servo.class, "servo3");

        //  gyro = h.get(BNO055IMU.class , "gyro" );

        // De aici in jos puteti sa ignorati ce scrie dar e pentru good practise
        // Motor run mode using the encoders

        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        // ---
//        this.motorLF.setDirection(DcMotor.Direction.REVERSE);
        // ---

        //  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //    parameters.mode = BNO055IMU.SensorMode.IMU;
        ///   parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        ///   parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // parameters.loggingEnabled = false;

        // gyro.initialize(parameters);

        //  while(opMode.isOpModeIsActive() && !gyro.isGyroCalibrated()){ }
    }

    public double motorLimit(double power1, double power2) {
        double result = power1 + power2;
        if (result > 1)
            result = 1;
        if (result < -1)
            result = -1;
        return result;
    }

    public int degToEncoder(double t)
    {
        return  (int)(t * 11.687704/65);
    }

    public int eulerTime(int t)
    {
        return (int)((100*t)/1.58);
    }

    public int tatamiToEuler2(double t)
    {
        return (int)((100*t)/1.29166666);
    }

    public int CMtoEuler(double t)
    {
        return tatamiToEuler2((2*t)/119.4);
    }

    public int tatamiToEuler(int x)
    {
        return (int)Math.round(x*(100/1.54)/1.232);
    }

    public double unitsToCM(double x)
    {
        return Math.round(x*100/16.444);
    }

    public double CMtoUnits(double x)
    {
        return Math.round(x*16.444/100);
    }

    public int unitsToEuler(double x)
    {
        return CMtoEuler(unitsToCM(x));
    }

    public int cmToEncoder(double x)
    {
        return (int)x*100/77;
    }

    public void ResetAngle()
    {
        lastAngle = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        GlobalAngle = 0;
    }

    public double getAngle()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }

    public double getAngle2()
    {
        Orientation aNgLeS = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = aNgLeS.firstAngle - lastAngle.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        GlobalAngle += deltaAngle;

        lastAngle = aNgLeS;

        return GlobalAngle;
    }

    public void turn2 ( double angle, double speed )
    {
        ResetAngle();
        if (angle < 0)
        {
            double tempAng = angle + 1;

            //this.motorLF.setPower(speed);
            //  this.motorLB.setPower(speed);
            //  this.motorRF.setPower(-speed);
            // this.motorRB.setPower(-speed);*/
            debug.addData("unghi: ", tempAng);
            debug.addData("unghi2: ", angle-7);
            debug.update();
            ElapsedTime rt = new ElapsedTime();
            rt.reset();

            while(opMode.isOpModeIsActive() && tempAng > angle )
            {
                //1 - (1 - getAngle2()/angle)
                tempAng = getAngle2();

                this.motorLF.setPower(speed);
                this.motorLB.setPower(speed);
                this.motorRF.setPower(-speed);
                this.motorRB.setPower(-speed);
                debug.addData("unghi: ", tempAng);
                debug.addData("unghi2: ", angle);
                debug.update();
                while (rt.milliseconds()<=25){}
                rt.reset();
            }
        }
        if (angle > 0)
        {
            double tempAng = -1*angle + 1;
            ElapsedTime rt = new ElapsedTime();
            rt.reset();

            while(opMode.isOpModeIsActive() && tempAng > -1*angle )
            {
                tempAng = getAngle2();

                this.motorLF.setPower(-speed);
                this.motorLB.setPower(-speed);
                this.motorRF.setPower(speed);
                this.motorRB.setPower(speed);

                debug.addData("unghi: ", tempAng);
                debug.addData("unghi2: ", tempAng);
                debug.update();
                while (rt.milliseconds()<=25){}
                rt.reset();
            }
        }

        this.mortusMotorus();

        rAngle = (rAngle + getAngle2()) % 360;

        ResetAngle();
    }

    public void setAng2(double ang, double s)
    {
        this.turn2(ang - this.rAngle, s);
    }

    /**
     * Returns constant K with respect to the ratio of the motor
     *
     * @param  iRatio the ratio of the motor, such as the 40 from 40:1
     */
    public double k(int iRatio)
    {
        double reductie = iRatio*1.5;
        double coutPerRev = 28;
        double wheelDiam = 4.0 * 2.54;
        return (reductie * coutPerRev) / (wheelDiam * 3.14);
    }

    /**
     * Returns constant K with respect to the ratio of the motor.
     * Returns an integer instead of a double, because I hate casts.
     *
     * @param  iRatio the ratio of the motor, such as the 40 from 40:1
     */
    public int ki(double iRatio)
    {
        double reductie = iRatio*1.5;
        double coutPerRev = 28;
        double wheelDiam = 4.0 * 2.54;
        return (int)((reductie * coutPerRev) / (wheelDiam * 3.14));
    }

    public void resetMotors()
    {
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void mortusMotorus()
    {
        this.motorLB.setPower(0);
        this.motorRF.setPower(0);
        this.motorLF.setPower(0);
        this.motorRB.setPower(0);

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setSpeed(double speed)
    {
        this.motorLB.setPower(speed);
        this.motorRF.setPower(speed);
        this.motorLF.setPower(speed);
        this.motorRB.setPower(speed);
    }

    public void setTarget(int target)
    {
        this.motorRB.setTargetPosition(motorLB.getCurrentPosition() + target);
        this.motorLF.setTargetPosition(motorLF.getCurrentPosition() + target);
        this.motorRF.setTargetPosition(motorRB.getCurrentPosition() + target);
        this.motorLB.setTargetPosition(motorRF.getCurrentPosition() + target);
    }

    public double angleBetween(double x, double y)
    {
        //  debug.addData("FNDsANG: ",toDegrees(atan2(y - this.posY,x - this.posX) ));
        //  debug.addData("X: ",x - this.posX);
        //  debug.addData("Y: ",y - this.posY);
        //  debug.addData("RX: ",this.posX);
        //  debug.addData("RY: ",this.posY);
        //debug.update();
        double a = toDegrees(atan2(y - this.posY,x - this.posX) );
        return (a + 360) % 360;
    }

    public void setAng(double t,double s)
    {
        // this.turn(t - this.getAngle(),s);
        setAng2(t,s);
    }

    public void slide(int dir, double speed)
    {
        this.motorRF.setPower(dir * speed);
        this.motorRB.setPower(dir * -1 * speed);
        this.motorLF.setPower(dir * -1 * speed);
        this.motorLB.setPower(dir * speed);
    }

    public void turn(double t,double s)
    {
        /*
        this.resetMotors();
        double target = (this.getAngle() + t)%360;

        if(target > this.getAngle()){
            while(opMode.isOpModeIsActive() && this.getAngle() < abs(target)) {
                this.motorLB.setPower(s);
                this.motorRF.setPower(-s);
                this.motorLF.setPower(s);
                this.motorRB.setPower(-s);
                debug.addData("A1 : ", 180 - abs(abs(target - this.getAngle()) - 180) );
                debug.addData("T : ", target);
                debug.addData("C : ", this.getAngle() );
                debug.update();
            }
        }else {
            while(opMode.isOpModeIsActive() && this.getAngle() > abs(target)) {
                this.motorLB.setPower(-s);
                this.motorRF.setPower(s);
                this.motorLF.setPower(-s);
                this.motorRB.setPower(s);
                debug.addData("A1 : ", 180 - abs(abs(target - this.getAngle()) - 180) );
                debug.addData("T : ", target);
                debug.addData("C : ", this.getAngle() );
                debug.update();
            }
        }*/
        this.turn2(t,s);
    }

    public void drive(int Target, double Speed)
    {
        Target = (int)(Target * ki(13.7));
        if(Target < 0) Speed = Speed*(-1);

        this.motorLB.setTargetPosition(this.motorLB.getCurrentPosition() + Target);
        this.motorRB.setTargetPosition(this.motorRB.getCurrentPosition() + Target);
        this.motorLF.setTargetPosition(this.motorLF.getCurrentPosition() + Target);
        this.motorRF.setTargetPosition(this.motorRF.getCurrentPosition() + Target);

        this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.setSpeed(Speed);

        while(this.motorLB.isBusy() && this.motorRB.isBusy() && this.motorLF.isBusy() && this.motorRF.isBusy() && opMode.isOpModeIsActive()){


            debug.addData("right_encoder_front", this.motorRF.getCurrentPosition());
            debug.addData("right_encoder_back", this.motorRB.getCurrentPosition());
            debug.addData("left_motor_back", this.motorLB.getCurrentPosition());
            debug.addData("left_motor_front", this.motorLF.getCurrentPosition());
            debug.addData("target", this.motorLB.getTargetPosition());
            debug.update();
        }

        mortusMotorus();

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void curve(double angle ,double speed)
    {
        ResetAngle();
        if (angle < 0)
        {
            double tempAng = getAngle2();

            //this.motorLF.setPower(speed);
            //  this.motorLB.setPower(speed);
            //  this.motorRF.setPower(-speed);
            // this.motorRB.setPower(-speed);*/
            debug.addData("unghi: ", tempAng);
            debug.addData("unghi2: ", angle-7);
            debug.update();

            while(opMode.isOpModeIsActive() && -1*tempAng > angle)
            {
                //1 - (1 - getAngle2()/angle)
                tempAng = getAngle2();


                this.motorRF.setPower(speed);
                this.motorRB.setPower(speed);
                debug.addData("unghi: ", tempAng);
                debug.addData("unghi2: ", angle);
                debug.update();
            }
        }
        if (angle > 0)
        {
            double tempAng = getAngle2();


            while(opMode.isOpModeIsActive() && tempAng > -1*angle)
            {
                tempAng = getAngle2();

                this.motorLF.setPower(speed);
                this.motorLB.setPower(speed);

                debug.addData("unghi: ", tempAng);
                debug.addData("unghi2: ", tempAng);
                debug.update();
            }
        }

        this.mortusMotorus();

        rAngle = (rAngle + getAngle2()) % 360;

        ResetAngle();
    }

    public void driveOnFunction(int target)
    {
        this.resetMotors();
        target = -target;
        double speed = 0.368; //drq stie dc, nu ne atingem de ancient magic
        double dist = 0;
        target *= ki(13.7);//ki(52);

        this.setTarget(target);

        this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double Target = (double)target;

        // e^-x^4 is more square, thus we get more max speed time
        speed = exp((-1)* Math.pow(((2/Target)*motorLB.getCurrentPosition() -  1),4));

        this.setSpeed(speed);

        while(this.motorRF.isBusy() && this.motorRB.isBusy() && this.motorLF.isBusy() && this.motorLB.isBusy() && this.opMode.isOpModeIsActive()){
            speed = exp((-1)* Math.pow(((2/Target)*motorLB.getCurrentPosition() -  1),4));
            //dist += speed;
            this.setSpeed(speed);

            if(speed > 0){
                this.posY += sin(this.getAngle()* PI/180) * speed;
                this.posX += cos(this.getAngle()* PI/180) * speed;
            }

            debug.addData("POSX: ",this.posX);
            debug.addData("POSY: ",this.posY);
            debug.addData("SPEED ",speed);
            debug.addData("X COS ",cos(this.getAngle()* PI/180) * speed);
            debug.addData("Y SIN ",sin(this.getAngle()* PI/180) * speed);
            debug.addData("ANGLE: ",this.angleBetween(this.CMtoUnits(179),this.CMtoUnits(179)));
            debug.update();

            debug.update();
        }


        this.mortusMotorus();
        this.resetMotors();

    }

    public void slideEncoder(int Target, double Speed)
    {
        Target = (int)(Target * ki(13.7));
        if(Target < 0) Speed = Speed*(-1);

        this.motorLB.setTargetPosition(this.motorLB.getCurrentPosition() - Target);
        this.motorRB.setTargetPosition(this.motorRB.getCurrentPosition() + Target);
        this.motorLF.setTargetPosition(this.motorLF.getCurrentPosition() + Target);
        this.motorRF.setTargetPosition(this.motorRF.getCurrentPosition() - Target);

        this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motorRF.setPower(Speed);
        this.motorRB.setPower(-Speed);
        this.motorLF.setPower(-Speed);
        this.motorLB.setPower(Speed);

        while(this.motorLB.isBusy() && this.motorRB.isBusy() && this.motorLF.isBusy() && this.motorRF.isBusy() && opMode.isOpModeIsActive()){

            debug.addData("right_encoder_front", this.motorRF.getCurrentPosition());
            debug.addData("right_encoder_back", this.motorRB.getCurrentPosition());
            debug.addData("left_motor_back", this.motorLB.getCurrentPosition());
            debug.addData("left_motor_front", this.motorLF.getCurrentPosition());
            debug.addData("target", this.motorLB.getTargetPosition());
            debug.update();
        }

        mortusMotorus();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveOnCM(int t, double s)
    {
        this.posY += sin(this.ang * PI/180) * t;
        this.posX += cos(this.ang * PI/180) * t;
        this.drive(this.cmToEncoder(t),-1*s);
    }

    public void driveOnFn(int t)
    {

        this.driveOnFunction(this.CMtoEuler(-1*t));
    }

    public void slideOnCM(int t, double s)
    {

        this.slideEncoder(t*100/70,s);
    }

    public void curveNoGyro(double t, double s)
    {
        if(t<0)
        {
            t = t*100/59.6;//nr magic pt curba
            int T = (int)(t * ki(13.7));
            if(s<0) T *= -1;
            this.motorRF.setTargetPosition(this.motorRB.getCurrentPosition() - T);
            this.motorRB.setTargetPosition(this.motorRF.getCurrentPosition() - T);

            this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.motorRB.setPower(-s);
            this.motorRF.setPower(-s);

            while(this.motorRB.isBusy() && this.motorRF.isBusy()  && opMode.isOpModeIsActive()){
                debug.addData("right_encoder_front", this.motorRF.getCurrentPosition());
                debug.addData("right_encoder_back", this.motorRB.getCurrentPosition());
                debug.addData("left_motor_back", this.motorRB.getCurrentPosition());
                debug.addData("left_motor_front", this.motorRF.getCurrentPosition());
                debug.addData("target", this.motorRB.getTargetPosition());
                debug.update();
            }

            mortusMotorus();
            this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            t = t*100/59.6;//nr magic pt curba
            int T = (int)(t * ki(13.7));
            if(s<0) T *= -1;

            this.motorLF.setTargetPosition(this.motorLB.getCurrentPosition() + T);
            this.motorLB.setTargetPosition(this.motorLF.getCurrentPosition() + T);

            this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.motorLB.setPower(s);
            this.motorLF.setPower(s);

            while(this.motorLB.isBusy() && this.motorLF.isBusy()  && opMode.isOpModeIsActive()){
                debug.addData("right_encoder_front", this.motorRF.getCurrentPosition());
                debug.addData("right_encoder_back", this.motorRB.getCurrentPosition());
                debug.addData("left_motor_back", this.motorLB.getCurrentPosition());
                debug.addData("left_motor_front", this.motorLF.getCurrentPosition());
                debug.addData("target", this.motorLB.getTargetPosition());
                debug.update();
            }

            mortusMotorus();
            this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void turnNoGyro(double t, double s)
    {
        t = t*50/61.2;
        int T = (int)(t * ki(13.7));
        if(t < 0) s = s*(-1);
        if(t < 0) T *= -1;
        if(T < 2)
            return;
        this.motorRB.setTargetPosition(this.motorLB.getCurrentPosition() + T);
        this.motorRF.setTargetPosition(this.motorRB.getCurrentPosition() + T);
        this.motorLB.setTargetPosition(this.motorLF.getCurrentPosition() - T);
        this.motorLF.setTargetPosition(this.motorRF.getCurrentPosition() - T);

        this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.motorRB.setPower(s);
        this.motorRF.setPower(s);
        this.motorLB.setPower(-s);
        this.motorLF.setPower(-s);

        while(this.motorLB.isBusy() && this.motorRB.isBusy() && this.motorLF.isBusy() && this.motorRF.isBusy() && opMode.isOpModeIsActive()){
            debug.addData("right_encoder_front", this.motorRF.getCurrentPosition());
            debug.addData("right_encoder_back", this.motorRB.getCurrentPosition());
            debug.addData("left_motor_back", this.motorLB.getCurrentPosition());
            debug.addData("left_motor_front", this.motorLF.getCurrentPosition());
            debug.addData("target", this.motorLB.getTargetPosition());
            debug.update();
        }

        mortusMotorus();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.ang = (this.ang + t) % 360;
    }

    public void slide45St(double speed)
    {
        this.motorRF.setPower(speed);
        this.motorLB.setPower(speed);
    }

    public void slide45Dr(double speed)
    {
        this.motorRF.setPower(speed);
        this.motorLB.setPower(speed);
    }

    public void goTo(double x, double y)
    {
        this.setAngle(this.angleBetween(x,y));
        this.driveOnCM((int)sqrt((x-this.posX)*(x-this.posX) + (y-this.posY)*(y-this.posY)),0.7);
        this.posX = x;
        this.posY = y;
    }

    public void setAngle(double t)
    {
        this.turnNoGyro( t - this.ang,0.5);
        this.ang = t;
    }
}