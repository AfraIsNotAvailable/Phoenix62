package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
public class RobotEx /*extends Robot*/ {


    private static final String VUFORIA_KEY = "AbJRubT/////AAABma6KEOwi/UjmpKjNKZN3/NWMSd1P03DjYNZUmfA4zeI/6iDpgj7s7Xvujkc5tKEYP4QwNmtgXUf2kml1Nb0Pozf+iAxWwM+CPmCTFYks5fp0ckAtACUxtCCOluwQCn5NlU8vgBHwBNeVis+2j/26tO8B2Lh1bNz/RZLK9jbIVCQVQRPPAZ2+IpBPQogX3Dc5I4jktld7zcoTEOV1a7y+0sV006TBpV0KnanLQwXyZfDDfjNC1xDsladUdQ35JHU5N2fEwDOnWC7DLAZNU7UgLIPUH1EoHUbbilp4K5HrDqk4SYovwrEeHWccA9tzIE2oT4vsejcEQ99zFVa5+MhQhWKJSBnUTWj696jXeCNwrbm/";
    public OpModeAddition opMode = null;
    public Telemetry debug = null;
    public BNO055IMU gyro = null;

    public double posX = 0, posY = 0;
    public double ang = 90;
    // Motoare pentru Movement
    public DcMotorEx motorRB = null;
    public DcMotorEx motorRF = null;
    public DcMotorEx motorLB = null;
    public DcMotorEx motorLF = null;

    // Motoare pentru intake/launch
    public DcMotorEx motorLauncher = null;
    public DcMotorEx motorIntake = null;
    public DcMotorEx motorArm = null;

    // Obiectul de loggerdata
    public LoggerData loggerData;

    // Clasa de servouri (ce mama naibii fac aici doamne ajuta help mi)
    public Servo servoArm = null;

    // Motor Counts
    public DcMotorControllerEx mCont = null;

    // Senzori
    public BNO055IMU gyro2;
    public TouchSensor sensor;

    Position rPos;
    Velocity rVel;

    public double axA = 17.25; //cm
    public double axB = 17.25; //cm
    public double wheelR = 5;//cm
    public double maxRPM = 435;
    public double robotRadius = 29; // Thought Experiment: Daca inscri robotul intr-un cerc, obti raza asta - folosita de matematica complicata a lui Croi
    BNO055IMU.Parameters parameters; // Never used outside of functions - created, initialized and used inside the Robot constructor for example
    private int armpos = 1;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public List<VuforiaTrackable> allTrackables;

    public RobotEx(HardwareMap h, OpModeAddition om, Telemetry t) {
        this.opMode = om;
        this.debug = t;
        Robot robot2 = new Robot(h, om, t);

        this.rPos = new Position(DistanceUnit.CM,0,0,0,10);
        this.rVel = new Velocity(DistanceUnit.CM,0,0,0,10);

        // Motor inits
        motorRB = h.get(DcMotorEx.class, "motorRB");
        motorRF = h.get(DcMotorEx.class, "motorRF");
        motorLB = h.get(DcMotorEx.class, "motorLB");
        motorLF = h.get(DcMotorEx.class, "motorLF");
        motorLauncher = h.get(DcMotorEx.class, "motorLauncher");
        motorIntake = h.get(DcMotorEx.class,"motorIntake");
        motorArm = h.get(DcMotorEx.class, "motorArm");
        servoArm = h.get(Servo.class,"servoArm");
        //servoWobble = h.get(Servo.class, "servoWobble");
        servoArm.setDirection(Servo.Direction.REVERSE);


        mCont = (DcMotorControllerEx)this.motorRB.getController();

        // Sensor inits
        gyro = h.get(BNO055IMU.class , "gyro" );
        gyro2 = h.get(BNO055IMU.class , "gyro2" );

        //sensor = h.get(TouchSensor.class, "touch");

        // De aici in jos puteti sa ignorati ce scrie dar e pentru good practice
        // Motor run mode using the encoders

        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorLF.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motorLauncher.setDirection(DcMotor.Direction.REVERSE);
        this.motorArm.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        //parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.DEEP;
        //parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523;
        parameters.loggingEnabled = false;

        gyro.initialize(parameters);

        gyro2.initialize(parameters);

        webcamName = h.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
        VuforiaLocalizer.Parameters vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        vparameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        vparameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        vparameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vparameters);

        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");//asta
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");//asta
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");//asta
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");//asta

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(vparameters.cameraName, cameraLocationOnRobot);
        }
        targetsUltimateGoal.activate();
        while(opMode.isOpModeIsActive() && !gyro.isGyroCalibrated() && !gyro2.isGyroCalibrated()){ }
    }

    public void setSpeed(double speed) {
        this.motorRB.setPower(speed);
        this.motorRF.setPower(speed);
        this.motorLB.setPower(speed);
        this.motorLF.setPower(speed);
    }

    // Functie de launch
    public void startLauncher(double pow) {
        this.motorLauncher.setPower(pow);
    }
    public void stopLauncher() {
        this.motorLauncher.setPower(0);
    }
    public int ki(double iRatio)
    {
        double reductie = iRatio*1.5;
        double coutPerRev = 28;
        double wheelDiam = 4.0 * 2.54;
        return (int)((reductie * coutPerRev) / (wheelDiam * 3.14));
    }
    // Functie de intake
    public void startIntake() {
        this.motorIntake.setPower(1);
    }
    public void stopIntake() {
        this.motorIntake.setPower(0);
    }

    // max radians per second, used for force normalizing
    public double maxRads(double rpm)
    {
        return 2*Math.PI*rpm/60;
    }

    //  The good one
    public double getAng2()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }

    public double get2Ang2()
    {
        Orientation angles = gyro2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }

    // The best one
    public double getAng3()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Orientation angles2 = gyro2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return (angles.firstAngle + 360 + 90) % 360;
    }

    public double radsToMS(double rads){ return this.robotRadius * rads;  }

    public void grab(){
        this.servoArm.setPosition(0);
        sleep(1100);
    }
    public void drop(){
        this.servoArm.setPosition(0.50);
        sleep(1100);
    }
    public void shoot(double t){
        this.startIntake();
        sleep(t);
        this.stopIntake();
    }

    public double[] vecRotate(double x, double y, double theta)
    {
        double[] ret = new double[2];
        double rad = theta * Math.PI / 180;
        ret[0] = cos(rad) * x - sin(rad) * y;
        ret[1] = sin(rad) * x + cos(rad) * y;
        return ret;
    }

    public double[] normalize(double[] w, double wl)
    {
        double wm = -1;
        for(int i = 0; i < 4; ++i)
        {
            if(wm < w[i])
                wm = w[i];
        }
        if(wm < wl)
            return w;

        for(int i = 0; i < 4; ++i)
        {
            w[i] = wl * w[i] / wm;
        }
        return  w;

    }
    public int cmToEncoder(double x)
    {
        return (int)x*100/77;
    }
    public int targetOfW(double w, double wm, double x, double y)
    {
        return (int)((w/wm*this.cmToEncoder(sqrt(x*x + y*y)))*1.065);
    }

    // returns in radiani pe secunda
    public double[] getAngulars(double Vx, double Vy, double W)
    {
        double[] w = new double[4];

        w[0] = Vx - Vy - (this.axA + this.axB) * W;
        w[1] = Vx + Vy + (this.axA + this.axB) * W;
        w[2] = Vx + Vy - (this.axA + this.axB) * W;
        w[3] = Vx - Vy + (this.axA + this.axB) * W;

        w[0] *= 1/this.wheelR;
        w[1] *= 1/this.wheelR;
        w[2] *= 1/this.wheelR;
        w[3] *= 1/this.wheelR;

        return w;
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

    public void slideOnCM(int t, double s)
    {

        this.slideEncoder(t*100/70,s);
    }
    public void driveOnCM(int t, double s)
    {
        this.posY += sin(this.ang * PI/180) * t;
        this.posX += cos(this.ang * PI/180) * t;
        this.drive(this.cmToEncoder(t),-1*s);
    }
    public double[] getForce(double w1,double w2, double w3, double w4){
        double[] f = new double[3];

        f[0] = w1 + w2 + w3 + w4;
        f[1] = w1 - w2 - w3 + w4;
        f[2] = -w1/(axB+axA) + w2/(axB+axA) + -w3/(axB+axA) + w4/(axB+axA);

        return  f;
    }
    // Stops the motors (HP reference)
    public void sectumsempera()
    {
        this.motorLF.setPower(0);
        this.motorLB.setPower(0);
        this.motorRB.setPower(0);
        this.motorRF.setPower(0);
    }

    // Starts the motors (another HP reference)
    public void episkey()
    {
        this.motorLB.setMotorEnable();
        this.motorRB.setMotorEnable();
        this.motorLF.setMotorEnable();
        this.motorRF.setMotorEnable();
    }

    // in centimetri pe secunda cred
    public void setVelocity(double x, double y)
    {
        double[] anglualrMatrix = this.getAngulars(y,x,0);
        anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));

        //  this.episkey();

        this.motorLF.setVelocity(anglualrMatrix[0]/1,AngleUnit.RADIANS);
        this.motorRF.setVelocity(anglualrMatrix[1]/1,AngleUnit.RADIANS);
        this.motorRB.setVelocity(anglualrMatrix[3]/1,AngleUnit.RADIANS);
        this.motorLB.setVelocity(anglualrMatrix[2]/1,AngleUnit.RADIANS);

        //  this.sectumsempera();
    }

    // Functia jmechera de triplu-axel
    public void setPatinaj(double x, double y, double k, double t)
    {

        double actv[] = this.vecRotate(x,y, this.getAng3());
        this.setVelocityW(actv[0], actv[1],Math.PI*(1/k));

        ElapsedTime et = new ElapsedTime();
        et.reset();
        while(et.milliseconds() < t){}
        /*
        double mag = sqrt(x*x+y*y);
        double[] anglualrMatrix = this.getAngulars(y,x,2*Math.PI/k*mag);
        anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));

        //  this.episkey();

        this.motorLF.setVelocity(anglualrMatrix[0],AngleUnit.RADIANS);
        this.motorRF.setVelocity(anglualrMatrix[1],AngleUnit.RADIANS);
        this.motorRB.setVelocity(anglualrMatrix[3],AngleUnit.RADIANS);
        this.motorLB.setVelocity(anglualrMatrix[2],AngleUnit.RADIANS);


        //  this.sectumsempera();*/
    }

    public double getAngularVel()
    {
        return this.gyro.getAngularVelocity().toAngleUnit(AngleUnit.RADIANS).xRotationRate;
    }

    // setVelocity() but fancier
    public void setVelocityW(double x, double y, double W)
    {
        double[] anglualrMatrix = this.getAngulars(y,x,W);
        anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));
        //  this.episkey();

        ///1 pt ca gen nu e ratia corecta si cand ii dai 10 cm/s iti da 32
        this.motorLF.setVelocity(anglualrMatrix[0]/1,AngleUnit.RADIANS);
        this.motorRF.setVelocity(anglualrMatrix[1]/1,AngleUnit.RADIANS);
        this.motorRB.setVelocity(anglualrMatrix[3]/1,AngleUnit.RADIANS);
        this.motorLB.setVelocity(anglualrMatrix[2]/1,AngleUnit.RADIANS);
    }

    public void setVelocity(double x, double y,double t)
    {
        double[] anglualrMatrix = this.getAngulars(y,x,0);
        anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));

        ElapsedTime tm = new ElapsedTime();
        tm.reset();
        this.episkey();


        while (tm.milliseconds() < t)
        {
            this.motorLF.setVelocity(anglualrMatrix[0]/1,AngleUnit.RADIANS);
            this.motorRF.setVelocity(anglualrMatrix[1]/1,AngleUnit.RADIANS);
            this.motorRB.setVelocity(anglualrMatrix[3]/1,AngleUnit.RADIANS);
            this.motorLB.setVelocity(anglualrMatrix[2]/1,AngleUnit.RADIANS);
        }

        this.sectumsempera();
    }

    public void setOrientation(String orientation)
    {
        this.episkey();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        double delay = time.milliseconds();

        while(this.getAng3() > 90)
        {
            while (delay < time.milliseconds()) {

            }

            delay = time.milliseconds() + 50;

            this.motorRB.setVelocity(20,AngleUnit.RADIANS);
            this.motorRF.setVelocity(20,AngleUnit.RADIANS);
            this.motorLB.setVelocity(-20,AngleUnit.RADIANS);
            this.motorLF.setVelocity(-20,AngleUnit.RADIANS);
        }

        this.sectumsempera();
    }

    // Either -1 or 1
    private int sgn(double x)
    {
        if (x < 0)
            return  -1;
        else
            return 1;
    }

    public void goToF(double x, double y, double roc)
    {
        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.episkey();
        ElapsedTime time = new ElapsedTime();
        time.reset();
        double delay = time.milliseconds();
        double ang = 0;
        double tx = this.posX + x;
        double ty = this.posY + y;

        if(x != 0 && y != 0)
        {
            if(x>y)
                this.setVelocity(45,y/x*45);
            else
                this.setVelocity(x/y*45,45);
        }
        else
        {
            if(x == 0)
                this.setVelocity(0,sgn(y) * 45);
            else
                this.setVelocity(sgn(x) * 45,0);
        }

        debug.addData("px",this.posX);
        debug.addData("py",this.posY);
        debug.update();

        while(this.posX <= tx && this.posY <= ty)
        {
            if(delay < time.milliseconds())
            {
                delay = time.milliseconds() + 50;
                this.posX += x/20;
                this.posY += y/20;
                ang += roc;
                double[] v = this.vecRotate(x,y,ang);
                x = v[0];
                y = v[1];

                if(x != 0 && y != 0)
                {
                    if(x>y)
                        this.setVelocity(45,y/x*45,roc);
                    else
                        this.setVelocity(x/y*45,45,roc);
                }
                else
                {
                    if(x == 0)
                        this.setVelocity(0,sgn(y) * 45,roc);
                    else
                        this.setVelocity(sgn(x) * 45,0,roc);
                }
            }
        }
        this.sectumsempera();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //this.resetMotors();
    }

    public void goToF2(double x, double y)
    {
        this.episkey();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double[] anglualrMatrix = this.getAngulars(y,x,0);
        double wmax = -1;

        for(int i = 0; i < 4; ++i)
        {
            if(anglualrMatrix[i] > wmax)
            {
                wmax = anglualrMatrix[i];
            }
        }

        debug.addData("t1:",this.targetOfW(anglualrMatrix[0],wmax,x,y));
        debug.addData("t2:",this.targetOfW(anglualrMatrix[1],wmax,x,y));
        debug.addData("t3:",this.targetOfW(anglualrMatrix[2],wmax,x,y));
        debug.addData("t4:",this.targetOfW(anglualrMatrix[3],wmax,x,y));
        debug.addData("p1:",this.motorRB.getCurrentPosition());
        debug.addData("p2:",this.motorRF.getCurrentPosition());
        debug.addData("p3:",this.motorLB.getCurrentPosition());
        debug.addData("p4:",this.motorLF.getCurrentPosition());
        debug.update();

        int tg4 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[3],wmax,x,y) * ki(13.7));
        int tg2 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[1],wmax,x,y) * ki(13.7));
        int tg3 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[2],wmax,x,y) * ki(13.7));
        int tg1 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[0],wmax,x,y) * ki(13.7));

        this.motorRB.setTargetPosition(tg4);
        this.motorRF.setTargetPosition(tg2);
        this.motorLB.setTargetPosition(tg3);
        this.motorLF.setTargetPosition(tg1);

        anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));

        this.motorRB.setVelocity(anglualrMatrix[3],AngleUnit.RADIANS);
        this.motorLB.setVelocity(anglualrMatrix[2],AngleUnit.RADIANS);
        this.motorRF.setVelocity(anglualrMatrix[1],AngleUnit.RADIANS);
        this.motorLF.setVelocity(anglualrMatrix[0],AngleUnit.RADIANS);

        while(  this.motorLB.isBusy() ||
                this.motorRB.isBusy() ||
                this.motorLF.isBusy() ||
                this.motorRF.isBusy() ||
                opMode.isOpModeIsActive()){

            this.motorRB.setVelocity(anglualrMatrix[3],AngleUnit.RADIANS);
            this.motorLB.setVelocity(anglualrMatrix[2],AngleUnit.RADIANS);
            this.motorRF.setVelocity(anglualrMatrix[1],AngleUnit.RADIANS);
            this.motorLF.setVelocity(anglualrMatrix[0],AngleUnit.RADIANS);
        }

        this.sectumsempera();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.posX += x;
        this.posY += y;
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

        this.motorRB.setVelocity(45*Speed,AngleUnit.RADIANS);
        this.motorLB.setVelocity(45*Speed,AngleUnit.RADIANS);
        this.motorRF.setVelocity(45*Speed,AngleUnit.RADIANS);
        this.motorLF.setVelocity(45*Speed,AngleUnit.RADIANS);

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

    public void turnNoGyro(double t, double s)
    {
        this.episkey();
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

        this.motorRB.setVelocity(s*45,AngleUnit.RADIANS);
        this.motorLB.setVelocity(-s*45,AngleUnit.RADIANS);
        this.motorRF.setVelocity(s*45,AngleUnit.RADIANS);
        this.motorLF.setVelocity(-s*45,AngleUnit.RADIANS);

        while(this.motorLB.isBusy() && this.motorRB.isBusy() && this.motorLF.isBusy() && this.motorRF.isBusy() && opMode.isOpModeIsActive()){
            debug.addData("right_encoder_front", this.motorRF.getCurrentPosition());
            debug.addData("right_encoder_back", this.motorRB.getCurrentPosition());
            debug.addData("left_motor_back", this.motorLB.getCurrentPosition());
            debug.addData("left_motor_front", this.motorLF.getCurrentPosition());
            debug.addData("target", this.motorLB.getTargetPosition());
            debug.update();
        }

        this.sectumsempera();

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

    public void goToF2(double x, double y, double wang) {
        this.episkey();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double[] anglualrMatrix = this.getAngulars(y,x,wang);
        double wmax = -1;

        for(int i = 0; i < 4; ++i)
        {
            if(anglualrMatrix[i] > wmax)
            {
                wmax = anglualrMatrix[i];
            }
        }

        int tg4 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[3],wmax,x,y) * ki(13.7));
        int tg2 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[1],wmax,x,y) * ki(13.7));
        int tg3 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[2],wmax,x,y) * ki(13.7));
        int tg1 = (int)(this.motorLB.getCurrentPosition() +
                this.targetOfW(anglualrMatrix[0],wmax,x,y) * ki(13.7));

        this.motorRB.setTargetPosition(tg4);
        this.motorRF.setTargetPosition(tg2);
        this.motorLB.setTargetPosition(tg3);
        this.motorLF.setTargetPosition(tg1);

        anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));

        this.motorRB.setVelocity(anglualrMatrix[3],AngleUnit.RADIANS);
        this.motorLB.setVelocity(anglualrMatrix[2],AngleUnit.RADIANS);
        this.motorRF.setVelocity(anglualrMatrix[1],AngleUnit.RADIANS);
        this.motorLF.setVelocity(anglualrMatrix[0],AngleUnit.RADIANS);

        double lx = x, ly = y;
        double lang = this.getAng2();
        double dang = 0;
        double dvel = gyro.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).xRotationRate;
        double angv = dvel;
        double countang = 0;

        ElapsedTime dtime = new ElapsedTime();
        dtime.reset();

        while(  this.motorLB.isBusy() &&
                this.motorRB.isBusy() &&
                this.motorLF.isBusy() &&
                this.motorRF.isBusy() &&
                opMode.isOpModeIsActive()){

            dang = this.getAng2() - lang;
            double[] nv = vecRotate(x,y,-countang);
            lang = this.getAng2();
            lx = nv[0];
            ly = nv[1];

            dvel = angv - gyro.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).xRotationRate;
            angv = gyro.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).xRotationRate;
            countang += dvel;

            double anga = dvel/dtime.seconds();

            anglualrMatrix = this.getAngulars(lx,ly,anga);
            anglualrMatrix = normalize(anglualrMatrix,this.maxRads(this.maxRPM));

            this.motorRB.setVelocity(anglualrMatrix[3],AngleUnit.RADIANS);
            this.motorLB.setVelocity(anglualrMatrix[2],AngleUnit.RADIANS);
            this.motorRF.setVelocity(anglualrMatrix[1],AngleUnit.RADIANS);
            this.motorLF.setVelocity(anglualrMatrix[0],AngleUnit.RADIANS);

            dtime.reset();
        }

        this.sectumsempera();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.posX += x;
        this.posY += y;
    }

    public void goToF3(double x, double y, double w){
        this.episkey();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(w != 0){

        }else{
            double abs = sqrt(x*x + y*y);
            double estDist = 0;
            ElapsedTime t = new ElapsedTime();
            t.reset();

            while (estDist != abs)
            {
                double[] anglualrMatrix = this.getAngulars(y,x,0);
                anglualrMatrix = this.normalize(anglualrMatrix, this.maxRads(this.maxRPM));

                this.motorLF.setVelocity(anglualrMatrix[0],AngleUnit.RADIANS);
                this.motorRF.setVelocity(anglualrMatrix[1],AngleUnit.RADIANS);
                this.motorRB.setVelocity(anglualrMatrix[3],AngleUnit.RADIANS);
                this.motorLB.setVelocity(anglualrMatrix[2],AngleUnit.RADIANS);
            }
        }

        this.sectumsempera();

        this.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void sleep(double ms)
    {
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (t.milliseconds() < ms){}
    }
    //degrees
    public void setVelocityFieldCentric(final double x, final double y, final double w, final double tg) {
        final Timer timer = new Timer();

        class Calculate extends TimerTask {
            public double angle = 0;

            public void run() {
                if(angle >= tg)
                {
                    timer.cancel();
                    timer.purge();
                    return;
                }
                angle += tg / 10;
                double[] v = vecRotate(x, y, -angle);
                double[] anglualrMatrix = getAngulars(v[1], v[0], tg / 10);
                debug.log().add(toStr(anglualrMatrix));
                debug.update();
                //  double[] anglualrMatrix = getAngulars(v[1], v[0], w / 50); //posibil sa fie in radiani idk
                anglualrMatrix = normalize(anglualrMatrix, maxRads(maxRPM));

                motorLF.setVelocity(anglualrMatrix[0], AngleUnit.RADIANS);
                motorRF.setVelocity(anglualrMatrix[1], AngleUnit.RADIANS);
                motorRB.setVelocity(anglualrMatrix[3], AngleUnit.RADIANS);
                motorLB.setVelocity(anglualrMatrix[2], AngleUnit.RADIANS);
                //sleep(85);
                //maybe
            }
        }

        timer.schedule(new Calculate(), 0, 100);
    }

    private String toStr(double[] vec) {
        String s = "";
        for (int i = 0; i < 4; i++) {
            s += String.valueOf(vec[i]);
            s += ",";
        }
        return  s;
    }

    public void pause(double ms)
    {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(ms > elapsedTime.milliseconds()){}
    }

    public void move(double ang, double dist, double speed)
    {
        double[] f = vecRotate(0 ,speed , -(ang - 90));

        if(dist > 0) {
            this.setVelocity(10 * f[0] / 33.7, 10 * f[1] / 33.7, 1000 * (dist / speed));
            this.posX += f[0];
            this.posY += f[1];
        }else{
            this.setVelocity(-10 * f[0] / 33.7, -10 * f[1] / 33.7, 1000 * (-dist / speed));
            this.posX -= f[0];
            this.posY -= f[1];
        }
    }

    public void moveTest(double ang, double dist, double speed)
    {
        double[] f = vecRotate(speed*cos(Math.PI * ang / 180) ,speed*sin(Math.PI * ang / 180) , this.getAng3());

        if(dist > 0) {
            this.setVelocity(10 * f[0] / 33.7, 10 * f[1] / 33.7, 1000 * (dist / speed));
            this.posX += f[0];
            this.posY += f[1];
        }else{
            this.setVelocity(-10 * f[0] / 33.7, -10 * f[1] / 33.7, 1000 * (-dist / speed));
            this.posX -= f[0];
            this.posY -= f[1];
        }
    }

    public void patinaj(double ang, double dist, double speed,double rspeed)
    {
        this.episkey();
        double[] f = vecRotate(0 ,speed , 0);
        if(dist > 0) {
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();

            while(dist/speed*1000 > elapsedTime.milliseconds()){
                double actv[] = this.vecRotate(f[1] / 2 ,f[0] * 0, this.getAng3());
                actv = this.vecRotate(actv[0] ,actv[1] , -2 - (ang - 90));
                this.setVelocityW(actv[0],actv[1],-Math.PI/10.5/2 * rspeed);
            }

            this.posX += f[0];
            this.posY += f[1];
        }else{
            ElapsedTime elapsedTime = new ElapsedTime();
            elapsedTime.reset();

            while(dist/speed*1000 > elapsedTime.milliseconds()) {
                double actv[] = this.vecRotate(f[1] / 2, f[0] * 0, this.getAng3());
                actv = this.vecRotate(actv[0], actv[1], -2 - (ang - 90));
                this.setVelocityW(-actv[0], -actv[1], +Math.PI / 10.5 / 2 * rspeed);
            }
            this.posX -= f[0];
            this.posY -= f[1];
        }

        this.sectumsempera();
    }

    public void arm(int pos) {
        motorArm.setMotorEnable();
        motorArm.setTargetPosition(pos);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(motorArm.getCurrentPosition() < pos){
//           // while(opMode.isOpModeIsActive() && motorArm.isBusy()) {
//                motorArm.setPower(-0.5);
//            //}
//        }else{
        //while(opMode.isOpModeIsActive() && motorArm.isBusy()) {
        motorArm.setPower(0.5);
        //}
        //}
        //  motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (opMode.isOpModeIsActive() && motorArm.isBusy()){}
        //  motorArm.setMotorDisable();
//        motorArm.setPower(0);
    }
    public void arm() {
        if(this.armpos == 1){
            this.motorArm.setPower(0.25);
            pause(1000/2);
            this.motorArm.setPower(0);
            this.armpos = 0;
        }else{
            this.motorArm.setPower(-0.35);
            pause(1000/2);
            this.motorArm.setPower(0);
            this.armpos = 1;
        }
    }

    public Position getPosVuforia(){
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                debug.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }


        VectorF translation = lastLocation.getTranslation();
        debug.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) , translation.get(1) , translation.get(2) );
        Position ret = new Position();
        ret.x = translation.get(0)/10;
        ret.y = translation.get(1)/10;
        ret.z = translation.get(2)/10;

        if(!targetVisible){
            debug.addLine("Posible positional mismatch, tracking target not found!");
        }

        return ret;


    }


}
