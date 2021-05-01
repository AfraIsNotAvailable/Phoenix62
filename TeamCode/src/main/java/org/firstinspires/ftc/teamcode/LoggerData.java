package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class LoggerData {

    RobotEx robot = null;

    public FileWriter writer;

    boolean fileCreated;

    public double speedLB;
    public double speedLF;
    public double speedRB;
    public double speedRF;

    public LoggerData(RobotEx r)
    {
        robot = r;
        fileCreated = false;
        speedLB = 0;
        speedLF = 0;
        speedRF = 0;
        speedRB = 0;
    }

    public String getValues() {

        String values = null;
        values += Double.toString(speedLF);
        values += ",";
        values += Double.toString(speedRF);
        values += ",";
        values += Double.toString(speedLB);
        values += ",";
        values += Double.toString(speedRB);
        values += ",";
        values += Float.toString(gamepad1.left_stick_x);
        values += ",";
        values += Float.toString(gamepad1.left_stick_y);
        values += ",";
        values += Float.toString(gamepad1.right_stick_x);
        values += ",";
        values += Double.toString(robot.getAng3());
        return values;
    }

    public String getHeader() {
        return "LF, RF, LB, RB, LSX, LSY, RSY, ANGHEL";
    }

    public void generateLogFile(String fileName) {
        try {
            File root = new File(Environment.getExternalStorageDirectory(), "Notes");
            if (!root.exists()) {
                root.mkdirs();
            }
            String timeStamp = new SimpleDateFormat("yyyy.MM.dd HH:mm:ss").format(new Date());
            timeStamp += "-";
            timeStamp += fileName;
            timeStamp += ".csv";
            File gpxfile = new File(root, timeStamp);
            writer = new FileWriter(gpxfile);
            writer.append(getHeader() );
            writer.append( "\n" );
            fileCreated = true;
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }


    public void writeLogLine() {

        speedLB = robot.motorLB.getVelocity(AngleUnit.RADIANS);
        speedLF = robot.motorLF.getVelocity(AngleUnit.RADIANS);
        speedRB = robot.motorRB.getVelocity(AngleUnit.RADIANS);
        speedRF = robot.motorRF.getVelocity(AngleUnit.RADIANS);

        try {
            if ( fileCreated ) {
                writer.append(getValues());
                writer.append("\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void closeFile() {

        try {
            writer.close();
        } catch (IOException e){
            e.printStackTrace();
        }

    }
}
