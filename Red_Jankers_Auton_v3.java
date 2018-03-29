package org.firstinspires.ftc.teamcode.Auton;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.concurrent.SynchronousQueue;

/**
 * Created by Janker on 10/2/2017.
 */
@Autonomous(name="Red_Jankers Auton_v3",group="Auton")
@Disabled
public class Red_Jankers_Auton_v3 extends OpMode {

    //* Everything before next comment is defining variables to be accessed
    //* for the the rest of the time

    Hardware bot =  new Hardware();

    static final int HEADING_THRESHOLD = 1;

    static final double P_C = 0.0022;
    static final double I_C = 0.00000125;
    static final double D_C = 0;

    int iError = 0;
    int previousError = 0;

    public ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.25;
    static final double TURN_SPEED = 0.20;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    int state = -1;
    int previousState = -1;

    boolean Center = false;
    boolean Right = false;
    boolean Left = false;
    boolean servoCheck = false;
    int fixedHeading = 0;



    DcMotor rearLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor rearRightMotor;
    DcMotor frontRightMotor;

    DcMotor rightIntake;
    DcMotor leftIntake;
    DcMotor Lift;
    DcMotor Extension;

    Servo leftPivot;
    Servo relicClamp;
    Servo relicArm;
    Servo leftJewelArm;
    Servo flopperClamp;
    Servo flopperDick;

    ModernRoboticsI2cGyro gyro;


    ColorSensor blueColorSensor;

    DistanceSensor blueDistanceSensor;

   // OpticalDistanceSensor odsSensor;

    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    @Override
    public void init() {

        rearLeftMotor = hardwareMap.dcMotor.get("rL");
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        rearRightMotor = hardwareMap.dcMotor.get("rR");
        frontRightMotor = hardwareMap.dcMotor.get("fR");

        Lift = hardwareMap.dcMotor.get("L");
        Extension = hardwareMap.dcMotor.get("E");
        leftIntake = hardwareMap.dcMotor.get("lI");
        rightIntake = hardwareMap.dcMotor.get("rI");


        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);

        leftPivot = hardwareMap.servo.get("lP");
        relicArm = hardwareMap.servo.get("rA");
        relicClamp = hardwareMap.servo.get("rC");
        leftJewelArm = hardwareMap.servo.get("lJA");
        flopperClamp = hardwareMap.servo.get("fC");
        flopperDick = hardwareMap.servo.get("fD");


        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");


        blueColorSensor = hardwareMap.get(ColorSensor.class, "bColorDistance");

        blueDistanceSensor = hardwareMap.get(DistanceSensor.class, "bColorDistance");

        //odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");

        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.calibrate();

        leftPivot.setPosition(.68);
        leftJewelArm.setPosition(.87);
        relicClamp.setPosition(1);
        relicArm.setPosition(0);



        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AavXRvD/////AAAAGYbu90CPGkoGh7X1nRUxwjsWz20nEAcBDAXkRzK4SjWSsBKxl/Ixw/qo0zdSsFvZgQFU7omaLYZ6zjjPIXUWGf8C0f/KcZcbmwc2MrPRO6fI0i7rXSdI1pn9thjJOsjQ5imtpAFm2nT5dkorcWt1Blevx5LjpU99u2ysu8g92vHO6JeTD3g7V2r9Zm4Fo6K/bbfvqmywG9oqMJRGAq27hvgXhvwzORdSdO+TtDBXrFgAvDuQozaD9FqeRa2bwFMLZrJM7YCthG9RT9ENYSRdbrHLLUXgk43tOZhK2N4ec9JP0Xqj8HhKs3pAjtvq5f9Q5kPjQwkd6w40wJmXoULHy8B6vgjI7F8XYdPT2QpCTGPO";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();

    }

    @Override
    public void init_loop() {
        runtime.reset();
        telemetry.addData("time", runtime.time());
        gyro.resetZAxisIntegrator();

    }

    @Override
    public void loop() {

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (blueColorSensor.red() * SCALE_FACTOR),
                (int) (blueColorSensor.green() * SCALE_FACTOR),
                (int) (blueColorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Red  ", blueColorSensor.red());
        telemetry.addData("Blue ", blueColorSensor.blue());

        telemetry.addData("Case", state);
        telemetry.addData("gyro", gyro.getIntegratedZValue());
        telemetry.addData("vumark", vuMark);


        telemetry.update();

        switch (state) {
            case -1:
                if (this.resetDelay()) {
                    state = previousState + 1;
                }
                break;

            case 0:
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.addData("center", "true");
                    Center = true;
                    this.nextState();
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("left", "true");
                    Left = true;
                    this.nextState();
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("right", "true");
                    Right = true;
                    this.nextState();
                }
                break;


            case 1:
                if (!gyro.isCalibrating()) {
                    gyro.resetZAxisIntegrator();
                    this.nextState();
                }
                break;


            case 2:
                leftPivot.setPosition(.62);
                leftJewelArm.setPosition(.18);
                if (runtime.time() < 2) {
                    telemetry.addData("waiting", "for the world to change");

                } else {
                    if (blueColorSensor.red() < blueColorSensor.blue() + 5) {
                        leftPivot.setPosition(.8);
                        telemetry.addData("close", "bad science");
                        this.nextState();

                    } else {
                        leftPivot.setPosition(.2);
                        telemetry.addData("far", "good science");
                        this.nextState();


                    }

                }
                break;
            case 3:
                if (runtime.time() < 4) {
                    telemetry.addData("waiting", "for the world to change");
                } else {
                    this.nextState();
                }
                break;

            case 4:
                if (leftPivot.getPosition() > .01) {
                    leftJewelArm.setPosition(.7);
                    leftPivot.setPosition(.68);
                    this.nextState();
                }
                break;
            case 5:
                this.encoderDrive(.25, -1780, 0);
                if (rearLeftMotor.getCurrentPosition() > 1780 ||
                        rearRightMotor.getCurrentPosition() > 1780 ||
                        frontRightMotor.getCurrentPosition() > 1780 ||
                        frontLeftMotor.getCurrentPosition() > 1780) {


                    this.resetEncoder();
                    this.nextState();

                }
                break;
            case 6:

                if (Center) {
                    this.nextState();
                }
                if (Left) {
                    this.encoderDrive(.25, -7, 0);
                    if (rearLeftMotor.getCurrentPosition() < -600 ||
                            rearRightMotor.getCurrentPosition() < -600 ||
                            frontRightMotor.getCurrentPosition() < -600 ||
                            frontLeftMotor.getCurrentPosition() < -600) {


                        this.resetEncoder();
                        this.nextState();

                    }
                }

                if (Right) {
                    this.encoderDrive(.25, 7, 0);
                    if (rearLeftMotor.getCurrentPosition() > 600 ||
                            rearRightMotor.getCurrentPosition() > 600 ||
                            frontRightMotor.getCurrentPosition() > 600 ||
                            frontLeftMotor.getCurrentPosition() > 600) {

                        this.resetEncoder();
                        this.nextState();
                    }
                }

                break;
            case 7:
                if (this.turn(-88)) {
                    telemetry.addData("we made", "it niggers");
                    this.nextState();

                }
                break;
            case 8:
                this.encoderDrive(.25, 7, -88);
                if (rearLeftMotor.getCurrentPosition() > 530 ||
                        rearRightMotor.getCurrentPosition() > 530 ||
                        frontRightMotor.getCurrentPosition() > 530 ||
                        frontLeftMotor.getCurrentPosition() > 530) {

                    this.resetEncoder();
                    this.nextState();


                }
                break;


            case 9:
                this.encoderDrive(.25, -7, -88);
                if (rearLeftMotor.getCurrentPosition() > -530 ||
                        rearRightMotor.getCurrentPosition() > -530 ||
                        frontRightMotor.getCurrentPosition() > -530 ||
                        frontLeftMotor.getCurrentPosition() > -530) {

                    this.resetEncoder();
                }
        }
    }


    public boolean resetDelay()   {
        if (leftPivot.getPosition() > .01) {

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return true;
        }
        return false;
    }

    public void nextState() {
        previousState = state;
        state = -1;

        telemetry.addData("Weird", "Science");

        this.resetEncoder();

        return;


    }

    public void encoderDrive( double speed, double distance, int angleTarget) {
        int Counts = (int) (COUNTS_PER_INCH * distance);

        frontRightMotor.setTargetPosition(Counts);
        frontLeftMotor.setTargetPosition(Counts);
        rearLeftMotor.setTargetPosition(Counts);
        rearRightMotor.setTargetPosition(Counts);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int angleCurrent;
        int angleError;
        double drivePower;

        angleCurrent = gyro.getIntegratedZValue();

        angleError = angleTarget - angleCurrent;

        if ((Math.abs(angleError) > HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C) + (iError * I_C);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(drivePower + speed);
            frontLeftMotor.setPower(drivePower);
            rearRightMotor.setPower(drivePower + speed);
            rearLeftMotor.setPower(drivePower);
        }
        if ((Math.abs(angleError) < HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C) + (iError * I_C);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(drivePower);
            frontLeftMotor.setPower(drivePower + speed);
            rearRightMotor.setPower(drivePower);
            rearLeftMotor.setPower(drivePower + speed);
        } else {
            frontRightMotor.setPower(speed);
            frontLeftMotor.setPower(speed);
            rearRightMotor.setPower(speed);
            rearLeftMotor.setPower(speed);
        }



    }
    public boolean turn(int angleTarget) {
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int angleCurrent;
        int angleError;
        double drivePower;

        angleCurrent = gyro.getIntegratedZValue();

        angleError = angleTarget - angleCurrent;

        if ((Math.abs(angleError) > HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C) + (iError * I_C);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(-drivePower);
            frontLeftMotor.setPower(drivePower);
            rearRightMotor.setPower(-drivePower);
            rearLeftMotor.setPower(drivePower);

            telemetry.addData("Target Angle", angleTarget);
            telemetry.addData("Current Angle", angleCurrent);
            telemetry.addData("Angle Error", angleError);
            telemetry.addData("Drive Power", drivePower);
           // telemetry.addData("Raw", odsSensor.getRawLightDetected());
           // telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.update();

            return false;
        } else {
            iError = 0;
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
            rearLeftMotor.setPower(0);

            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            return true;
        }
    }

    public double getMaxVelocity(double currentMax, double wheelVelocity){
        if (Math.abs(currentMax) > Math.abs(wheelVelocity)){
            return Math.abs(currentMax);
        } else{
            return Math.abs(wheelVelocity);
        }
    }
    public void resetEncoder(){
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



}




















