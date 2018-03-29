package org.firstinspires.ftc.teamcode.Auton;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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
@Autonomous(name="Suck Test w/Movement",group="Auton")
@Disabled
public class Suck_Test extends OpMode {

    //* Everything before next comment is defining variables to be accessed
    //* for the the rest of the time

    Hardware bot =  new Hardware();

    static final int HEADING_THRESHOLD = 1;

    static final double P_C = 0.01;
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

    DcMotor Lift;
    DcMotor Extension;
    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo leftPivot;
    Servo relicClamp;
    Servo relicArm;
    Servo leftJewelArm;
    Servo flopperClamp;
    Servo flopperDick;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    ColorSensor blueColorSensor;

    DistanceSensor blueDistanceSensor;

    OpticalDistanceSensor odsSensor;


    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    int redCount = 0;
    int blueCount = 0;
    String measuredColor = "None";

    int encoderCount;

    int i = 0;
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


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled      = true;
        gParameters.loggingTag          = "IMU";
        gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

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

        leftPivot.setPosition(.68);
        leftJewelArm.setPosition(.95);
        relicClamp.setPosition(1);
        relicArm.setPosition(0);
        flopperDick.setPosition(.45);





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
        telemetry.addData("fL Encoder: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData("rR Encoder: ", rearRightMotor.getCurrentPosition());
        telemetry.addData("fR Encoder: ", frontRightMotor.getCurrentPosition());
        telemetry.addData("rL Encoder: ", rearLeftMotor.getCurrentPosition());

    }

    @Override
    public void loop() {


        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (blueColorSensor.red() * SCALE_FACTOR),
                (int) (blueColorSensor.green() * SCALE_FACTOR),
                (int) (blueColorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        //telemetry.addData("fL", frontLeftMotor.getPower());
        //telemetry.addData("rR", rearRightMotor.getPower());
        //telemetry.addData("fL D", frontLeftMotor.getCurrentPosition());
        //telemetry.addData("rR D", rearRightMotor.getCurrentPosition());
        //telemetry.addData("fR D", frontRightMotor.getCurrentPosition());
        //telemetry.addData("rL D", rearLeftMotor.getCurrentPosition());



        // telemetry.addData("time", runtime.time());
        // telemetry.addData("Red  ", blueColorSensor.red());
        // telemetry.addData("Blue ", blueColorSensor.blue());

        // telemetry.addData("Case", state);
        // telemetry.addData("gyro", gyro.getIntegratedZValue());


        // telemetry.addData("Hue", hsvValues[0]);

        // telemetry.addData("Raw", odsSensor.getRawLightDetected());
        //telemetry.addData("Normal", odsSensor.getLightDetected());

        telemetry.update();

        switch (state) {
            case -1:
                if (this.resetDelay()) {
                    state = previousState + 1;
                }
                break;


            // Drive forward
            case 0:

                if (runtime.time() < .5) {
                    rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightIntake.setPower(1);
                    leftIntake.setPower(1);
                } else if (runtime.time() < 1.5) {
                    frontRightMotor.setPower(.7);
                    frontLeftMotor.setPower(.7);
                    rearRightMotor.setPower(.7);
                    rearLeftMotor.setPower(.7);
                } else if (runtime.time() < 1.75) {
                    frontRightMotor.setPower(-.7);
                    frontLeftMotor.setPower(.7);
                    rearRightMotor.setPower(-.7);
                    rearLeftMotor.setPower(.7);
                } else if (runtime.time() < 2) {
                    frontRightMotor.setPower(.7);
                    frontLeftMotor.setPower(-.7);
                    rearRightMotor.setPower(.7);
                    rearLeftMotor.setPower(-.7);
                } else if (runtime.time() < 2.5) {
                    frontRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    rearRightMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                } else {
                    if (i == 1) {
                        rightIntake.setPower(0);
                        leftIntake.setPower(0);
                        i = 0;
                        this.nextState();
                    } else{
                        i++;
                        this.goToState(0);
                    }

                }
                break;


            default:
                break;
        }
    }

    public boolean resetDelay()   {
        if (leftPivot.getPosition() > .01) {

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            runtime.reset();

            return true;
        }
        return false;
    }

    public void nextState() {
        previousState = state;
        state = -1;

        telemetry.addData("Weird", "Science");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        float angleCurrent;
        float angleError;
        double drivePower;

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        angleError = angleTarget - angleCurrent;

        if ((Math.abs(angleError) > HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C) + (iError * I_C);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(drivePower + speed);
            frontLeftMotor.setPower(.15);
            rearRightMotor.setPower(drivePower + speed);
            rearLeftMotor.setPower(.15);
        }
        if ((Math.abs(angleError) < HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C) + (iError * I_C);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(.15);
            frontLeftMotor.setPower(drivePower + speed);
            rearRightMotor.setPower(.15);
            rearLeftMotor.setPower(drivePower + speed);
        } else {
            frontRightMotor.setPower(speed);
            frontLeftMotor.setPower(speed);
            rearRightMotor.setPower(speed);
            rearLeftMotor.setPower(speed);
        }



    }

    /**
     * Move the bot a user defined amount in inches. To move backwards the speed must be reversed.
     */
    public boolean PIDdrive(double distance, double speed) {

        int counts = (int) (COUNTS_PER_INCH * distance);

        frontRightMotor.setTargetPosition(counts);
        frontLeftMotor.setTargetPosition(counts);
        rearLeftMotor.setTargetPosition(counts);
        rearRightMotor.setTargetPosition(counts);


        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);

        if (!frontLeftMotor.isBusy() ||
                !frontRightMotor.isBusy() ||
                !rearLeftMotor.isBusy()  ||
                !rearRightMotor.isBusy())  {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);

            this.resetEncoder();

            return true;
        }

        telemetry.addData("Encoder Target: ", counts);
        telemetry.addData("FL Current Encoder: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData("FR Current Encoder: ", frontRightMotor.getCurrentPosition());
        telemetry.addData("RL Current Encoder: ", rearLeftMotor.getCurrentPosition());
        telemetry.addData("RR Current Encoder: ", rearRightMotor.getCurrentPosition());
        telemetry.addData("FL Speed: ", frontLeftMotor.getPower());
        telemetry.addData("FR Speed: ", frontRightMotor.getPower());
        telemetry.addData("RL Speed: ", rearLeftMotor.getPower());
        telemetry.addData("RR Speed: ", rearRightMotor.getPower());
        telemetry.update();

        return false;

    }

    public boolean turn(int angleTarget) {
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float angleCurrent;
        float angleError;
        double drivePower;

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        angleError = angleTarget - angleCurrent;

        if ((Math.abs(angleError) > HEADING_THRESHOLD)) {
            iError += angleError;

            drivePower = (angleError * P_C) + (iError * I_C);

            drivePower = Range.clip(drivePower, -1.0, 1.0);

            frontRightMotor.setPower(drivePower);
            frontLeftMotor.setPower(-drivePower);
            rearRightMotor.setPower(drivePower);
            rearLeftMotor.setPower(-drivePower);

            telemetry.addData("Target Angle", angleTarget);
            telemetry.addData("Current Angle", angleCurrent);
            telemetry.addData("Angle Error", angleError);
            telemetry.addData("Drive Power", drivePower);
            //telemetry.addData("Raw", odsSensor.getRawLightDetected());
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
// *********************************************
    //          Navigation Functions
    // *********************************************

    /**
     * Move the bot a user defined amount in inches. To move backwards the speed must be reversed and the distance.
     */
    /*
    public boolean drive(MecanumHardware bot, double distance, double speed) {
        runUsingEncoders();

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);

        int counts = (int) (COUNTS_PER_INCH * distance);

        if (Math.abs(frontLeftMotor.getCurrentPosition()) >= Math.abs(counts) ||
                Math.abs(frontRightMotor.getCurrentPosition()) >= Math.abs(counts) ||
                Math.abs(rearLeftMotor.getCurrentPosition()) >= Math.abs(counts) ||
                Math.abs(rearRightMotor.getCurrentPosition()) >= Math.abs(counts) ||
                getRuntime() > 8) {
            stopDrive();

            resetEncoders();

            return true;
        }

        telemetry.addData("Encoder Target: ", counts);
        telemetry.addData("Current Encoder: ", frontLeftMotor.getCurrentPosition());
        telemetry.update();

        return false;

    }
    */


    /**
     * Drives the robot at a specified angle and speed
     * @param bot The hardware config
     * @param speed The speed at which the bot moves
     * @param theta The angle at which the bot moves. Depending on the orientation of the gyro,
     *              the angle measured starts from the front of the robot and rotation clockwise or
     *              counterclockwise results in a positive or negative angle. Angles are measured in radians.
     */
    /*
    public void angleDrive(MecanumHardware bot, double speed, double theta){
        //Declare and/or intialize variables for the mecanum wheel velocity equations
        double raw_v_1, raw_v_2, raw_v_3, raw_v_4;
        double scaled_v_1, scaled_v_2, scaled_v_3, scaled_v_4;

        double max_v = 1;

        //Declare and/or initialize variables for maintaining the robot's current facing
        int angleCurrent;

        //Declare and/or initialize variables for the PID control loop that corrects the robot's rotation
        int pError;
        int dError;
        double pOut;
        double dOut;
        double correctionOutput;

        angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

        //Uses a PID control loop in order to make sure the heading of the bot doesn't change
        //Calculate the PID errors and the appropriate power levels
        pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        dError = pError - previousError;

        pOut = pError * P_COEFF_ANGLE_DRIVE;
        dOut = dError * D_COEFF_ANGLE_DRIVE;

        correctionOutput = (pOut +dOut);

        previousError = pError;

        //Calculate the raw motor output
        raw_v_1 = speed*Math.sin(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_1);
        raw_v_2 = speed*Math.cos(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_2);
        raw_v_3 = speed*Math.cos(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_3);
        raw_v_4 = speed*Math.sin(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_4);

        //If the raw motor outputs exceed the [-1, 1], the outputs are scaled
        if (max_v > 1.0){
            scaled_v_1 = raw_v_1/max_v;
            scaled_v_2 = raw_v_2/max_v;
            scaled_v_3 = raw_v_3/max_v;
            scaled_v_4 = raw_v_4/max_v;
        }
        else{
            scaled_v_1 = raw_v_1;
            scaled_v_2 = raw_v_2;
            scaled_v_3 = raw_v_3;
            scaled_v_4 = raw_v_4;
        }

        //Set the robot to run at the calculate power
        frontLeftMotor.setPower(scaled_v_1);
        frontRightMotor.setPower(scaled_v_2);
        rearLeftMotor.setPower(scaled_v_3);
        rearRightMotor.setPower(scaled_v_4);


        return;
    }
    */

    /**
     * Get the maximum velocity of the four wheel in order to scale everything else by it
     */
    /*
    public double getMaxVelocity(double currentMax, double wheelVelocity){

        if (Math.abs(currentMax) > Math.abs(wheelVelocity)){
            return Math.abs(currentMax);
        } else{
            return Math.abs(wheelVelocity);
        }
    }
    */
    /**
     * Corrects the robot's orientation based upon its gyro reading to match the starting/fixed heading
     * @return Whether the robot has corrected its orientation to the starting/fixed heading
     */
    /*
    public boolean correctPosition(int position){
        runUsingEncoders();

        int angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

        //Declare and/or initialize variables for the PID control loop that corrects the robot's rotation
        int pError;
        int dError;
        double pOut;
        double dOut;
        double correctionOutput;

        if(!(position == angleCurrent) && getRuntime() < 1.25){
            //telemetry.addData(">", "Correcting Position");
            //telemetry.update();
            angleCurrent = gyro.getIntegratedZValue();      //Gets the current angle

            //Uses a PID control loop in order to make sure the heading of the bot doesn't change
            //Calculate the PID errors and the appropriate power levels
            pError = position - angleCurrent;        //Finds the current difference between the target and current
            dError = pError - previousError;

            pOut = pError * P_COEFF_ANGLE_DRIVE;
            dOut = dError * D_COEFF_ANGLE_DRIVE;

            correctionOutput = pOut + dOut;

            previousError = pError;


            //Set the robot to run at the calculate power
            frontLeftMotor.setPower(correctionOutput);
            frontRightMotor.setPower(-correctionOutput);
            rearLeftMotor.setPower(correctionOutput);
            rearRightMotor.setPower(-correctionOutput);

            return false;

        }
        else{
            stopDrive();
            return true;
        }
    }
    */
    /**
     *  Determines if bot is aligned with the beacon by seeing if the optical distance sensor is over the
     *  line
     * @return If the optical distance sensor reads a threshold value, indicating the bot is over the line
     */
    /*
    public boolean lineDetected(){
        telemetry.addData("Light Level: ", lineSensor.getLightDetected());

        if (lineSensor.getLightDetected() > .4){
            return true;
        }
        return false;
    }
    */

    // *********************************************
    //          Beacon Pressing Functions
    // *********************************************

    /**
     *  Sense the color of the beacon
     */

    public boolean getColor(ColorSensor RGBSensor) {
        //For half a second, the color sensor blue and red values are compared.
        //If the reading had more blue an increment is added to blueCount; the same is done for red
        //
        //After  the blueCount and redCount are compared to determine the measured color

        //if (getRuntime() < .5){
        //   return false;
        if (getRuntime() < .5) {

            if (RGBSensor.red() > RGBSensor.blue()) {
                redCount++;
            } else {
                blueCount++;
            }

            return  false;
        } else {
            if (redCount > blueCount) {
                measuredColor = "Red";
            } else {
                measuredColor = "Blue";
            }
            telemetry.addData("Blue Count: ", blueCount);
            telemetry.addData("Red Count: ", redCount);
            telemetry.addData("Measured Color:", measuredColor);
            telemetry.update();
            return true;
        }

    }


    /**
     * Actuate the given servo in order to press the button
     * @param pressor
     * @return Whether the full action has been completed
     */
    /*
    public boolean pushButton(CRServo pressor) {

        if (getRuntime() < 2) {
            if (rangeSensor.getUltrasonicLevel() > 7){
                pressor.setPower(1);
                angleDrive(robot,.175, strafeInward);
                return false;
            } else {
                pressor.setPower(1);
                return false;
            }
        } else if (getRuntime() >= 2 && getRuntime() < 4) {
            pressor.setPower(0);
            rostopDrive();
            return false;
        } else{
            pressor.setPower(.5);
            resetStartTime();
            return true;
        }
    }
    */
    /**
     * Swap the direction of the given servo
     * @param servo The servo that is inverting direction
     */
    /*
    public void invertServo(CRServo servo) {
        if (servo.getDirection() == CRServo.Direction.FORWARD){
            servo.setDirection(CRServo.Direction.REVERSE);
        } else {
            servo.setDirection(CRServo.Direction.FORWARD);
        }
    }
    */
    // *********************************************
    //          State Machine Functions
    // *********************************************



    /**
     * Preps the state machine to change to the designated state
     * @param dstState The next state
     */

    public void goToState(int dstState){
        previousState = dstState - 1;
        state = -1;
        runtime.reset();
        return;
    }


    /**
     * Function that is called to end the state machine
     */
    public void stopStateMachine() {
        state = 7261;
        return;
    }
}






















