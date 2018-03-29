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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Janker on 10/2/2017.
 */
@Autonomous(name="Full Autonomous Blue Side",group="Auton")
@Disabled
public class Blue_Jankers_Auton_v5 extends OpMode {

    //* Everything before next comment is defining variables to be accessed
    //* for the the rest of the time

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final int HEADING_THRESHOLD = 1;

    static final double P_C = 0.01;
    static final double I_C = 0.00000125;
    static final double D_C = 0;

    int iError = 0;
    int previousError = 0;

    public ElapsedTime runtime = new ElapsedTime();

    int state = 0;
    int previousState = 0;

    int i = 1;

    boolean Center = false;
    boolean Right = false;
    boolean Left = false;
    String currentCol = "Center";

    DcMotor rearLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor rearRightMotor;
    DcMotor frontRightMotor;

    DcMotor Lift;
    DcMotor Extension;
    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo jewelPivot;
    Servo jewelArm;
    Servo relicClamp;
    Servo relicArm;
    Servo glyphClamp;
    Servo glyphDump;
    Servo alignArm;

    BNO055IMU imu;

    Orientation angles;

    ColorSensor blueColorSensor;
    DistanceSensor blueDistanceSensor;
    DistanceSensor aDist;
    String measuredColor = "None";

    DigitalChannel digitalTouch;

    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    static final String        ALLIANCE_COLOR       = "Blue";
    static final String        OPPONENT_COLOR       = "Red";

    // *********************************************
    //          Navigation Settings
    //              - Distance in inches
    //              - Angles in degrees
    //              - Strafe angles in radians
    //
    //           FINE TUNE AUTONOMOUS NAVIGATION HERE
    // *********************************************
    static final double GLYPH_DUMP_DIST                             = -3;               // How far the bot moves forward b4 dumping
    static final double GLYPH_PUSH_DIST                             = -3;            // How far the bot pushes the glyphs forward after dumping
    static final double REVERSE_DIST                                = 6;                // How far the bot reverses after pushing the glyphs in
    static final double FAST_DRIVE_SPEED                            = -.7;
    static final double SLOW_DRIVE_SPEED                            = -.2;
    static final double STRAFE_SPEED                                = .7;
    static final double STRAFE_LEFT                                 = Math.PI/2;
    static final double STRAFE_RIGHT                                = -Math.PI/2;
    static final double STRAFE_DIST_LEFT_CENTER                     = -3.75;                // The distance when strafing from the left to center column
    static final double STRAFE_DIST_LEFT_RIGHT                      = -8;
    static final double STRAFE_DIST_CENTER_LEFT                     = 3.75;
    static final double STRAFE_DIST_CENTER_RIGHT                    = -3.75;
    static final double STRAFE_DIST_RIGHT_CENTER                    = 3.75;
    static final double STRAFE_DIST_RIGHT_LEFT                      = 8;

    // Settings dependent on alliance color
    static final double CENTER_COL_DIST_BLUE                        = 24;               // The dist the bot runs off platform to align w/ center col (Blue)
    static final double RIGHT_COL_DIST_BLUE                         = 24 + 5.75;        
    static final double LEFT_COL_DIST_BLUE                          = 24 - 5.25;
    static final int    ANGLE_OF_BOX_BLUE                           = -88;              // Angle to face the blue cryptobox
    static final double CENTER_COL_DIST_RED                         = -24;
    static final double RIGHT_COL_DIST_RED                          = -(24 - 5.25);
    static final double LEFT_COL_DIST_RED                           = -(24 + 5.75);
    static final int    ANGLE_OF_BOX_RED                           = 88;

    // *********************************************
    //          Servo Positions
    // *********************************************
    static final double GLYPH_CLAMP_LOCK                            = .75;
    static final double GLYPH_CLAMP_UNLOCK                          = .4;
    static final double GLYPH_DUMP_RAISED                           = .9;
    static final double GLYPH_DUMP_LOWERED                          = .4;
    static final double JEWEL_ARM_RAISED                            = .7;
    static final double JEWEL_ARM_LOWERED                           = .18;
    static final double JEWEL_PIVOT_RAISED                          = .68;
    static final double JEWEL_PIVOT_LOWERED                         = .62;
    static final double JEWEL_PIVOT_LEFT                            = .72;               //Position for the jewel arm to hit the left jewel
    static final double JEWEL_PIVOT_RIGHT                           = .48;
    static final double ALIGN_ARM_RAISED                            = 1;
    static final double ALIGN_ARM_DETECT                            = .35;
    static final double ALIGN_ARM_LOWERED                           = .05;

    double centerDist;
    double rightDist;
    double leftDist;
    int boxAngle;
    double strafeDist;
    int encoderCount;
    double strafeDir;
    boolean atWall = false;

    @Override
    public void init() {

    // *********************************************
    //          FInding & Accessing Hardware
    // *********************************************
        rearLeftMotor = hardwareMap.dcMotor.get("rL");
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        rearRightMotor = hardwareMap.dcMotor.get("rR");
        frontRightMotor = hardwareMap.dcMotor.get("fR");

        Lift = hardwareMap.dcMotor.get("L");
        Extension = hardwareMap.dcMotor.get("E");
        leftIntake = hardwareMap.dcMotor.get("lI");
        rightIntake = hardwareMap.dcMotor.get("rI");

        jewelPivot = hardwareMap.servo.get("lP");
        relicArm = hardwareMap.servo.get("rA");
        relicClamp = hardwareMap.servo.get("rC");
        jewelArm = hardwareMap.servo.get("lJA");
        glyphClamp = hardwareMap.servo.get("fC");
        glyphDump = hardwareMap.servo.get("fD");
        alignArm = hardwareMap.servo.get("aA");

        blueColorSensor = hardwareMap.get(ColorSensor.class, "bColorDistance");
        blueDistanceSensor = hardwareMap.get(DistanceSensor.class, "bColorDistance");

        aDist = hardwareMap.get(DistanceSensor.class, "aDist");

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    // *********************************************
    //          Setting Motor Directions and Modes
    // *********************************************
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        Lift.setDirection(DcMotor.Direction.REVERSE);
        Extension.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);

        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // *********************************************
    //          Setting Initial Servo Positions
    // *********************************************
        jewelPivot.setPosition(.68);
        jewelArm.setPosition(.95);
        relicClamp.setPosition(0);
        relicArm.setPosition(.58);
        glyphDump.setPosition(GLYPH_DUMP_LOWERED);
        glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
        alignArm.setPosition(0);

    // *********************************************
    //          IMU Setup
    // *********************************************

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

        // Retrieve and initialize the IMU. The imu is built into the Rev Expansion Hub
        // The imu is accessible through the I2C port 0
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

     // *********************************************
    //          Vuforia Setup
    // *********************************************
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AavXRvD/////AAAAGYbu90CPGkoGh7X1nRUxwjsWz20nEAcBDAXkRzK4SjWSsBKxl/Ixw/qo0zdSsFvZgQFU7omaLYZ6zjjPIXUWGf8C0f/KcZcbmwc2MrPRO6fI0i7rXSdI1pn9thjJOsjQ5imtpAFm2nT5dkorcWt1Blevx5LjpU99u2ysu8g92vHO6JeTD3g7V2r9Zm4Fo6K/bbfvqmywG9oqMJRGAq27hvgXhvwzORdSdO+TtDBXrFgAvDuQozaD9FqeRa2bwFMLZrJM7YCthG9RT9ENYSRdbrHLLUXgk43tOZhK2N4ec9JP0Xqj8HhKs3pAjtvq5f9Q5kPjQwkd6w40wJmXoULHy8B6vgjI7F8XYdPT2QpCTGPO";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();

        // Depending on the alliance color, use the specified navigation settings for the autonomous
        if (ALLIANCE_COLOR ==  "Blue"){
            centerDist = CENTER_COL_DIST_BLUE;
            rightDist = RIGHT_COL_DIST_BLUE;
            leftDist = LEFT_COL_DIST_BLUE;
            boxAngle = ANGLE_OF_BOX_BLUE;
        } else {
            centerDist = CENTER_COL_DIST_RED;
            rightDist = RIGHT_COL_DIST_RED;
            leftDist = LEFT_COL_DIST_RED;
            boxAngle = ANGLE_OF_BOX_RED;
        }


    }


    @Override
    public void init_loop() {
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

    }

    @Override
    public void loop() {

        telemetry.addData("Case:", state);

        switch (state) {

            case -1:
                if (this.resetDelay()) {
                    state = previousState + 1;
                }
                break;

            // ID the pictograph, has a timeout that defaults to center
            case 0:
                relicArm.setPosition(1);
                alignArm.setPosition(ALIGN_ARM_LOWERED);

                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (runtime.time() < 10){
                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        telemetry.addData("center", "true");
                        Center = true;
                        currentCol = "Center";
                        this.nextState();
                    } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                        telemetry.addData("left", "true");
                        Left = true;
                        currentCol = "Left";
                        this.nextState();
                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        telemetry.addData("right", "true");
                        Right = true;
                        currentCol = "Right";
                        this.nextState();
                    }
                } else{
                    Center = true;
                    currentCol = "Center";
                    this.nextState();
                }

                break;

            // Extend the color sensor, read the color of the jewel, and hit the jewel
            case 1:

                if (runtime.time() < .5) {
                    jewelPivot.setPosition(JEWEL_PIVOT_LOWERED);
                    jewelArm.setPosition(JEWEL_ARM_LOWERED);
                    telemetry.addData("waiting", "for extension");
                    telemetry.addData("Red Values: ", blueColorSensor.red());
                    telemetry.addData("Blue Values: ", blueColorSensor.blue());

                } else if (runtime.time() < 1){
                    if (blueColorSensor.red() > blueColorSensor.blue()){
                        measuredColor = "Red";
                    } else{
                        measuredColor = "Blue";
                    }
                } else if (runtime.time() < 1.25) {
                    telemetry.addData("Measured Color: ", measuredColor);
                    if (measuredColor.equals(OPPONENT_COLOR)) {
                        jewelPivot.setPosition(JEWEL_PIVOT_LEFT);
                        telemetry.addData("Left Jewel: ", OPPONENT_COLOR);

                    } else {
                        jewelPivot.setPosition(JEWEL_PIVOT_RIGHT);
                        telemetry.addData("Left Jewel: ", ALLIANCE_COLOR);
                    }
                } else {
                    this.nextState();

                }
                break;

            // Drive forward to align with the pictograph column and retract the color sensor
            case 2:
                double distance;

                jewelArm.setPosition(JEWEL_ARM_RAISED);
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED);

                if (Center) {
                    distance = centerDist;
                }
                else if (Left) {
                    distance = leftDist;
                }
                else  {
                   distance = rightDist;
                }

                if (ALLIANCE_COLOR.equals("Red")){
                    if (this.PIDdrive(distance, -FAST_DRIVE_SPEED)) {
                        this.resetEncoder();
                        this.nextState();
                    }
                } else{
                    if (this.PIDdrive(distance, FAST_DRIVE_SPEED)) {
                        this.resetEncoder();
                        this.nextState();
                    }
                }
                
                break;


            // Turn to face the box
            case 3:
                if (this.turn(boxAngle)) {
                    this.resetEncoder();
                    this.nextState();
                }
                break;

            case 4:
                if (this.PIDdrive(7, -FAST_DRIVE_SPEED)) {
                    this.nextState();
                }
                break;

            // Reverse to pickup the second glyph
            case 5:

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
                    frontLeftMotor.setPower(-.7);
                    rearRightMotor.setPower(-.7);
                    rearLeftMotor.setPower(-.7);
                } else if (runtime.time() < 2.75) {
                    frontRightMotor.setPower(.1);
                    frontLeftMotor.setPower(.1);
                    rearRightMotor.setPower(.1);
                    rearLeftMotor.setPower(.1);
                } else {
                    if (i == 1) {
                        frontRightMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        rearRightMotor.setPower(0);
                        rearLeftMotor.setPower(0);
                        i = 0;
                        this.nextState();
                    } else{
                        i++;
                        this.goToState(5);
                    }

                }

                //telemetry.addData("fL Encoder: ", frontLeftMotor.getCurrentPosition());
                //telemetry.addData("rR Encoder: ", rearRightMotor.getCurrentPosition());
                //telemetry.addData("fR Encoder: ", frontRightMotor.getCurrentPosition());
                //telemetry.addData("rL Encoder: ", rearLeftMotor.getCurrentPosition());
                break;

            case 6:

                // Turn to face the box
                if (this.turn(boxAngle)) {
                    this.resetEncoder();
                    this.nextState();
                }
                break;

            // Push the glyphs in
            case 7:
                if (this.PIDdrive(-6, FAST_DRIVE_SPEED)) {
                    this.nextState();
                }
                break;



            // Drive forward to the box and align itself with the column
            case 8:
                if(!atWall) {
                    rightIntake.setPower(-1);
                    leftIntake.setPower(-1);
                    glyphClamp.setPosition(GLYPH_CLAMP_LOCK);
                    alignArm.setPosition(ALIGN_ARM_DETECT);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (digitalTouch.getState() == true) {
                        frontLeftMotor.setPower(-.2);
                        frontRightMotor.setPower(-.2);
                        rearLeftMotor.setPower(-.2);
                        rearRightMotor.setPower(-.2);
                    } else {
                        rightIntake.setPower(0);
                        leftIntake.setPower(0);
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        rearLeftMotor.setPower(0);
                        rearRightMotor.setPower(0);
                        alignArm.setPosition(.45);
                        atWall = true;
                    }
                } else {
                    if(aDist.getDistance(DistanceUnit.CM) > 16){
                        angleDrive(.1, STRAFE_LEFT, boxAngle);
                        telemetry.addData("Strafing: ", "Left");
                    } else if (aDist.getDistance(DistanceUnit.CM) < 7.5){
                        angleDrive(.1, STRAFE_RIGHT, boxAngle);
                        telemetry.addData("Strafing: ", "Right");
                    } else{
                        atWall = false;
                        this.nextState();
                    }
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", aDist.getDistance(DistanceUnit.CM)));
                }
               break;



            // Dump the glyph
            case 9:
                if (runtime.time() < .5) {
                    alignArm.setPosition(ALIGN_ARM_RAISED);
                    glyphDump.setPosition(GLYPH_DUMP_RAISED);
                } else {
                    glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
                    this.nextState();
                }
                break;

            // Push the glyphs in
            case 10:
                if (this.PIDdrive(GLYPH_PUSH_DIST, SLOW_DRIVE_SPEED)) {
                    boxAngle = (int) angles.firstAngle;
                    this.nextState();
                }
                break;

            // Reverse from the box and reset glyph system
            case 11:
                if (this.PIDdrive(REVERSE_DIST, -FAST_DRIVE_SPEED)) {
                    this.nextState();
                }
                break;

            // Decide which column to strafe to next
            case 12:

                alignArm.setPosition(ALIGN_ARM_LOWERED);
                glyphClamp.setPosition(GLYPH_CLAMP_UNLOCK);
                glyphDump.setPosition(GLYPH_DUMP_LOWERED);

                if(currentCol.equals("Center")){
                    if (!Left){
                        strafeDist = STRAFE_DIST_CENTER_LEFT;
                        strafeDir = STRAFE_LEFT;
                        currentCol = "Left";
                        Left = true;
                        this.nextState();
                    } else if (!Right){
                        strafeDist = STRAFE_DIST_CENTER_RIGHT;
                        strafeDir = STRAFE_RIGHT;
                        Right = true;
                        currentCol = "Right";
                        this.nextState();
                    } else{
                        this.stopStateMachine();
                    }

                } else if (currentCol.equals("Left")){
                    if (!Center){
                        strafeDist = STRAFE_DIST_LEFT_CENTER;
                        strafeDir = STRAFE_RIGHT;
                        currentCol = "Center";
                        Center = true;
                        this.nextState();
                    } else if (!Right){
                        strafeDist = STRAFE_DIST_LEFT_RIGHT;
                        strafeDir = STRAFE_RIGHT;
                        currentCol = "Right";
                        Right = true;
                        this.nextState();
                    } else{
                        this.stopStateMachine();
                    }

                } else{
                    if (!Center){
                        strafeDist = STRAFE_DIST_RIGHT_CENTER;
                        strafeDir = STRAFE_LEFT;
                        currentCol = "Center";
                        Left = true;
                        this.nextState();
                    } else if (!Left){
                        strafeDist = STRAFE_DIST_RIGHT_LEFT;
                        strafeDir = STRAFE_LEFT;
                        currentCol = "Left";
                        Left = true;
                        this.nextState();
                    }  else{
                        this.stopStateMachine();
                    }

                }
                break;

            // Strafe to the desired column
            case 13:
                if (angleDrive(strafeDist, STRAFE_SPEED, strafeDir, boxAngle)) {
                    this.goToState(4);
                }
                break;

            case 7261:
                jewelPivot.setPosition(JEWEL_PIVOT_RAISED);
                jewelArm.setPosition(JEWEL_ARM_RAISED);
                relicClamp.setPosition(1);
                relicArm.setPosition(0);
                requestOpModeStop();
                break;

            default:
                break;
        }
    }


    // *********************************************
    //          Navigation Functions
    // *********************************************

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

        //telemetry.addData("Encoder Target: ", counts);
        //telemetry.addData("FL Current Encoder: ", frontLeftMotor.getCurrentPosition());
        //telemetry.addData("FR Current Encoder: ", frontRightMotor.getCurrentPosition());
        //telemetry.addData("RL Current Encoder: ", rearLeftMotor.getCurrentPosition());
        //telemetry.addData("RR Current Encoder: ", rearRightMotor.getCurrentPosition());
        //telemetry.addData("FL Speed: ", frontLeftMotor.getPower());
        //telemetry.addData("FR Speed: ", frontRightMotor.getPower());
        //telemetry.addData("RL Speed: ", rearLeftMotor.getPower());
        //telemetry.addData("RR Speed: ", rearRightMotor.getPower());
        //telemetry.update();

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

    public void resetEncoder(){
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     * Drives the robot at a specified angle and speed
     * @param speed The speed at which the bot moves
     * @param theta The angle at which the bot moves. Depending on the orientation of the gyro,
     *              the angle measured starts from the front of the robot and rotation clockwise or
     *              counterclockwise results in a positive or negative angle. Angles are measured in radians.
     */

     public boolean angleDrive(double distance, double speed, double theta, double fixedHeading){
        //Declare and/or intialize variables for the mecanum wheel velocity equations
        double raw_v_1, raw_v_2, raw_v_3, raw_v_4;
        double scaled_v_1, scaled_v_2, scaled_v_3, scaled_v_4;

        double max_v = 1;

        //Declare and/or initialize variables for maintaining the robot's current facing
        double angleCurrent;
        double angleError;

        //Declare and/or initialize variables for the PID control loop that corrects the robot's rotation
        double pError;
        double correctionOutput;

        // Get the angle reading from the imu
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        angleError = fixedHeading - angleCurrent;

        //Uses a PI control loop in order to make sure the heading of the bot doesn't change
        //Calculate the PI errors and the appropriate power levels
        pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iError = iError + previousError;

        correctionOutput = (pError * P_C) + (iError *I_C);


        //Calculate the raw motor output
        raw_v_1 = speed*Math.sin(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_1);
        raw_v_2 = speed*Math.cos(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_2);
        raw_v_3 = speed*Math.cos(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_3);
        raw_v_4 = speed*Math.sin(theta + (Math.PI/4)) + correctionOutput;
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

        // Calculates the encoder counts
        // Since the mecanum strafes the counts have to be scaled by a power of radical two
        int counts = (int) (COUNTS_PER_INCH * distance* 1.414);

        frontLeftMotor.setTargetPosition(counts);
        frontRightMotor.setTargetPosition(-counts);
        rearLeftMotor.setTargetPosition(-counts);
        rearRightMotor.setTargetPosition(counts);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(scaled_v_1);
        frontRightMotor.setPower(scaled_v_2);
        rearLeftMotor.setPower(scaled_v_3);
        rearRightMotor.setPower(scaled_v_4);

        if (!frontLeftMotor.isBusy() ||
                !frontRightMotor.isBusy() ||
                !rearLeftMotor.isBusy()  ||
                !rearRightMotor.isBusy())  {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);

            return true;
        }


        return false;
    }

    /**
     * Drives the robot at a specified angle and speed
     * @param speed The speed at which the bot moves
     * @param theta The angle at which the bot moves. Depending on the orientation of the gyro,
     *              the angle measured starts from the front of the robot and rotation clockwise or
     *              counterclockwise results in a positive or negative angle. Angles are measured in radians.
     */
    public void angleDrive(double speed, double theta, double fixedHeading){
        //Declare and/or intialize variables for the mecanum wheel velocity equations
        double raw_v_1, raw_v_2, raw_v_3, raw_v_4;
        double scaled_v_1, scaled_v_2, scaled_v_3, scaled_v_4;

        double max_v = 1;

        //Declare and/or initialize variables for maintaining the robot's current facing
        double angleCurrent;
        double angleError;

        //Declare and/or initialize variables for the PID control loop that corrects the robot's rotation
        double pError;
        double correctionOutput;

        // Get the angle reading from the imu
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        angleCurrent = angles.firstAngle;

        angleError = fixedHeading - angleCurrent;

        //Uses a PI control loop in order to make sure the heading of the bot doesn't change
        //Calculate the PI errors and the appropriate power levels
        pError = fixedHeading - angleCurrent;        //Finds the current difference between the target and current
        iError = iError + previousError;

        correctionOutput = (pError * P_C) + (iError *I_C);


        //Calculate the raw motor output
        raw_v_1 = speed*Math.sin(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_1);
        raw_v_2 = speed*Math.cos(theta + (Math.PI/4)) + correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_2);
        raw_v_3 = speed*Math.cos(theta + (Math.PI/4)) - correctionOutput;
        max_v = this.getMaxVelocity(max_v, raw_v_3);
        raw_v_4 = speed*Math.sin(theta + (Math.PI/4)) + correctionOutput;
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



    /**
     * Get the maximum velocity of the four wheel in order to scale everything else by it
     */

    public double getMaxVelocity(double currentMax, double wheelVelocity){

        if (Math.abs(currentMax) > Math.abs(wheelVelocity)){
            return Math.abs(currentMax);
        } else{
            return Math.abs(wheelVelocity);
        }
    }

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
    //          State Machine Functions
    // *********************************************


    /**
     *  Check if the encoders are reset and adds in a small delay to account for hardware cycles
     *  Sets the motors to run using encoders
     */
    public boolean resetDelay(){
        if (frontLeftMotor.getCurrentPosition() == 0||
                frontRightMotor.getCurrentPosition() == 0 ||
                rearLeftMotor.getCurrentPosition() == 0  ||
                rearRightMotor.getCurrentPosition() == 0 || runtime.time() > 2){

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            previousError = 0;
            iError = 0;
            runtime.reset();

            return true;
        }
        return false;
    }

    /**
     * Increments into the next state but first loops to the delay function
     * The current state is bookmarked in order to move to the next state after
     * calling the delay.
     */
    public void nextState() {
        previousState = state;
        state = -1;

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return;

    }


    /**
     * Preps the state machine to change to the designated state
     * @param dstState The next state
     */
    public void goToState(int dstState){
        previousState = dstState - 1;
        state = -1;
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
