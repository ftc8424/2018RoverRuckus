package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * Created by FTC8424 on 10/13/2016.  Updated 8/14/2018.
 *
 * This is the initial helper class for the hardware components of Cyber Eagles (FTC8424) robot.
 * It is NOT an OpMode or any of the others, it's a helper class that has the hardware
 * map and some of the helper methods that are needed for all OpModes.
 *
 * Starting in the 2018-19 season, this is no longer the HardwareHepler class, but the Base
 * base class which supports a two-wheel robot and two IMU's (REV electronics).
 * All other components of a robot will extend this class to define the characteristics of that
 * particular robot (e.g., a 4-wheel robot).  This will then build a full class hierarchy of
 * Base components so if the robot is a 4-motor robot with just a manipulator, then it might
 * be defined as class Manip4Motor extends Motor4 {} with class Motor4
 * extends Base {}.
 *
 * This class relies on the Constants class for pulling the values for the things in the
 * configuration file of the robot controller.
 *
 */

public class Base {

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;


    /* Keep track of the time, available to me and to sub-classes. */

    protected ElapsedTime runtime = new ElapsedTime();

    /* This is the core of REV Expansion hubs, just the IMU's */
    public BNO055IMU imu = null;
    public BNO055IMU imu2 = null;

    /* These are values that can be used by the opModes (e.g., robot.LBack.setPower(); */
    public DcMotor LBack = null;
    public DcMotor RBack = null;

    /*
     * Protected instance variables (meaning, my sub-classes can see these as well).
     */
    // TODO: Create class of encoder values for all different motor encoders and allow classes to select/override per robot type

    /* Wheel ratio values for the encoders (see end of this file for calculations). */
    protected static final int   COUNTS_PER_SECOND_MAX = 600;  // REV Core Hex
    protected static final double COUNTS_PER_MOTOR_REV = 560;  // REV HD 20:1 CPR
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gears, just motor shafts
    protected static final double WHEEL_DIAMETER_INCHES= 4.0;   // 4" Omni wheels and 4" Stealth and 4" Mecanum

    protected static final double encoderInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751);
    protected static final double lbencoderInch = (152.5 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751);
    protected static final double lfencoderInch = (543 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751);
    protected static final double rfencoderInch = (543 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751);
    protected static final double rbencoderInch = (543 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751);

    /* Other privates for things such as the runtime, the hardware Map, etc. */
    protected HardwareMap hwMap = null;

    /**
     * This is the super-class robot_init() and it will call each initMotor(), initServo() and
     * initSensor() which should exist in the sub-classes, through polymorphism.  Each sub-class
     * that has motors should at least have a call to the right flavor of super.initMotor() to get
     * the back motors to work.  There aren't any servo's in the base class, but there's a super
     * catcher anyway just in case.
     */
    public void robot_init(HardwareMap hw) { robot_init(hw, true); }
    public void robot_init(HardwareMap hw, boolean revLeft) {
        this.hwMap = hw;
        this.initIMU();        // Probably just the one in this base class
        this.initMotor(revLeft);      // Polymorphism should call one of my sub-classes initMotor().

        this.initServo();      // Ditto
        this.initSensor();     // Ditto squared
        runtime.reset();  // Reset the runtime to be now, when we initialized
    }

    /**
     * Initialize the IMU(s) from the system.
     */
    public void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();*/
        imu = hwMap.get(BNO055IMU.class, Constants.IMU);
        //imu2 = hwMap.get(BNO055IMU.class, Constants.IMU2);
        //imu.initialize(parameters);
        //imu2.initialize(parameters);    // Use the same params, note will need different if orientation

    }

    /**
     * Configure and initialize the two back motors, with no reversal.  SUB-CLASS MUST OVERRIDE.
     *
     * Polymorphism will cause sub-classes to call theirs and they must then call super.initMotor()
     * to get me to be called.
     */
    public void initMotor() {
        this.initMotor(false);   // Don't reverse unless told to do that
    }

    /**
     * Configure and Initialize the two back motors, reversing the left side if revLeft is true, SUBCLASS MUST OVERRIDE.
     *
     * Note, polymorphism will cause my sub-classes to call theirs with an override and then
     * they'll call super.initMotor().  If the type of robot is using a normal drive, then the left
     * side motors will be set to REVERSE so that giving them the positive values moves the robot
     * forward.  In those cases, initMotor(true) should be called.
     *
     * If this is a holonomic drive (e.g., Mecanum or omni-wheel in crab-drive format) then you
     * don't want to do that, so pass with false (or just call the version without parameters).
     *
     * @param revLeft    Should I reverse the left motors?
     */
    public void initMotor(boolean revLeft) {
        LBack = hwMap.dcMotor.get(Constants.LBACK);
        RBack = hwMap.dcMotor.get(Constants.RBACK);

        RBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (revLeft)
            LBack.setDirection(DcMotorSimple.Direction.REVERSE);

        else
            LBack.setDirection(DcMotorSimple.Direction.FORWARD);


        //this.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Default to no encoders
        LBack.setPower(0);
        RBack.setPower(0);
    }

    /**
     * Set the Encoder mode for the base class's motors.  This should be overridden in the
     * sub-classes and they should call super.setEncoderMode() if they need to set a value.
     *
     * @param mode
     */
    public void setEncoderMode( DcMotor.RunMode mode ) {
        LBack.setMode(mode);
        RBack.setMode(mode);
    }

    /**
     * This method is catcher for super.initServo() calls, but there are no servos in this
     * base class, so the call just returns immediately.
     */
    public void initServo() {
        // No servos in this base class, but needs to be here to catch super.initServo()
    }

    /**
     * This method is catcher for super.initSensor() calls, but there are no sensors (other than
     * IMU) in this base class, so this call just returns immediately.
     */
    public void initSensor() {
        // No sensors in this base class, but needs to be here to catch super.initSensor();
    }

    /**
     * Get the heading of the IMU in a 0-359 format, with 0-180 being left and 181-369 right of
     * primary orientation.
     *
     * @return    The heading of the robot from base initialization.
     */
    public double getHeading() {
        return getHeading(imu);      // Default to get heading from first IMU
    }

    /**
     * Get the heading of the IMU specified in a 0-359 format, with 0-180 being right and 181-359 left
     * of primary orientation.
     *
     * @param imu    The specific IMU to use (e.g., robot.imu2 or robot.imu)
     * @return    The heading of the robot from base initialization
     */
    public double getHeading(BNO055IMU imu) {

        double retVal = 0;

        if (imu != null) {

            retVal = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (retVal > 0 && retVal < 180) {
                retVal = 360 - retVal;
            } else if (retVal < 0 && retVal > -180) {
                retVal = -1 * retVal;
            }

        }
        return retVal;
    }
    /**
     * The code shuld come from the PushbotAutoDriveByGyro.java file as well as the helper
     * methods such as onHeading, gyroHold, etc., but it needs to take into account the robotType
     * so that it only sends power to the appropriate motors.
     *
     * @param caller
     * @param heading
     * @param timeoutS
     * @return
     * @throws InterruptedException
     */
    public boolean gyroTurn(LinearOpMode caller,
                            double heading,
                            double timeoutS) throws InterruptedException {
        int zValue;
        double gHeading;
        int heading360;
        int absHeading;
        double deltaHeading;
        double deltaAngle;
        double rightPower;
        double leftPower;
        double turnspeed;
        double turnFloor = .25;
        double stopTime = runtime.seconds() + timeoutS;


        do {
            gHeading = getHeading();
            caller.telemetry.addData("gyroTurn:", "gHeading: %.1f, going to %.1f", gHeading, heading);
            caller.telemetry.update();
            /*
             * Turn left if the difference from where we're heading to where we want to head
             * is smaller than -180 or is between 1 and 180.  All else (including the 0 and 180
             * situations) turn right.
             */
            deltaHeading = gHeading - heading;
            deltaAngle = Math.abs(deltaHeading);
            turnspeed = deltaAngle/360;
            if (turnspeed < turnFloor || deltaAngle < 30){
                turnspeed = turnFloor;
            }
            if (31 < deltaAngle && deltaAngle < 75){
                turnspeed = .25;
            }
            if (76 < deltaAngle && deltaAngle < 120){
                turnspeed = .5;
            }
            if (121 < deltaAngle && deltaAngle < 180){
                turnspeed = .75;
            }
            if ( deltaHeading < -180 || (deltaHeading > 0 && deltaHeading < 180) ) {
                leftPower = -turnspeed;
                rightPower = turnspeed;
            } else {
                leftPower = turnspeed;
                rightPower = -turnspeed;
            }
            normalDrive(caller, leftPower, rightPower);
            caller.telemetry.update();
        }
        while (caller.opModeIsActive() && Math.abs(gHeading - heading) > 0.2 && runtime.seconds() < stopTime );

        normalDrive(caller, 0.0, 0.0);
        if ( Math.abs(gHeading - heading) <= .4 )
            return true;
        else
            return false;
    }

    public void normalDrive (LinearOpMode caller, double leftBackPower, double rightBackPower) {
        if ( !caller.opModeIsActive() )
            return;

        LBack.setPower(leftBackPower);
        RBack.setPower(rightBackPower);
        caller.telemetry.addData("Base-normalDrive:", "Back Power set to L:%.2f, R:%.2f", leftBackPower, rightBackPower);

    }

    public boolean setEncoderPosition (LinearOpMode caller, DcMotor m1, int target, long timeOut) {
        m1.setTargetPosition(target);
        int m1Pos = m1.getTargetPosition();
        double stopTime = runtime.milliseconds() + timeOut;
        while ( caller.opModeIsActive() && m1Pos != target && runtime.milliseconds() < stopTime ) {
            m1.setTargetPosition(target);
            m1Pos = m1.getTargetPosition();
        }
        return m1Pos == target;
    }

    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        int newLeftBackTarget;
        int newRightBackTarget;
        //int getHeading = gyro.getIntegratedZValue();
        long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

        if ( !caller.opModeIsActive() )
            return;

        setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean lbEncoderSet = false;
        boolean rbEncoderSet = false;

        newLeftBackTarget = LBack.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
        newRightBackTarget = RBack.getCurrentPosition() + (int) Math.round(rightInches * encoderInch);
        lbEncoderSet = setEncoderPosition(caller, LBack, newLeftBackTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, RBack, newRightBackTarget, encoderTimeout);
        if ( ! (lbEncoderSet && rbEncoderSet) ) {
            caller.telemetry.addLine("Encoders CANNOT be set, aborting OpMode");
            caller.telemetry.update();
            //caller.sleep(10000);    // Can't go any further, allow telemetry to show, then return
            return;
        }

        // keep looping while we are still active, and there is time left, and motors haven't made position.
        boolean isBusy;
        int lbCurPos;
        int rbCurPos;
        double stopTime = runtime.seconds() + timeoutS;
        double leftBackPower;
        double rightBackPower;
        double lastSetTime = runtime.milliseconds();
        int HeadingLoop;

        do {
            leftBackPower = speed;
            rightBackPower = speed;
            if (leftBackPower <= 0.01) {
                lastSetTime = runtime.milliseconds();
                leftBackPower = speed;
                rightBackPower = speed;
            }

            leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
            rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);
            LBack.setPower(leftBackPower);
            RBack.setPower(rightBackPower);

            caller.telemetry.addData("Power:", "Left Power %.2f, Right Power %.2f", leftBackPower, rightBackPower);
            caller.telemetry.update();
            lbCurPos = LBack.getCurrentPosition();
            rbCurPos = RBack.getCurrentPosition();
            isBusy = (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPos - newRightBackTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        LBack.setPower(0);
        RBack.setPower(0);

        // Turn off RUN_TO_POSITION
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.TFOD_MODEL_ASSET, Constants.LABEL_GOLD_MINERAL, Constants.LABEL_SILVER_MINERAL);
    }


}


/************************************************************************************************
 * For encoder math, here is the information from AndyMark's web site, so it will be key in
 * setting up the setMaxSpeed() when in PID mode, as well as when figuring out the counts per
 * inch mode.
 *
 *    NeveRest 60:1 Motors:
 *    ---------------------
 *        7 pulses per revolution of hall effect encoder, and a 60:1 gearbox, so 7*60 ==
 *      420 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *     1680 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *      105 revolutions per minute of output shaft (RPM), so (1680 * 105) / 60 ==
 *     2940 pulses per second is the max Speed setting of the encoders on this motor
 *
 *    NeveRest 40:1 Motors:
 *    ---------------------
 *          7 pulses per revolution of hall effect encoder, and a 40:1 gearbox, so 7*40 ==
 *        280 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *       1120 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        150 revolutions per minute of output shaft (RPM), so (1120 * 150) / 60 ==
 *       2800 pulses per second is the max Speed setting of the encoders on this motor
 *
 *    NeveRest 20:1 Motors:
 *    ----------------------
 *          7 pulses per revolution of hall effect encoder, and a 20:1 gearbox, so 7*20 ==
 *        140 pulses per revolution of the encoder, there are 4 revolutions of encoder to output
 *        560 pulses per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        300 revolutions per minute of output shaft (RPM), so (560 * 300) / 60 ==
 *       2800 pulses per second is the max Speed setting of the encoders on this motor
 *
 * So, for these two motors, the encoder COUNTS_PER_MOTOR_REV above would be 1,120 for the 40:1
 * and 560 for the 20:1, and the COUNTS_PER_SECOND_MAX above would be 2800 for both.
 *
 * Here are the values for the REV Robotics Core Hex and HD Hex gearboxes
 *
 *     REV Core Hex Motors:
 *     --------------------
 *          4 cycles per rotation of encoder shaft and a 72:1 gearbox, so 4*72
 *        288 counts per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        125 revolutions per minute of output shaft (RPM), so (288 * 125) / 60 ==
 *        600 counts per second is the max Speed setting of the encoders on this motor
 *
 *     REV HD Hex Motors (40:1 Gearbox):
 *     ----------------------------------
 *         28 cycles per rotation of encoder shaft and a 40:1 gearbox, so 28*40
 *       1120 counts per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        150 revolutions per minute of output shaft (RPM), so (1120 * 150) / 60 ==
 *       2800 counts per second is the max Speed setting of the encoders on this motor
 *
 *     REV HD Hex Motors (20:1 Gearbox):
 *     ----------------------------------
 *         28 cycles per rotation of encoder shaft and a 20:1 gearbox, so 28*20
 *        560 counts per revolution of the OUTPUT SHAFT (e.g., the motor shaft)
 *        300 revolutions per minute of output shaft (RPM), so (560 * 300) / 60 ==
 *       2800 counts per second is the max Speed setting of the encoders on this motor
 *
 *************************************************************************************************/
