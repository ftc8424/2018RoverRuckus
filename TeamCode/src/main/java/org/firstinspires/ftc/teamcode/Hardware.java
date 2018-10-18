package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by FTC8424 on 10/13/2016.  Updated 8/14/2018.
 *
 * This is the initial helper class for the hardware components of Cyber Eagles (FTC8424) robot.
 * It is NOT an OpMode or any of the others, it's a helper class that has the hardware
 * map and some of the helper methods that are needed for all OpModes.
 *
 * Starting in the 2018-19 season, this is no longer the HardwareHepler class, but the Hardware
 * base class which supports a two-wheel robot and two IMU's (REV electronics).
 * All other components of a robot will extend this class to define the characteristics of that
 * particular robot (e.g., a 4-wheel robot).  This will then build a full class hierarchy of
 * Hardware components so if the robot is a 4-motor robot with just a manipulator, then it might
 * be defined as class Manip4Motor extends Hardware4Motor {} with class Hardware4Motor
 * extends Hardware {}.
 *
 * This class relies on the Constants class for pulling the values for the things in the
 * configuration file of the robot controller.
 *
 */

public class Hardware {

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
    /* Wheel ratio values for the encoders (see end of this file for calculations). */
    protected static final int   COUNTS_PER_SECOND_MAX = 2800;  // AndyMark NeveRest 40:1/20:1
    protected static final double COUNTS_PER_MOTOR_REV = 1120;  // AndyMark NeveRest 40:1 CPR
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gears, just motor shafts
    protected static final double WHEEL_DIAMETER_INCHES= 4.0;   // 4" Omni wheels and 4" Stealth

    protected static final double encoderInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
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
    public void robot_init(HardwareMap hw) {
        this.hwMap = hw;
        initIMU();        // Probably just the one in this base class
        initMotor();      // Polymorphism should call one of my sub-classes initMotor().
        initServo();      // Ditto
        initSensor();     // Ditto squared
        runtime.reset();  // Reset the runtime to be now, when we initialized
    }

    /**
     * Initialize the IMU(s) from the system.
     */
    public void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, Constants.IMU);
        //imu2 = hwMap.get(BNO055IMU.class, Constants.IMU2);
        imu.initialize(parameters);
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

        if (revLeft)
            LBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Default to no encoders
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
    private void initServo() {
        // No servos in this base class, but needs to be here to catch super.initServo()
    }

    /**
     * This method is catcher for super.initSensor() calls, but there are no sensors (other than
     * IMU) in this base class, so this call just returns immediately.
     */
    private void initSensor() {
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
     * Get the heading of the IMU specified in a 0-359 format, with 0-180 being left and 181-359 right
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

    public void normalDrive (OpMode caller, double leftBackPower, double rightBackPower) {
        LBack.setPower(leftBackPower);
        RBack.setPower(rightBackPower);
        caller.telemetry.addData("normalDrive:", "Back Power set to L:%.2f, R:%.2f", leftBackPower, rightBackPower);

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

        LBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // if ( robotType == FULLAUTO ) {
        // leftMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightMidDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}



        /*
         * Determine new target position and pass to motor controller
         */
        // if ( robotType == FULLAUTO ) {
        //   newLeftMidTarget = leftMidDrive.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
        // newRightMidTarget = rightMidDrive.getCurrentPosition() + (int) Math.round(rightInches * encoderInch);
        //} else {
        //newLeftMidTarget = 0;
        //newRightMidTarget = 0;
        // }
        newLeftBackTarget = LBack.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightBackTarget = RBack.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
//        caller.telemetry.addLine("encoderDrive-MID:")
//                .addData("Left Tgt POS: ", newLeftMidTarget)
//                .addData("Right Tgt POS:" ,  newRightMidTarget);
//        caller.telemetry.addLine("EncoderDrive-BCK:")
//                .addData("Left Tgt POS: ", newLeftBackTarget)
//                .addData("Right Tgt POS: ", newRightBackTarget);
//        caller.telemetry.update();

        boolean lmEncoderSet = false;
        boolean rmEncoderSet = false;
        boolean lbEncoderSet = false;
        boolean rbEncoderSet = false;

        lbEncoderSet = setEncoderPosition(caller, LBack, newLeftBackTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, RBack, newRightBackTarget, encoderTimeout);
        //  if ( robotType == FULLAUTO ) {
        //    lmEncoderSet = setEncoderPosition(caller, leftMidDrive, newLeftMidTarget, encoderTimeout);
        //     rmEncoderSet = setEncoderPosition(caller, rightMidDrive, newRightMidTarget, encoderTimeout);
        //} else {
        lmEncoderSet = true;
        rmEncoderSet = true;
        //  }
//        caller.telemetry.addLine("EncoderSet:")
//                .addData("LB: ", lbEncoderSet)
//                .addData("RB: ", rbEncoderSet)
//                .addData("LM: ", lmEncoderSet)
//                .addData("RM: ", rmEncoderSet);
//        caller.telemetry.update();
        if ( ! (lmEncoderSet && lbEncoderSet && rmEncoderSet && rbEncoderSet) ) {
            caller.telemetry.addLine("Encoders CANNOT be set, aborting OpMode");
            caller.telemetry.update();
            caller.sleep(10000);    // Can't go any further, allow telemetry to show, then return
            return;
        }

        // reset the timeout time and start motion.

//        caller.telemetry.addLine("Encoder Drive: ")
//                .addData("PowerSet: ", "%.4f", Math.abs(speed));
//        caller.telemetry.update();

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

            //   if(robotType == FULLAUTO){
            //     leftMidDrive.setPower(leftBackPower);
            //   rightMidDrive.setPower(rightBackPower);
            //}
            caller.telemetry.addData("Power:", "Left Power %.2f, Right Power %.2f", leftBackPower, rightBackPower);
            caller.telemetry.update();
            lbCurPos = LBack.getCurrentPosition();
            rbCurPos = RBack.getCurrentPosition();
            //   if ( robotType == FULLAUTO ) {
            //      lmCurPos = leftMidDrive.getCurrentPosition();
            //      rmCurPos = rightMidDrive.getCurrentPosition();
            //  } else {

            //  }
            isBusy = (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPos - newRightBackTarget) >= 5);
            //     if ( robotType == FULLAUTO )
            //        isBusy = isBusy && (Math.abs(lmCurPos - newLeftMidTarget) >= 5) && (Math.abs(rmCurPos - newRightMidTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        LBack.setPower(0);
        RBack.setPower(0);
        // if ( robotType == FULLAUTO ) {
        //    leftMidDrive.setPower(0);
        //    rightMidDrive.setPower(0);
        // }

        // Turn off RUN_TO_POSITION

        LBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  if ( robotType == FULLAUTO ) {
        //      leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //      rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  }
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
 *************************************************************************************************/
