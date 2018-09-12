package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by FTC8424 on 10/13/2016.  Updated 8/14/2018.
 *
 * This is the initial helper class for the hardware components of Cyber Eagles (FTC8424) robot.
 * It is NOT an OpMode or any of the others, it's a helper class that has the hardware
 * map and some of the helper methods that are needed for all OpModes.
 *
 * Starting in the 2018-19 season, this is no longer the HardwareHepler class, but the Hardware
 * base class which supports a two-wheel robot.  All other components of a robot will extend this
 * class to define the characteristics of that particular robot (e.g., a 4-wheel robot).  This
 * will then build a full class hierarchy of Hardware components so if the robot is a 4-motor robot
 * with just a manipulator, then it might be defined as class Manip4Motor extends Hardware4Motor {}
 * with class Hardware4Motor extends Hardware {}.
 *
 * This class relies on the Constants class for pulling the values for the things in the
 * configuration file of the robot controller.
 *
 */

public class Hardware {

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
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751); //YEEET first 50 digits

    /* Other privates for things such as the runtime, the hardware Map, etc. */
    private HardwareMap hwMap = null;

    /**
     * Constructor for Hardware types, takes the HardwareMap from the OpMode and saves it for later.
     *
     * @param hw    The hardware map we'll use for this particular OpMode run
     */
    public Hardware(HardwareMap hw) {
        hwMap = hw;
    }

    /**
     * This is the initialization routine for every OpMode in the system.  This is intended to be
     * overriden by all of the sub-classes and they then call super.robot_init() in their
     * methods so that I get called as well.
     *
     * This method goes and instantiates every element in the hardware map that is appropriate
     * for the type of robot we are and then sets the initial configuration options for them.
     *
     * @param hwMap    The hardware map entry from the OpMode.
     */
    public void robot_init() {
        initMotor();
    }

    /**
     * Configure and Initialize the two back motors.  Note, polymorphism will cause my sub-classes
     * to call theirs with an override and then they'll call super.initMotor().
     */
    public void initMotor() {
        LBack = hwMap.dcMotor.get(Constants.LBACK);
        RBack = hwMap.dcMotor.get(Constants.RBACK);

            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //rpCenter = hwMap.dcMotor.get(cfgrpCenter);
            if( robotType != MEC_TROLLBOT) {
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            }

            else {
                rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (robotType == FULLTELEOP) {
                leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if ( robotType == FULLAUTO || robotType == FULLTELEOP  || robotType == TROLLBOTMANIP || robotType == MEC_TROLLBOT) {
                if ( robotType != MEC_TROLLBOT ) {
                    lift = hwMap.dcMotor.get(cfgLift);
                    //lift.setDirection(DcMotor.Direction.REVERSE);
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    waitForReset(lift, 2000);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (robotType == FULLTELEOP || robotType == FULLAUTO) {
                        leftManip = hwMap.dcMotor.get(cfgleftManip);
                        rightManip = hwMap.dcMotor.get(cfgrightManip);
                        leftManip.setDirection(DcMotor.Direction.REVERSE);
                    }
                }
                if (robotType != FULLAUTO && robotType != FULLTELEOP){
                   leftMidDrive = hwMap.dcMotor.get(cfgLMidDrive);
                   rightMidDrive = hwMap.dcMotor.get(cfgRMidDrive);
                   if(robotType != MEC_TROLLBOT) {
                       rightMidDrive.setDirection(DcMotor.Direction.FORWARD);
                       leftMidDrive.setDirection(DcMotor.Direction.REVERSE);
                   }

                   else {
                       rightMidDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                       leftMidDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                   }

                   rightMidDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                   leftMidDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               }
            }

            /*
             * If autonomous, then reset encoders and set mode to be with encoders
             * NOTE:  This should really throw an exception or something, but all it does
             * is silently ignore if the resetting of the encoders didn't work and blindly
             * sets the mode to be RUN_USING_ENCODER.  Can't really call telemetry because
             * don't have a caller to know which should build the telemetry and send.  Badness
             * will ensue when the actual autonomous runs and hopefully this note will help
             * folks figure out the failed reset might be at fault.
             */

            boolean resetOk = false;
            if ( robotType == AUTOTEST || robotType == FULLAUTO || robotType == COLORTEST ) {
                resetOk = waitForReset(leftBackDrive, rightBackDrive, 2000);
               // if ( robotType == FULLAUTO )
                 //   resetOk = resetOk && waitForReset(leftMidDrive, rightMidDrive, 2000);
                if (resetOk) {
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  //  if ( robotType == FULLAUTO ) {
                   //     leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                     //   rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                 //   }
                }
            }


         /* Now that hardware is mapped, set to initial positions/settings. */
        if ( robotType != TROLLBOT_SERVOTEST ) {
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            if ( robotType == TROLLBOT || robotType == MEC_TROLLBOT) {
                leftMidDrive.setPower(0);
                rightMidDrive.setPower(0);

            }
        }
    }

    /**
     * This method is used to initialize servos and set their positions.
     */
    private void initServo() {
        if ( robotType == TROLLBOT_SERVOTEST ) {
            servotest = hwMap.servo.get(cfgServoTest);
        }

        if ( robotType == COLORTEST || robotType == FULLTELEOP || robotType == FULLAUTO ) {
            colorArm = hwMap.servo.get(cfgcolorArm);
            colorArm.setPosition(cArmStart);
            colorGate = hwMap.servo.get(cfgcolorGate);
            colorGate.setPosition(cGateStart);

        }
    }

    /**
     * This method is used to initialize the sensors based on the type of sensor and robot.
     */
    private void initSensor() {

        /* Set the sensors based on type */
        if ( robotType == AUTOTEST || robotType == COLORTEST || robotType == FULLAUTO || robotType == FULLTELEOP ) {
            color = hwMap.colorSensor.get(cfgrpColorSensor);
        }

        /* Set the sensors based on type */


        /* Get the Gyro */
        if ( robotType == AUTOTEST || robotType == COLORTEST ) {
           gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get(cfgGyro);


        }
        else if( robotType == FULLAUTO){
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
            imu = hwMap.get(BNO055IMU.class, cfgIMU);
            //imu = (BNO055IMU) hwMap.gyroSensor.get(cfgIMU);
            imu.initialize(parameters);
        }
    }
    //__________________________________________________________________________________________________
    //BELOW IS THE getHeading METHOD

    public double getHeading() {

        double retVal = 0;

        if (imu != null) {

            retVal = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (retVal > 0 && retVal < 180) {
                retVal = 360 - retVal;
            } else if (retVal < 0 && retVal > -180) {
                retVal = -1 * retVal;
            }

        }
        else if (gyro != null) {

            retVal = getHeading();

        }
        return retVal;
    }
    //ABOVE IS THE getHeading METHOD
//__________________________________________________________________________________________________

    public void gyroCalibrate() {

        if (gyro != null) {
        gyro.calibrate();

        }


    }

    public boolean gyroIsCalibrating() {

        if (gyro != null) {
            return gyro.isCalibrating();
        }
        else{
            return false;
        }
    }


    public void gyroResetZAxisIntegrator() {

        if (gyro != null) {
            gyro.resetZAxisIntegrator();
        }
    }

    /**
     * TODO THIS ROUTINE NEEDS TO BE REPLACED - REVERTED due to other errors
     *
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
        double rightPower;
        double leftPower;
        double turnspeed = TURN_SPEED;
        double stopTime = runtime.seconds() + timeoutS;

        do {
            gHeading = getHeading();
            //zValue = gyro.getIntegratedZValue();
            //heading360 = zValue % 360;
//            if ( heading360 > 0 )
//                absHeading = heading360;
//            else
//                absHeading = heading360 + 360;
            //deltaHeading = absHeading - heading;
            //caller.telemetry.addData("gyroTurn:", "delta: %d absHeading: %d, currently at %d going to %d", deltaHeading, absHeading, zValue, heading);
            caller.telemetry.addData("gyroTurn:", "gHeading: %.1f, going to %.1f", gHeading, heading);
            caller.telemetry.update();
            //caller.sleep(1000);
//            if (Math.abs(deltaHeading) <= 180 && heading360 > 180) {
//                leftPower = -turnspeed;
//                rightPower = turnspeed;
//            } else {
//                leftPower = turnspeed;
//                rightPower = -turnspeed;
//            }
            /*
             * Turn left if the difference from where we're heading to where we want to head
             * is smaller than -180 or is between 1 and 180.  All else (including the 0 and 180
             * situations) turn right.
             */
            deltaHeading = gHeading - heading;
            if ( deltaHeading < -180 || (deltaHeading > 0 && deltaHeading < 180) ) {
                leftPower = -turnspeed;
                rightPower = turnspeed;
            } else {
                leftPower = turnspeed;
                rightPower = -turnspeed;
            }
            if (robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
                leftMidDrive.setPower(leftPower);
                rightMidDrive.setPower(rightPower);
            }
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            gHeading = getHeading();
        }
//        while (caller.opModeIsActive() && Math.abs(deltaHeading) > 1 && runtime.seconds() < stopTime );
        while (caller.opModeIsActive() && Math.abs(gHeading - heading) > 0.4 && runtime.seconds() < stopTime );
        if (robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
            leftMidDrive.setPower(0.0);
            rightMidDrive.setPower(0.0);
        }
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
//        if ( deltaHeading <= 1 )
        if ( Math.abs(gHeading - heading) <= 1 )
            return true;
        else
            return false;
//        if(heading360  < 0){
//              absHeading = heading360 + 360;
//        } else {
//             absHeading = heading360;
//        }

//        int deltaHeading = absHeading - heading;
//
//        if (deltaHeading <=180){
//            do {
//                leftMidDrive.setPower(-turnspeed);
//                rightMidDrive.setPower(turnspeed);
//                leftBackDrive.setPower(-turnspeed);
//                rightBackDrive.setPower(turnspeed);
//                zValue = gyro.getIntegratedZValue();
//                caller.telemetry.addData("gyroTurn:", "Turning to %d, currently at %d", absHeading, zValue);
//                caller.telemetry.update();
//            }
//            while (caller.opModeIsActive() && Math.abs(zValue - absHeading) > 1 && runtime.seconds() < stopTime);
//        } else {
//            do {
//                leftMidDrive.setPower(turnspeed);
//                rightMidDrive.setPower(-turnspeed);
//                leftBackDrive.setPower(turnspeed);
//                rightBackDrive.setPower(-turnspeed);
//                zValue = gyro.getIntegratedZValue();
//                caller.telemetry.addData("gyroTurn:", "Turning to %d, currently at %d", absHeading, zValue);
//                caller.telemetry.update();
//            }
//            while (caller.opModeIsActive() && Math.abs(zValue - absHeading) > 1 && runtime.seconds() < stopTime);
//        }
//        leftMidDrive.setPower(0.0);
//        rightMidDrive.setPower(0.0);
//        leftBackDrive.setPower(0.0);
//        rightBackDrive.setPower(0.0);
//        return Math.abs(gyro.getIntegratedZValue() - absHeading) <= 1;
        }
/*This code we wrote establishes different things 1.  It makes sure that during gyro that if certain
statements are true than the code will stop working, 2. I don't know what else.
 */
static final double     AutoFrontTurnSpeed              = 0.15;
    public boolean AutoFrontsTurn(LinearOpMode caller,
                            double heading,
                            double timeoutS) throws InterruptedException {
        int zValue;
        double gHeading;
        int heading360;
        int absHeading;
        double deltaHeading;
        double rightPower;
        double leftPower;
        double turnspeed = AutoFrontTurnSpeed;
        double stopTime = runtime.seconds() + timeoutS;

        do {
            gHeading = getHeading();
            caller.telemetry.addData("AutoFrontsTurn:", "gHeading: %.1f, going to %.1f", gHeading, heading);
            caller.telemetry.update();

            deltaHeading = gHeading - heading;
            if ( deltaHeading < -180 || (deltaHeading > 0 && deltaHeading < 180) ) {
                leftPower = -turnspeed;
                rightPower = turnspeed;
            } else {
                leftPower = turnspeed;
                rightPower = -turnspeed;
            }
            if (robotType == FULLTELEOP || robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
                leftMidDrive.setPower(leftPower);
                rightMidDrive.setPower(rightPower);
            }
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            gHeading = getHeading();
        }
        while (caller.opModeIsActive() && Math.abs(gHeading - heading) > 0.4 && runtime.seconds() < stopTime );
        if ( robotType == TROLLBOT || robotType == TROLLBOTMANIP) {
            leftMidDrive.setPower(0.0);
            rightMidDrive.setPower(0.0);
        }
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        if ( Math.abs(gHeading - heading) <= 1 )
            return true;
        else
            return false;
    }
    /**
     * Drive by the encoders, running to a position relative to the current position based
     * on encoder ticks for a left and right motor.  It will move to a position for a specified
     * period of time, so it will stop if they get to the desired position, if the time runs out
     * or if the OpMode is cancelled.
     *
     * Originally written in PushbotAutoDriveByEncoder_Linear.java from FtcRobotController area,
     * modified by FTC8424 for defensive encoder moves.
     *
     * @param caller                  Reference to calling class, must be LinearOpMode
     * @param speed                   The speed of the movement
     * @param leftInches              The target position of left motors, in inches from current
     * @param rightInches           The target position of right motors, in inches from current
     *
    * @param timeoutS                The timeout in seconds to allow the move
     * @throws InterruptedException
     */
    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        int newLeftMidTarget;
        int newRightMidTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        //int getHeading = gyro.getIntegratedZValue();
        long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

        if ( !caller.opModeIsActive() )
            return;

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            newLeftMidTarget = 0;
            newRightMidTarget = 0;
       // }
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
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

        lbEncoderSet = setEncoderPosition(caller, leftBackDrive, newLeftBackTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, rightBackDrive, newRightBackTarget, encoderTimeout);
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
        int lmCurPos;
        int rmCurPos;
        int lbCurPos;
        int rbCurPos;
        double stopTime = runtime.seconds() + timeoutS;
        double leftPower;
        double rightPower;
        double lastSetTime = runtime.milliseconds();
        int HeadingLoop;

        do {
            leftPower = speed;
            rightPower = speed;
            if (leftPower <= 0.01) {
                lastSetTime = runtime.milliseconds();
                leftPower = speed;
                rightPower = speed;
            }

            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);

         //   if(robotType == FULLAUTO){
           //     leftMidDrive.setPower(leftPower);
             //   rightMidDrive.setPower(rightPower);
            //}
            caller.telemetry.addData("Power:", "Left Power %.2f, Right Power %.2f", leftPower, rightPower);
            caller.telemetry.update();
            lbCurPos = leftBackDrive.getCurrentPosition();
            rbCurPos = rightBackDrive.getCurrentPosition();
         //   if ( robotType == FULLAUTO ) {
          //      lmCurPos = leftMidDrive.getCurrentPosition();
          //      rmCurPos = rightMidDrive.getCurrentPosition();
          //  } else {
                lmCurPos = Integer.MAX_VALUE;
                rmCurPos = Integer.MAX_VALUE;
          //  }
            isBusy = (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPos - newRightBackTarget) >= 5);
       //     if ( robotType == FULLAUTO )
        //        isBusy = isBusy && (Math.abs(lmCurPos - newLeftMidTarget) >= 5) && (Math.abs(rmCurPos - newRightMidTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
       // if ( robotType == FULLAUTO ) {
        //    leftMidDrive.setPower(0);
        //    rightMidDrive.setPower(0);
       // }

        // Turn off RUN_TO_POSITION

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  if ( robotType == FULLAUTO ) {
      //      leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //      rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  }
    }

    /**
     * Drive the robot forward/backward based on power settings passed in.
     *
     * @param leftPower      Power setting (-1.0 - 1.0)
     * @param rightPower     Power setting (-1.0 - 1.0)
     */

    public void normalDrive (OpMode caller, double leftPower, double rightPower) {
        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        if ( robotType == TROLLBOT || robotType == MEC_TROLLBOT ) {
            leftMidDrive.setPower(leftPower);
            rightMidDrive.setPower(rightPower);
        }
        caller.telemetry.addData("normalDrive:", "Power set to L:%.2f, R:%.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
    }

    /**
     * Drive the robot side to side based on power settings passed in.
     *
     * @param leftPower      Power setting (-1.0 - 1.0)
     * @param rightPower     Power setting (-1.0 - 1.0)
     */

    public void sideDrive (OpMode caller, double leftPower, double rightPower) {
        leftBackDrive.setPower(-leftPower);
        rightBackDrive.setPower(rightPower);
        if (  robotType == TROLLBOT ) {
                leftMidDrive.setPower(leftPower);
                rightMidDrive.setPower(-rightPower);
        }
        caller.telemetry.addData("sideDrive:", "Power set to L:%.2f, R:%.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
    }

    /**
     * Get the voltage of the 12V battery and return it.
     *
     * @return  The voltage of the 12V
     */
    public double getVoltage() {
        return hwMap.voltageSensor.iterator().next().getVoltage();
    }

    /**
     * Waits for the encoders to be reset on the 4 motors, and returns a boolean as to whether
     * the motors have reset or not.  Calls waitForReset() with two motor method signature multiple
     * times to do the actual work and combines the return values to give an overall return of
     * true only if all four motors were properly reset.
     *
     * @param m1    Motor 1 to reset
     * @param m2    Motor 2 to reset
     * @param m3    Motor 3 to reset
     * @param m4    Motor 4 to reset
     * @param msTimeOut The time to wait, in milliseonds, for a valid reset
     * @return      Whether the reset was successful or not
     */
    public boolean waitForReset(DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4, long msTimeOut) {
        boolean resetOk = false;

        resetOk = waitForReset(m1, m2, msTimeOut/2);
        return resetOk && waitForReset(m3, m4, msTimeOut/2);
    }
    public boolean waitForReset(DcMotor m1, long msTimeOut) {
        boolean resetOk = false;

        resetOk = waitForReset(m1, null, msTimeOut);
        return resetOk ;
    }

    /**
     * Waits for the encoders to be reset on the 2 motors and returns a boolean as to whether
     * the motors have reset or not.  If they haven't reset in the timeOut milliseconds, then
     * it will return false.  This is the true method that does stuff, the other method with the
     * 4 motor signature just calls this method multiple times.
     *
     * @param m1        Motor 1 to reset
     * @param m2        Motor 2 to reset
     * @param msTimeOut   The time to wait, in milliseconds, for a valid reset
     * @return          Whether the reset was successful or not
     */
    public boolean waitForReset(DcMotor m1, DcMotor m2, long msTimeOut) {
        int m1Pos = m1.getCurrentPosition();
        int m2Pos = 0 ;//m2.getCurrentPosition();
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(m2 != null){
            m2Pos = m2.getCurrentPosition();
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        double stopTime = runtime.milliseconds() + msTimeOut;
        while ( (m1Pos != 0 || m2Pos != 0) && runtime.milliseconds() < stopTime ) {
            m1Pos = m1.getCurrentPosition();
            if(m2 != null){
                m2Pos = m2.getCurrentPosition();
            }

        }
        return m1Pos == 0 && m2Pos == 0;
    }

    /**
     * Sets the target encoder position for this particular motor and waits to ensure that the
     * position was properly set.  If it didn't set correctly, within the timeOut (in milliseconds)
     * time, then it will returns false.
     *
     * @param caller    The calling OpMode (for opModeIsActive() call)
     * @param m1        The motor on which to set the position
     * @param target    The target position to set
     * @param timeOut   The time, in milliseconds, for it to properly set
     * @return          True if target set within timeOut milliseconds, false otherwise
     */
    private boolean setEncoderPosition (LinearOpMode caller, DcMotor m1, int target, long timeOut) {
        m1.setTargetPosition(target);
        int m1Pos = m1.getTargetPosition();
        double stopTime = runtime.milliseconds() + timeOut;
        while ( caller.opModeIsActive() && m1Pos != target && runtime.milliseconds() < stopTime ) {
            m1.setTargetPosition(target);
            m1Pos = m1.getTargetPosition();
        }
        return m1Pos == target;
    }
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( LinearOpMode caller,
                            double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (caller.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * encoderInch);
            newLeftTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (caller.opModeIsActive() &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftBackDrive.setPower(leftSpeed);
                rightBackDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                caller.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                caller.telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                caller.telemetry.addData("Actual",  "%7d:%7d",      leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                caller.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                caller.telemetry.update();
            }

            // Stop all motion;
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn2 (  LinearOpMode caller, double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (caller.opModeIsActive() && !onHeading (caller, speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            caller.telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( LinearOpMode caller, double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (caller.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(caller,speed, angle, P_TURN_COEFF);
            caller.telemetry.update();
        }

        // Stop all motion;
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /****************************************************************************************
     * Private methods follow.
     ****************************************************************************************/

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(LinearOpMode caller, double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftBackDrive.setPower(leftSpeed);
        rightBackDrive.setPower(rightSpeed);

        // Display it for the driver.
        caller.telemetry.addData("Target", "%5.2f", angle);
        caller.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        caller.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
