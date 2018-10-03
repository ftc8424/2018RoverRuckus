package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Hardware4Motor extends Hardware {

    /* These are values that can be used by the opModes (e.g., robot.LBack.setPower(); */
    public DcMotor LFront = null;
    public DcMotor RFront = null;

    /**
     * Constructor for Hardware types, takes the HardwareMap from the OpMode and saves it for later.
     * Each sub-class should call super.Hardware(hw) in case any of the classes in the hierarchy
     * wants to do something at construction time.
     *
     * @param hw The hardware map we'll use for this particular OpMode run
     */
    public Hardware4Motor(HardwareMap hw) {
        super(hw);
    }

    /**
     * Configure and initialize the two back motors, with no reversal.  SUB-CLASS MUST OVERRIDE.
     * <p>
     * Polymorphism will cause sub-classes to call theirs and they must then call super.initMotor()
     * to get me to be called.
     */
    @Override
    public void initMotor() {
        initMotor(false);
    }

    /**
     * Configure and Initialize the two back motors, reversing the left side if revLeft is true, SUBCLASS MUST OVERRIDE.
     * <p>
     * Note, polymorphism will cause my sub-classes to call theirs with an override and then
     * they'll call super.initMotor().  If the type of robot is using a normal drive, then the left
     * side motors will be set to REVERSE so that giving them the positive values moves the robot
     * forward.  In those cases, initMotor(true) should be called.
     * <p>
     * If this is a holonomic drive (e.g., Mecanum or omni-wheel in crab-drive format) then you
     * don't want to do that, so pass with false (or just call the version without parameters).
     *
     * @param revLeft Should I reverse the left motors?
     */
    @Override
    public void initMotor(boolean revLeft) {
        LFront = hwMap.dcMotor.get(Constants.LFRONT);
        RFront = hwMap.dcMotor.get(Constants.RFRONT);

        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (revLeft)
            LFront.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Default to no encoders
        LFront.setPower(0);
        RFront.setPower(0);

        super.initMotor(revLeft);
    }

    /**
     * Set the Encoder mode for the base class's motors.  This should be overridden in the
     * sub-classes and they should call super.setEncoderMode() if they need to set a value.
     *
     * @param mode
     */
    @Override
    public void setEncoderMode(DcMotor.RunMode mode) {
        LFront.setMode(mode);
        RFront.setMode(mode);
        super.setEncoderMode(mode);
    }

    public void normalDrive (OpMode caller, double leftBackPower, double rightBackPower, double leftFrontPower, double rightFrontPower) {
        LFront.setPower(leftFrontPower);
        RFront.setPower(rightFrontPower);
        //LBack.setPower(leftBackPower);
        //RBack.setPower(rightBackPower);
        caller.telemetry.addData("normalDrive:", "Front Power set to L:%.2f, R:%.2f", leftFrontPower, rightFrontPower);


        super.normalDrive(caller, leftBackPower, rightBackPower);
    }
    @Override
    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        //int newLeftBackTarget;
        //int newRightBackTarget;
        //int getHeading = gyro.getIntegratedZValue();
        long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

        if ( !caller.opModeIsActive() )
            return;

        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        newLeftFrontTarget = 0;
        newRightFrontTarget = 0;
        // }
        newLeftFrontTarget = LFront.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightFrontTarget = RFront.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
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

        lbEncoderSet = setEncoderPosition(caller, LFront, newLeftFrontTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, RFront, newRightFrontTarget, encoderTimeout);
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
        int lfCurPos;
        int rfCurPos;
        double stopTime = runtime.seconds() + timeoutS;
        double leftFrontPower;
        double rightFrontPower;
        double lastSetTime = runtime.milliseconds();
        int HeadingLoop;

        do {
            leftFrontPower = speed;
            rightFrontPower = speed;
            if (leftFrontPower <= 0.01) {
                lastSetTime = runtime.milliseconds();
                leftFrontPower = speed;
                rightFrontPower = speed;
            }

            leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
            rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
            LFront.setPower(leftFrontPower);
            RFront.setPower(rightFrontPower);

            //   if(robotType == FULLAUTO){
            //     leftMidDrive.setPower(leftBackPower);
            //   rightMidDrive.setPower(rightBackPower);
            //}
            caller.telemetry.addData("Power:", "Left Power %.2f, Right Power %.2f", leftFrontPower, rightFrontPower);
            caller.telemetry.update();
            lfCurPos = LFront.getCurrentPosition();
            rfCurPos = RFront.getCurrentPosition();
            //   if ( robotType == FULLAUTO ) {
            //      lmCurPos = leftMidDrive.getCurrentPosition();
            //      rmCurPos = rightMidDrive.getCurrentPosition();
            //  } else {

            //  }
            isBusy = (Math.abs(lfCurPos - newLeftFrontTarget) >= 5) && (Math.abs(rfCurPos - newRightFrontTarget) >= 5);
            //     if ( robotType == FULLAUTO )
            //        isBusy = isBusy && (Math.abs(lmCurPos - newLeftMidTarget) >= 5) && (Math.abs(rmCurPos - newRightMidTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        LFront.setPower(0);
        RFront.setPower(0);
        // if ( robotType == FULLAUTO ) {
        //    leftMidDrive.setPower(0);
        //    rightMidDrive.setPower(0);
        // }

        // Turn off RUN_TO_POSITION

        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  if ( robotType == FULLAUTO ) {
        //      leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //      rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  }
    }

}