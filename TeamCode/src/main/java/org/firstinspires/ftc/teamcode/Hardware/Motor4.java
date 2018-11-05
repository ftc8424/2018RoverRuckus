package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Motor4 extends Base {

    /* These are values that can be used by the opModes (e.g., robot.LBack.setPower(); */
    public DcMotor LFront = null;
    public DcMotor RFront = null;

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


        if (revLeft) {
            LFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        super.initMotor(revLeft);

        this.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Default to no encoders
        LFront.setPower(0);
        RFront.setPower(0);
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
        //super.setEncoderMode(mode);
    }

    public void normalDrive (OpMode caller, double leftPower, double rightPower) {
        super.normalDrive(caller, leftPower, rightPower);
        LFront.setPower(leftPower);
        RFront.setPower(rightPower);
        caller.telemetry.addData("M4-normalDrive:", "Front Power set to L:%.2f, R:%.2f", leftPower, rightPower);

    }

    @Override
    public void encoderDrive(LinearOpMode caller,
                             double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        //int getHeading = gyro.getIntegratedZValue();
        long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

        if ( !caller.opModeIsActive() )
            return;

        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        newLeftFrontTarget = 0;
        newRightFrontTarget = 0;
        newRightBackTarget = 0;
        newLeftBackTarget = 0;
        // }
        newLeftFrontTarget = LFront.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightFrontTarget = RFront.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
        newLeftBackTarget = LBack.getCurrentPosition() + (int)Math.round(leftInches * encoderInch);
        newRightBackTarget = RBack.getCurrentPosition() + (int)Math.round(rightInches * encoderInch);
//        caller.telemetry.addLine("encoderDrive-MID:")
//                .addData("Left Tgt POS: ", newLeftMidTarget)
//                .addData("Right Tgt POS:" ,  newRightMidTarget);
//        caller.telemetry.addLine("EncoderDrive-BCK:")
//                .addData("Left Tgt POS: ", newLeftBackTarget)
//                .addData("Right Tgt POS: ", newRightBackTarget);
//        caller.telemetry.update();

        boolean lfEncoderSet = false;
        boolean rfEncoderSet = false;
        boolean lbEncoderSet = false;
        boolean rbEncoderSet = false;

        lfEncoderSet = setEncoderPosition(caller, LFront, newLeftFrontTarget, encoderTimeout);
        rfEncoderSet = setEncoderPosition(caller, RFront, newRightFrontTarget, encoderTimeout);
        lbEncoderSet = setEncoderPosition(caller, LFront, newLeftFrontTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, RFront, newRightFrontTarget, encoderTimeout);
        //  if ( robotType == FULLAUTO ) {
        //    lmEncoderSet = setEncoderPosition(caller, leftMidDrive, newLeftMidTarget, encoderTimeout);
        //     rmEncoderSet = setEncoderPosition(caller, rightMidDrive, newRightMidTarget, encoderTimeout);
        //} else {
        lfEncoderSet = true;
        rfEncoderSet = true;
        lbEncoderSet = true;
        rbEncoderSet = true;
        //  }
//        caller.telemetry.addLine("EncoderSet:")
//                .addData("LB: ", lbEncoderSet)
//                .addData("RB: ", rbEncoderSet)
//                .addData("LM: ", lmEncoderSet)
//                .addData("RM: ", rmEncoderSet);
//        caller.telemetry.update();
        if ( ! (lfEncoderSet && lbEncoderSet && rfEncoderSet && rbEncoderSet) ) {
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
        int lbCurPos;
        int rbCurPob;
        double stopTime = runtime.seconds() + timeoutS;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double lastSetTime = runtime.milliseconds();
        int HeadingLoop;

        do {
            leftFrontPower = speed;
            rightFrontPower = speed;
            leftBackPower = speed;
            rightBackPower = speed;
            if (leftFrontPower <= 0.01) {
                lastSetTime = runtime.milliseconds();
                leftFrontPower = speed;
                rightFrontPower = speed;
                leftBackPower = speed;
                rightBackPower = speed;
            }

            leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
            rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
            leftBackPower = Range.clip(leftBackPower, -1.0, 1.0);
            rightBackPower = Range.clip(rightBackPower, -1.0, 1.0);
            LFront.setPower(leftFrontPower);
            RFront.setPower(rightFrontPower);
            LBack.setPower(leftBackPower);
            RBack.setPower(rightBackPower);

            //   if(robotType == FULLAUTO){
            //     leftMidDrive.setPower(leftBackPower);
            //   rightMidDrive.setPower(rightBackPower);
            //}
            caller.telemetry.addData("Power:", "Left Front Power %.2f, Right Front Power %.2f, Left Back Power %.2f, Right Back Power %.2f",
                            leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            caller.telemetry.update();
            lfCurPos = LFront.getCurrentPosition();
            rfCurPos = RFront.getCurrentPosition();
            lbCurPos = LBack.getCurrentPosition();
            rbCurPob = RBack.getCurrentPosition();
            //   if ( robotType == FULLAUTO ) {
            //      lmCurPos = leftMidDrive.getCurrentPosition();
            //      rmCurPos = rightMidDrive.getCurrentPosition();
            //  } else {

            //  }
            isBusy = (Math.abs(lfCurPos - newLeftFrontTarget) >= 5) && (Math.abs(rfCurPos - newRightFrontTarget) >= 5);
            //     if ( robotType == FULLAUTO )
            isBusy = isBusy && (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPob - newRightBackTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        LFront.setPower(0);
        RFront.setPower(0);
        LBack.setPower(0);
        RBack.setPower(0);
        // if ( robotType == FULLAUTO ) {
        //    leftMidDrive.setPower(0);
        //    rightMidDrive.setPower(0);
        // }

        // Turn off RUN_TO_POSITION

        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  if ( robotType == FULLAUTO ) {
        //      leftMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //      rightMidDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  }
    }

}