package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

/**
 * Created by aagrockstar on 4/17/2018.
 */


public class MecanumDrive extends Motor4 {

    public void initMotor(boolean revLeft) {
        super.initMotor(revLeft);
        LFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * This method takes the values from the x and y of game pad and turns them into proper power
     * settings for each of the motors.
     *
     * @param leftX This is the left x stick value, denoting the heading of the robot
     * @param leftY This is the left y stick value, denoting the direction/translation of robot
     * @param rightX This is the right x stick value, denoting something
     * @return An array of doubles for power values to left front, right front, left back and right back
     */

    public double[] motorPower(double leftX, double leftY, double rightX) {
        double[] power = {0.0, 0.0, 0.0, 0.0};

        double magnitude = Math.hypot(leftY, leftX);
        double angle = Math.atan2(leftX, leftY) + Math.PI / 4;
        double rotation = -rightX;
       /* magnitude = Range.clip (magnitude, -1, 1);
        angle = Range.clip (angle,0, 2 * Math.PI );
        rotation = Range.clip (rotation, -1,1);
*/
        power[0] = magnitude * Math.sin(angle) + rotation;

        power[1] = magnitude * Math.cos(angle) - rotation;

        power[2] = magnitude * Math.cos(angle) + rotation;

        power[3] = magnitude * Math.sin(angle) - rotation;

        return scalePower(power[0], power[1], power[2], power[3]);
        //return power;
    }

    /**
     * This method scales the power values for all of the motors so tht proper angle, direction,
     * and rotation will be maintained.
     *
     * @param lfPower
     * @param rfPower
     * @param lbPower
     * @param rbPower
     * @return This is returning the scaled value of the wheel powers, in left front, right front, left back
     * and right back.
     */
    public double[] scalePower(double lfPower, double rfPower, double lbPower, double rbPower) {
        double[] power = {lfPower, rfPower, lbPower, rbPower};

        double max = Math.abs(power[0]);

        if (Math.abs(power[1]) > max) {
            max = Math.abs(power[1]);
        }
        if (Math.abs(power[2]) > max) {
            max = Math.abs(power[2]);
        }
        if (Math.abs(power[3]) > max) {
            max = Math.abs(power[3]);
        }

        if (max > 1.0) {
            power[0] /= max;
            power[1] /= max;
            power[2] /= max;
            power[3] /= max;
        }
        return power;
    }

    // TODO:  Replace encoderStrafe() stub with real code that makes it go right/left based on inches

    public void encoderStrafe(LinearOpMode caller,
                              double speed,
                              double leftInches, double rightInches,
                              double timeoutS) throws InterruptedException {
        encoderDrive(caller, speed, leftInches, rightInches, timeoutS);
    }
    
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

        if (!caller.opModeIsActive())
            return;

        LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        newLeftFrontTarget = LFront.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
        newRightFrontTarget = RFront.getCurrentPosition() + (int) Math.round(rightInches * encoderInch);
        newLeftBackTarget = LBack.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
        newRightBackTarget = RBack.getCurrentPosition() + (int) Math.round(rightInches * encoderInch);
        caller.telemetry.addLine("encoderDrive-Front")
               .addData("Left Tgt POS", newLeftFrontTarget)
                .addData("Right Tgt POS" ,  newRightFrontTarget);
        caller.telemetry.addLine("EncoderDrive-Back:")
                .addData("Left Tgt POS", newLeftBackTarget)
                .addData("Right Tgt POS", newRightBackTarget);
       caller.telemetry.update();
       sleep(2000);

        boolean lfEncoderSet = false;
        boolean rfEncoderSet = false;
        boolean lbEncoderSet = false;
        boolean rbEncoderSet = false;

        lfEncoderSet = setEncoderPosition(caller, LFront, newLeftFrontTarget, encoderTimeout);
        rfEncoderSet = setEncoderPosition(caller, RFront, newRightFrontTarget, encoderTimeout);
        lbEncoderSet = setEncoderPosition(caller, LBack, newLeftBackTarget, encoderTimeout);
        rbEncoderSet = setEncoderPosition(caller, RBack, newRightBackTarget, encoderTimeout);
        if (!(lfEncoderSet && lbEncoderSet && rfEncoderSet && rbEncoderSet)) {
            caller.telemetry.addLine("Encoders CANNOT be set, aborting OpMode");
            caller.telemetry.update();
            caller.sleep(10000);    // Can't go any further, allow telemetry to show, then return
            return;
        }

        // keep looping while we are still active, and there is time left, and motors haven't made position.
        boolean isBusy;
        int lfCurPos;
        int rfCurPos;
        int lbCurPos;
        int rbCurPos;
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

            lfCurPos = LFront.getCurrentPosition();
            rfCurPos = RFront.getCurrentPosition();
            lbCurPos = LBack.getCurrentPosition();
            rbCurPos = RBack.getCurrentPosition();
            caller.telemetry.addData("Power:", "Left Front Power %.2f, Right Front Power %.2f, Left Back Power %.2f, Right Back Power %.2f",
                    leftFrontPower, rightFrontPower, leftBackPower, rightBackPower)
                   .addData("Position:", "Left Front  %d, Right Front  %d, Left Back  %d, Right Back  %d",
                    lfCurPos, rfCurPos, lbCurPos, rbCurPos);
            caller.telemetry.update();
            isBusy = (Math.abs(lfCurPos - newLeftFrontTarget) >= 5) && (Math.abs(rfCurPos - newRightFrontTarget) >= 5);
            isBusy = isBusy && (Math.abs(lbCurPos - newLeftBackTarget) >= 5) && (Math.abs(rbCurPos - newRightBackTarget) >= 5);
        }
        while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

        // Stop all motion;
        LFront.setPower(0);
        RFront.setPower(0);
        LBack.setPower(0);
        RBack.setPower(0);

        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO:  Add encoderDrive() method for driving by encoders using Mecanum drive power sets

    }
}




