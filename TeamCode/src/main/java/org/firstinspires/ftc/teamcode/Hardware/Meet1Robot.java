package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class Meet1Robot extends MecanumDrive {

    public Servo ColorServo = null;
    public Servo MarkerServo = null;
    public double ColorStart = 0.0;
    public double ColorDeploy = 1.0;
    public double ColorSample = 0.5;
    public DcMotor LiftMotor = null;


    @Override
    public void initServo(){
        super.initServo();
        ColorServo = hwMap.servo.get(Constants.ColorServo);
        MarkerServo = hwMap.servo.get(Constants.MarkerServo);
        try {
            //deploy(ColorServo, ColorSample);
            //deploy(ColorServo, ColorStart);
            deploy(MarkerServo, ColorSample);
            deploy(MarkerServo, ColorStart);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void initMotor(boolean revLeft) {
        super.initMotor(revLeft);
        LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor = hwMap.dcMotor.get(Constants.LiftMotor);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor.setPower(0);

    }

    /**
     * This is the method used for 180 degree servos.  It gets their current position and sets
     * it to their other position.  It will just be a bunch of if..elseif..else statements
     * for each 180-degree servo to determine the target position.  If we can't find the
     * type of servo we are, then just return back to the caller.
     *
     * @param servo
     */
    public void deploy(Servo servo, double targetPos) throws InterruptedException {
        double currentPos = servo.getPosition();
        if (currentPos > targetPos){
            for (double d = currentPos; d >= targetPos; d -= 0.1) {
                servo.setPosition(d);
                sleep(50);
            }
        }
        else {
            for (double d = currentPos; d <= targetPos; d += 0.1) {
                servo.setPosition(d);
                sleep(50);
            }
        }

    }
}