package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Meet1Robot extends MecanumDrive {

    public Servo ColorServo = null;
    private double ColorStart = 0.1;
    private double ColorDeploy = 0.9;
    private double ColorSample = .5;
    public DcMotor LiftMotor = null;


    @Override
    public void initServo(){
        super.initServo();
        ColorServo = hwMap.servo.get(Constants.ColorServo);

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
    public void deploy(Servo servo) {
        double targetPosition = 0;
        if (servo.equals(ColorServo)) targetPosition = servo.getPosition()== ColorStart ? ColorDeploy : ColorStart;
        else return;
        servo.setPosition(targetPosition);

    }
}