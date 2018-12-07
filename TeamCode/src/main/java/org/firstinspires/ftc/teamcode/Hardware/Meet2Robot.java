package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class Meet2Robot extends MecanumDrive {

    public Servo MarkerServo = null;
    public  Servo LockServo = null;
    public double MarkerStart = 0.0;
    public double MarkerDeploy = 1.0;
    public double MarkerInit = 0.5;
    public double LiftLock = 1.0;
    public double LiftUnlock = 0.0;
    public DcMotor LiftMotor = null;
    public int LiftUp =  -2046;
    public int LiftDown = 0;

    @Override
    public void initServo(){
        super.initServo();
        LockServo = hwMap.servo.get(Constants.LockServo);
        MarkerServo = hwMap.servo.get(Constants.MarkerServo);
        try {
            //deploy(ColorServo, ColorSample);
            //deploy(ColorServo, MarkerStart);
            deploy(MarkerServo, MarkerInit);
            deploy(MarkerServo, MarkerStart);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void initMotor(boolean revLeft) {
        super.initMotor(revLeft);
        LiftMotor = hwMap.dcMotor.get(Constants.LiftMotor);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setPower(0);

    }

    // TODO: Fix this to use the TensorFlow object from the ConceptTensorFlowObjectDetection and return true if GoldMineralX is >= 200 && <= 400
    public boolean isGold(){

       /* int redValue = color.red();
        int blueValue = color.blue();
        int greenValue = color.green();
        if (blueValue > 20 && greenValue > 25 && redValue > 35){
            return false;
        }
        else if (blueValue < 10 && greenValue > 10 && redValue > 10) {
            return true;
        } else {
            return false;
        }*/
       return false;
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
            for (double d = currentPos; d > targetPos; d -= 0.1) {
                servo.setPosition(d);
                sleep(100);
            }
        }
        else {
            for (double d = currentPos; d < targetPos; d += 0.1) {
                servo.setPosition(d);
                sleep(100);
            }
        }

    }
}