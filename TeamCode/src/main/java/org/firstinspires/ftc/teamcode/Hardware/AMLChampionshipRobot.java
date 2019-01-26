package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class AMLChampionshipRobot extends Meet2Robot {

    public DcMotor ClawMotor = null;
    public int ClawDown = -191;
    public int ClawBasket = 45;
    public int ClawStraight = 53;
    public DcMotor BasketMotor = null;
    public Servo ClawServo = null;


    @Override
    public void initServo(){
        super.initServo();
        ClawServo = hwMap.servo.get(Constants.ClawServo);
        ClawServo.setPosition(.5);

        /*LockServo = hwMap.servo.get(Constants.LockServo);
        MarkerServo = hwMap.servo.get(Constants.MarkerServo);
        try {
            //deploy(ColorServo, ColorSample);
            //deploy(ColorServo, MarkerStart);
            deploy(MarkerServo, MarkerInit);
            sleep(100);
            deploy(MarkerServo, MarkerStart);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/
    }

    @Override
    public void initMotor(boolean revLeft) {
        super.initMotor(revLeft);
       /* LiftMotor = hwMap.dcMotor.get(Constants.LiftMotor);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftMotor.setPower(0);*/
        ClawMotor = hwMap.dcMotor.get(Constants.ClawMotor);
        ClawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ClawMotor.setPower(0);
        BasketMotor = hwMap.dcMotor.get(Constants.BasketMotor);
        BasketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BasketMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BasketMotor.setPower(0);
    }
    public void setEncoderMode (DcMotor dcMotor, DcMotor.RunMode mode) {
        dcMotor.setMode(mode);
    }

    // TODO: Fix this to use the TensorFlow object from the ConceptTensorFlowObjectDetection and return true if GoldMineralX is >= 200 && <= 400
    /*public boolean isGold(){



        int redValue = color.red();
        int blueValue = color.blue();
        int greenValue = color.green();
        if (blueValue > 20 && greenValue > 25 && redValue > 35){
            return false;
        }
        else if (blueValue < 10 && greenValue > 10 && redValue > 10) {
            return true;
        } else {
            return false;
        }
       return false;
    }*/

    /**
     * This is the method used for 180 degree servos.  It gets their current position and sets
     * it to their other position.  It will just be a bunch of if..elseif..else statements
     * for each 180-degree servo to determine the target position.  If we can't find the
     * type of servo we are, then just return back to the caller.
     *
     * @param servo
     */
    /*public void deploy(Servo servo, double targetPos) throws InterruptedException {
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

    }*/
}