package org.firstinspires.ftc.teamcode;

/**
 * Created by aagrockstar on 4/17/2018.
 */


public class HardwareMecanum extends Hardware4Motor{

    /**
     * This method takes the values from the x and y of game pad and turns them into proper power
     * settings for each of the motors.
     *
     * @param x
     *  This is the x stick value, denoting the heading of the robot
     * @param y
     *  This is the y stick value, denoting the direction/translation of robot
     * @return
     *   An array of doubles for power values to left front, right front, left back and right back
     */

    public double[] motorPower (double leftX, double leftY, double rightX) {
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
     * @return
     *  This is returning the scaled value of the wheel powers, in left front, right front, left back
     *  and right back.
     */
    public double[] scalePower (double lfPower, double rfPower, double lbPower, double rbPower) {
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


    // TODO:  Add encoderDrive() method for driving by encoders using Mecanum drive power sets

}
