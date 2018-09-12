package org.firstinspires.ftc.teamcode;

/**
 * Created by aagrockstar on 4/17/2018.
 */

public class MecanumHelper {

    /**
     * This method takes the values from the game pad and turns them into proper power
     * settings for each of the motors.
     *
     * @param magnitude
     *  This is the hypotenuse from right triangle of left stick values
     * @param angle
     *  This is the ARC Tangent from right triangle of left stick values
     * @param rotation
     *  This is the right stick x for how quickly to rotate to the angle
     * @return
     *   An array of doubles for power values to left front, right front, left back and right back
     */
    public double[] motorPower (double magnitude, double angle, double rotation) {
        double[] power = {0.0, 0.0, 0.0, 0.0};

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


}
