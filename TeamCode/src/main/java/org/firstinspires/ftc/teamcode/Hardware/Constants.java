package org.firstinspires.ftc.teamcode.Hardware;

public final class Constants {
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /**
     * All REV electronics will have at least one IMU defined (port 0 on sensors), and if there's
     * a second REV Expansion Hub, then the second imu will need to be renamed to imu2.
     */
    public static final String IMU = "imu";
    public static final String IMU2 = "imu2";

    /**
     * The basic robot's configuration:  two motors, left back and right back.
     */
    public static final String LBACK = "l back";
    public static final String RBACK = "r back";

    /**
     * This robot has two front motors as well.
     */
    public static final String LFRONT = "l front";
    public static final String RFRONT = "r front";

    public static final String ColorSensor = "Color Sensor";

    public static final String ColorServo = "Color Servo";
    public static final String MarkerServo = "M Ser";

    public static final double TURN_SPEED = 0.3;
    public static final String LiftMotor = "Lift Motor";

    public static final String LockServo = "L Ser";

    /**
     * The Vuforia license key, used by all opModes
     *
     *      * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     *      * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     *      * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     *      * web site at https://developer.vuforia.com/license-manager.
     *      *
     *      * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     *      * random data. As an example, here is a example of a fragment of a valid key:
     *      *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     *      * Once you've obtained a license key, copy the string from the Vuforia web site
     *      * and paste it in to your code on the next line, between the double quotes.
     *
     */
    public static final String VUFORIA_KEY = " AV2itX3/////AAABmWuabpdKwU9OnMZ3SQ3fo3Bw8HKkTNAKbZWLSp6YojfTOW1XKsHONIejtzHGovWa7sqg8oXfz7KWdzjlhd/+i0404zTpAx22nSC/CKoC0vw6CEsuhznCwgK4GifS1OFCgj1UdDjGPjP5B9uxkAqIKhqiQ92MUSdt5fPo19XeMrVwCbiJT/NEy+1kG5FPatw5Wq8RKNfA0ScHx3as+U2bCIs6Jw5i/M7n78oZ+j2M3XHvAt2tEZTq0e0pcRQy7B2TrW21zmvDXgYqR0ApD0aM/wqZJKKXWgx6FQa+RN8tidGXoXtierZSLvbg/hNLLe7I0EGw+a3k4oKOeL3mWdxDGU/5KEyVF6x0gl6zGerSLSao\n";
}
