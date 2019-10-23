package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ActualAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor rightBack;
    // DcMotor leftFront;
    //DcMotor rightFront;

    double slope;
    double areaUnderCurve;
    double totalAreaUnderCurve;
    ElapsedTime runTime = new ElapsedTime();
    double time;
    double oldTime;
    double oldAngleLeft;
    double proportional;
    double fortypower = .5;
    int start = 0;

    Orientation angles;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    LinearOpMode opMode;
    int countsTicksOfMotor = 560;
    int cheat = 0;
    double angleLeft = 1;

    public void MoveInch(double speed, double inches) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (4 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need
        runtime = new ElapsedTime();

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        //if the position is less than the number of inches, than it sets the motors to speed
        if (Math.abs(leftBack.getCurrentPosition()) <= ticks) {
            leftBack.setPower(speed);
            rightBack.setPower(speed * .5);
            //leftFront.setPower(speed);
            //rightFront.setPower(speed);
        }
        if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
            cheat = cheat + 1;
            runOpMode();
        }
        // leftBack.setPower(speed * 0);
        // rightBack.setPower(speed * 0);
        //leftFront.setPower(speed * 0);
        //rightFront.setPower(speed * 0);

    }

    public double Derivative(double newX, double oldX, double newY, double oldY)
    {
        slope = (newY - oldY)/(newX - oldX);
        return slope;
    }
    public double Integral(double newX, double oldX, double y)
    {
        areaUnderCurve = (newX - oldX)*(y);
        totalAreaUnderCurve = totalAreaUnderCurve + areaUnderCurve;
        return totalAreaUnderCurve;
    }
    public void runOpMode() {
        // lb.setPower();
        while (start == 0) {
            time = runTime.milliseconds();
            start = 0;
            // State used for updating telemetry
            Orientation angles;
            Acceleration gravity;

            //----------------------------------------------------------------------------------------------
            // Main logic
            //----------------------------------------------------------------------------------------------

            leftBack = hardwareMap.dcMotor.get("Left Back");
            rightBack = hardwareMap.dcMotor.get("Right Back");
            //leftFront = hardwareMap.dcMotor.get("Left Front");
            //rightFront = hardwareMap.dcMotor.get("Right Front");
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually leftback = hardwaremap.Dc
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            angles = imu.getAngularOrientation();

            telemetry.addData("cheat", cheat);
            waitForStart();

      /*  leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(560);
        rightBack.setTargetPosition(560);
        leftFront.setTargetPosition(560);
        rightFront.setTargetPosition(560);
        countsTicksOfMotor = leftBack.getCurrentPosition();


        leftBack.setPower(1);
        rightBack.setPower(1);
        leftFront.setPower(1);
        rightFront.setPower(1);*/

            idle();

        }
    }

}

