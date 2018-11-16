/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Meet1Robot;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Depot", group="Linear Opmode")
@Disabled
public abstract class AutoBase extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    protected Meet1Robot robot = new Meet1Robot();
    protected double initialHeading = 0;
    protected double timeoutS = 5;
    protected double finalHeading;
    protected double deployHeading = 0;
    protected double halfHeading = 0;
    protected double lastHeading = 0;
    protected double lastFinalHeading = 0;

    /**
     * Initialize the robot for autonomous-specific things (e.g., set encoder mode, initialize IMU)
     */

    public void initRobot() {

        robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LiftMotor.setPower(0);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //imu2 = hwMap.get(BNO055IMU.class, Constants.IMU2);
        robot.imu.initialize(parameters);
        //imu2.initialize(parameters);    // Use the same params, note will need different if orientation
    }

    /**
     * The autonomous code to run when the robot is on the Depot side of the field.
     *
     * This is the same code regardless of our color, because the lander and the depot are the
     * same, with the only difference being the gyro heading values, as they are 90 degrees off
     * of each other.  All other movements, encoderDrive() encoderStrafe() inches and things are
     * exactly the same.
     *
     * @throws InterruptedException
     */

    public void runDepot() throws InterruptedException {

        runtime.reset();
        boolean turnSuccessful = true;
        int times = 0;

        do {
            turnSuccessful = robot.gyroTurn(this, initialHeading, timeoutS);
            if (turnSuccessful == false) {
                telemetry.addData("TURN STATUS", "UNSUCCESSFUL, ATTEMPTING RECOVERY")
                        .addData("Servo Position", robot.ColorServo.getPosition());
                telemetry.update();
                sleep(1000);
                double heading = robot.getHeading();
                switch (times) {
                    case 0:
                        robot.encoderStrafe(this,.75, 2, 0, 2);
                        break;
                    case 1:
                        robot.encoderStrafe(this, .75, 0, 4, 2);
                        break;
                    case 2:
                        if (heading >= 0 && heading <= 90) robot.encoderDrive(this, .75, 4, 4, 2);
                        if (heading >= 91 && heading <= 180) robot.encoderDrive(this, .75, -4, -4, 2);
                        if (heading >= 181 && heading <= 269) robot.encoderDrive(this, .75, 4, 4, 2);
                        if (heading >= 270 && heading <= 359) robot.encoderDrive(this, .75, -4, -4, 2);
                        break;
                }
            }
        }
        while (opModeIsActive() && turnSuccessful == false && times++ < 3);


        robot.encoderDrive(this, 1, 45, 45, 5);
        robot.encoderStrafe(this, .37, 47, 0, 5);
        robot.gyroTurn(this, deployHeading, 5);
        robot.encoderDrive(this, 1, 36, 36, 5);
        robot.deploy(robot.MarkerServo, robot.ColorDeploy);
        sleep(2000);

        // TODO: Rework all of the sampling below to do the sample from the depot rather than coming all the way back out again

        robot.encoderDrive(this, 0.75, -3, -3, 2);
        robot.gyroTurn(this,225,2);
        robot.encoderStrafe(this, .75, 0, 1, 2);


/*
        robot.encoderDrive(this, 1, -24, -24, 5);
        robot.gyroTurn(this, halfHeading, 3);
        robot.encoderDrive(this, .75, 24, 25, 5);
        robot.gyroTurn(this, lastHeading, 3);
        robot.encoderDrive(this, .75, 65, 60, 2);
        robot.gyroTurn(this, lastFinalHeading, 3);
        robot.encoderDrive(this, .5, 15, 10, 4);
        robot.gyroTurn(this, halfHeading, 3);
        robot.encoderDrive(this, .75, 10, 10, 3);
        robot.gyroTurn(this, 45, 3);
        robot.encoderDrive(this, .3, 6, 6, 2);
        robot.encoderStrafe(this, .37, 0 , 24, 5);
*/

        // TODO: Need to put a check to make sure the OpMode is still active in EVERY if condition

        if (turnSuccessful) {
            robot.deploy(robot.ColorServo, robot.ColorSample);
            // robot.LiftMotor.setTargetPosition(0);

            // TODO: Put some comments in here before the different clauses so we know what the robot is supposed to be doing.  Helps with troubleshooting.

            /*robot.encoderDrive(this, .75, 8, 8, 5);
            robot.encoderStrafe(this, 0.75, 0, 15, 4);
            robot.encoderDrive(this, 0.75, 4, 4, 2); */

            telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                    .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                    .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                    .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition())
                    .addData("is it gold", robot.isGold());
            telemetry.update();
            sleep(1000);

            if (robot.isGold() == false) {

                robot.encoderDrive(this, .75, -4, -4, 2);
                robot.encoderStrafe(this, 0.75, 17, 0, 10);
                robot.encoderDrive(this, .75, 4, 4, 2);



                telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                        .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                        .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                        .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                telemetry.addData("is it gold", robot.isGold());
                telemetry.addData("blueValue", robot.color.blue());
                telemetry.addData("redValue", robot.color.red());
                telemetry.addData("greenValue", robot.color.green());
                telemetry.update();
                sleep(1000);
                if (robot.isGold() == false) {
                    robot.encoderDrive(this, .75, -4, -4, 2);
                    robot.encoderStrafe(this, 0.75, 17, 0, 10);
                    robot.encoderDrive(this, .75, 4, 4, 2);                    telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                            .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                            .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                            .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                    telemetry.addData("is it gold", robot.isGold());
                    telemetry.addData("blueValue", robot.color.blue());
                    telemetry.addData("redValue", robot.color.red());
                    telemetry.addData("greenValue", robot.color.green());
                    telemetry.update();
                    sleep(1000);
                    if (robot.isGold() == true) {
                        robot.ColorServo.setPosition(robot.ColorDeploy);
                        sleep(1500);
                        robot.ColorServo.setPosition(robot.ColorStart);
                        telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                                .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                                .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                                .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                        telemetry.addData("is it gold", robot.isGold());
                        telemetry.addData("blueValue", robot.color.blue());
                        telemetry.addData("redValue", robot.color.red());
                        telemetry.addData("greenValue", robot.color.green());
                        telemetry.update();
                        sleep(1000);
                        //robot.encoderDrive(this, .75, -5, -5, 1);
                        robot.encoderStrafe(this, .75, 33, 0, 10);
                        robot.gyroTurn(this, finalHeading, 4);
                        robot.encoderDrive(this, .75, -20,-20, 10);
                    } else {
                        robot.deploy(robot.ColorServo, robot.ColorDeploy);
                        sleep(100);
                        robot.deploy(robot.ColorServo, robot.ColorStart);
                        telemetry.addData("IS NOT GOLD", "EXITING");
                        telemetry.update();
                        //sleep(2000);
                        //robot.encoderDrive(this, .75, -5, -5, 1);
                        robot.encoderStrafe(this, .75, 33, 0, 10);
                        robot.gyroTurn(this, finalHeading, 4);
                        robot.encoderDrive(this, .75, -20,-20, 10);
                    }
                } else {

                    robot.deploy(robot.ColorServo, robot.ColorDeploy);
                    sleep(100);
                    robot.deploy(robot.ColorServo, robot.ColorStart);
                    robot.encoderStrafe(this, .75, 50, 0, 10);


                    robot.gyroTurn(this, finalHeading, 4);
                    robot.encoderDrive(this, .75, -20,-20, 10);
                }
            } else {

                robot.deploy(robot.ColorServo, robot.ColorDeploy);
                sleep(100);
                robot.deploy(robot.ColorServo, robot.ColorStart);
                robot.encoderStrafe(this, .75, 46, 0, 10);



                robot.gyroTurn(this, finalHeading, 4);
                robot.encoderDrive(this, .75, -20,-20, 10);
            }
        }
        robot.gyroTurn(this,0, 2);
        robot.encoderDrive(this, 1, -60,-60, 10);

    }


    /**
     * The autonomous code to run when the robot is on the Crater side of the field.
     *
     * This is the same code regardless of our color, because the lander and the crater are the
     * same, with the only difference being the gyro heading values, as they are 90 degrees off
     * of each other.  All other movements, encoderDrive() encoderStrafe() inches and things are
     * exactly the same.
     *
     * @throws InterruptedException
     */

    public void runCrater() throws InterruptedException {

        runtime.reset();
        boolean turnSuccessful = false;
        int times = 0;
        robot.encoderStrafe(this, .75, 0, 10, 5);
        do {
            turnSuccessful = robot.gyroTurn(this, initialHeading, timeoutS);
            if (turnSuccessful == false) {
                double heading = robot.getHeading();
                switch (times) {
                    case 0:
                        robot.encoderStrafe(this,.75, 2, 0, 2);
                        break;
                    case 1:
                        robot.encoderStrafe(this, .75, 0, 4, 2);
                        break;
                    case 2:
                        if (heading >= 0 && heading <= 90) robot.encoderDrive(this, .75, 4, 4, 2);
                        if (heading >= 91 && heading <= 180) robot.encoderDrive(this, .75, -4, -4, 2);
                        if (heading >= 181 && heading <= 269) robot.encoderDrive(this, .75, 4, 4, 2);
                        if (heading >= 270 && heading <= 359) robot.encoderDrive(this, .75, -4, -4, 2);
                        break;
                }
            }


        }
        while (opModeIsActive() && turnSuccessful == false && times++ < 3);

        robot.encoderStrafe(this, .37, 35, 0, 5);
        robot.gyroTurn(this, 90, 5);
        robot.encoderDrive(this, .37, 43, 0, 5);
        robot.gyroTurn(this, 0, 5);
        robot.encoderDrive(this, .37, 52, 52, 5);
        //Deploy linear actuator for team marker
        robot.encoderDrive(this, .37, -52, -52, 5);
        robot.encoderStrafe(this, .37, 0, 34, 5);

        // TODO: Need to put a check to make sure the OpMode is still active in EVERY if condition

        if (turnSuccessful) {
            robot.deploy(robot.ColorServo, robot.ColorSample);
            robot.encoderDrive(this, .75, 8, 8, 2);
            robot.encoderStrafe(this, 0.75, 15, 0, 4);
            robot.encoderDrive(this, 0.75, 6, 6, 2);


            telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                    .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                    .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                    .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
            telemetry.addData("is it gold", robot.isGold());
            telemetry.addData("blueValue", robot.color.blue());
            telemetry.addData("redValue", robot.color.red());
            telemetry.addData("greenValue", robot.color.green());
            telemetry.update();


            if (robot.isGold() == false) {
                robot.encoderStrafe(this, .75, 0, 17, 10);
                telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                        .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                        .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                        .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                telemetry.addData("is it gold", robot.isGold());
                telemetry.addData("blueValue", robot.color.blue());
                telemetry.addData("redValue", robot.color.red());
                telemetry.addData("greenValue", robot.color.green());
                telemetry.update();
                if (robot.isGold() == false) {
                    robot.encoderStrafe(this, .75, 0, 17, 10);
                    telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                            .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                            .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                            .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                    telemetry.update();
                    if (robot.isGold() == true) {
                        robot.deploy(robot.ColorServo, robot.ColorDeploy);
                        sleep(100);
                        robot.deploy(robot.ColorServo, robot.ColorStart);
                        telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                                .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                                .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                                .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                        telemetry.update();
                        robot.encoderStrafe(this, .75, 0, 10, 2);
                        robot.encoderDrive(this, .75, 10, 10, 3);
                    } else {

                        robot.deploy(robot.ColorServo, robot.ColorDeploy);
                        sleep(100);
                        robot.deploy(robot.ColorServo, robot.ColorStart);
                        robot.encoderStrafe(this, .75, 0, 10, 2);
                        robot.encoderDrive(this, .75, 10, 10, 3);

                        telemetry.addData("IS NOT GOLD", "EXITING");
                        telemetry.update();

                    }
                } else {

                    robot.deploy(robot.ColorServo, robot.ColorDeploy);
                    sleep(100);
                    robot.deploy(robot.ColorServo, robot.ColorStart);
                    robot.encoderDrive(this, 0.75, -5, -5, 10);
                    robot.encoderStrafe(this, .75, 0, 27, 3);
                    robot.encoderDrive(this, .75, 10, 10, 3);
                }
            } else {
                robot.deploy(robot.ColorServo, robot.ColorDeploy);
                sleep(100);
                robot.deploy(robot.ColorServo, robot.ColorStart);
                robot.encoderDrive(this, 0.75, -5, -5, 10);
                robot.encoderStrafe(this, .75, 0, 44, 3);
                robot.encoderDrive(this, .75, 10, 10, 3);


                }
            } else {
                robot.deploy(robot.ColorServo, robot.ColorDeploy);
                sleep(100);
                robot.deploy(robot.ColorServo, robot.ColorStart);
                robot.encoderDrive(this, .75, -5, -5, 10);
                robot.encoderStrafe(this, .75, 0, 44, 3);
                robot.encoderDrive(this, .75 , 10, 10, 3);

            }



    }
}
