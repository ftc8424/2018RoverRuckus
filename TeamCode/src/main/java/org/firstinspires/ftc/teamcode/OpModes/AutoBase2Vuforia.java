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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.AMLChampionshipRobot;
import org.firstinspires.ftc.teamcode.Hardware.Constants;

import java.util.List;


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
public abstract class AutoBase2Vuforia extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    protected AMLChampionshipRobot robot = new AMLChampionshipRobot();
    protected double initialHeading = 0;
    protected double timeoutS = 5;
    protected double finalHeading;
    protected double deployHeading = 0;
    protected double halfHeading = 0;
    protected double lastHeading = 0;
    protected double lastFinalHeading = 0;
    protected double zeroHeading = 0;
    protected double leftSampleAngle = 0;
    protected double LiftLockPower = 0.5;
    protected double xposition = 0.0;
    protected double yposition = 0.0;

    /**
     * Initialize the robot for autonomous-specific things (e.g., set encoder mode, initialize IMU)
     */

    public void initRobot() {

        robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LiftMotor.setPower(0);
        robot.LockServo.setPosition(robot.LiftLock);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.initVuforia();



        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {

            robot.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

    }

    /**
     * The autonomous code to run when the robot is on the Depot side of the field.
     * <p>
     * This is the same code regardless of our color, because the lander and the depot are the
     * same, with the only difference being the gyro heading values, as they are 90 degrees off
     * of each other.  All other movements, encoderDrive() encoderStrafe() inches and things are
     * exactly the same.
     *
     * @throws InterruptedException
     */

    public void runDepot(boolean latched) throws InterruptedException {

        runtime.reset();
        boolean turnSuccessful = true;
        int times = 0;
        /** Activate Tensor Flow Object Detection. */
        if (robot.tfod != null) {
            robot.tfod.activate();
        }
        if (latched) {
            robot.deploy(robot.LockServo, robot.LiftUnlock);
            sleep(500);
            do {
                robot.LiftMotor.setPower(-LiftLockPower);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() >= robot.LiftUp + 10);
            robot.LiftMotor.setPower(0);

            robot.encoderDrive(this, .5, -4, -4, 3);
            do {
                robot.LiftMotor.setPower(LiftLockPower);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() < robot.LiftDown - 5);

            robot.LiftMotor.setPower(0);
            robot.encoderStrafe(this, .5, 0, 5, 3);

        } else {
            robot.encoderStrafe(this, .5, 0, 3, 2);
            robot.encoderDrive(this, .5, -4, -4, 2);
        }
        robot.encoderDrive(this, .5, 4, 4, 3);


        do {


            turnSuccessful = robot.gyroTurn(this, initialHeading, timeoutS);

            if (turnSuccessful == false) {
                telemetry.addData("TURN STATUS", "UNSUCCESSFUL, ATTEMPTING RECOVERY");
                telemetry.update();
                sleep(1000);
                double heading = robot.getHeading();
                switch (times) {
                    case 0:
                        robot.encoderStrafe(this, .25, 2, 0, 2);
                        break;
                    case 1:
                        robot.encoderStrafe(this, .25, 0, 4, 2);
                        break;
                    case 2:
                        if (heading >= 0 && heading <= 90)
                            robot.encoderDrive(this, .25, 4, 4, 2);
                        if (heading >= 91 && heading <= 180)
                            robot.encoderDrive(this, .25, -4, -4, 2);
                        if (heading >= 181 && heading <= 269)
                            robot.encoderDrive(this, .25, 4, 4, 2);
                        if (heading >= 270 && heading <= 359)
                            robot.encoderDrive(this, .25, -4, -4, 2);
                        break;
                }
            }
        }
        while (opModeIsActive() && turnSuccessful == false && times++ < 3);

        robot.encoderDrive(this, .5, 6, 6, 3);
        sleep(100);

        switch (sampleMineralsDepot()) {
            //switch (goldCenter){
            case goldNotFound:
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderDrive(this, .75, 40, 40, 4);
                telemetry.addData("Deploying Marker", "");
                telemetry.update();
                sleep(1000);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                sleep(100);
                telemetry.addData("Marker Deployed", "");
                telemetry.update();
                sleep(1000);
                if (robot.gyroTurn(this, deployHeading, 5)) {
                    telemetry.addData("Turn Successful", "");
                    telemetry.update();
                    sleep(1000);
                    robot.encoderStrafe(this, .5, 7, 0, 10);
                    telemetry.addData("Strafe Completed", "");
                    telemetry.update();
                    sleep(1000);
                    robot.encoderDrive(this, .75, -58, -58, 4);
                }
                break;


            case goldLeft:
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderDrive(this, .5, 8, 8, 3);
                robot.encoderStrafe(this, .5, 18, 0, 3);
                robot.encoderDrive(this, .5, 36, 36, 5);
                robot.gyroTurn(this, deployHeading, 3);
                robot.encoderDrive(this, .5, 6, 6, 2);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                double msecs = runtime.milliseconds();
                do {
                    robot.gyroTurn(this, deployHeading, timeoutS);
                } while (opModeIsActive() && runtime.milliseconds() < msecs + 1000);
                robot.encoderDrive(this, 1, -75, -75, 10);

                break;

            case goldCenter:
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderDrive(this, .5, 50, 50, 4);
                telemetry.addData("Deploying Marker", "");
                telemetry.update();
                sleep(1000);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                msecs = runtime.milliseconds();
                do {
                    robot.gyroTurn(this, initialHeading, timeoutS);
                } while (opModeIsActive() && runtime.milliseconds() < msecs + 1000);
                telemetry.addData("Marker Deployed", "");
                telemetry.update();
                sleep(1000);
                if (robot.gyroTurn(this, deployHeading, 5)) {
                    telemetry.addData("Turn Successful", "");
                    telemetry.update();
                    sleep(200);
                    robot.encoderStrafe(this, .5, 20, 0, 10);
                    telemetry.addData("Strafe Completed", "");
                    telemetry.update();
                    sleep(1000);
                    robot.encoderDrive(this, .75, -71, -71, 10);
                } else {
                    telemetry.addData("Turn Unsuccessful", "");
                    telemetry.update();
                    sleep(1000);
                }

                break;

            case goldRight:
                robot.gyroTurn(this, initialHeading, 5);
                robot.encoderDrive(this, .5, 10, 10, 3);
                robot.encoderStrafe(this, .5, 0, 17, 3);
                robot.encoderDrive(this, .5, 34, 34, 3);
                robot.gyroTurn(this, deployHeading, 5);
                robot.encoderStrafe(this, .5, 26, 0, 3);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                msecs = runtime.milliseconds();
                do {
                    robot.gyroTurn(this, finalHeading, timeoutS);
                } while (opModeIsActive() && runtime.milliseconds() < msecs + 1000);                robot.encoderDrive(this, 1, -77, -77, 10);

                break;
        }

        // TODO:  DEPOT-STEP-3:  NOW Drive to the Depot and deploy the marker
        if (!opModeIsActive()) {
            return;
        }


       /* robot.encoderDrive(this, 1, 45, 45, 5);
        robot.encoderStrafe(this, .37, 47, 0, 5);
        robot.gyroTurn(this, deployHeading, 5);
        robot.encoderDrive(this, 1, 36, 36, 5);
        robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
        sleep(2000);

       */

        // TODO:  DEPOT-STEP-4:  Drive to our Alliance's crater and park
        //if (!opModeIsActive())

        //  return;

        //robot.gyroTurn(this,0, 2);
        //robot.encoderDrive(this, 1, -60,-60, 10);
/*
        // MEET 1 OVERRIDE:  Don't sample, just drive out of the Depot, grab the left-most mineral and head to crater
        robot.encoderDrive(this, 0.75, -3, -3, 2);
        robot.gyroTurn(this,225,2);
        robot.encoderStrafe(this, .75, 0, 1, 2);

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

    } /* runDepot() */

    public void runCrater(boolean latched) throws InterruptedException {
        runCrater(latched, false);
    }

    /**
     * The autonomous code to run when the robot is on the Crater side of the field.
     * <p>
     * This is the same code regardless of our color, because the lander and the crater are the
     * same, with the only difference being the gyro heading values, as they are 90 degrees off
     * of each other.  All other movements, encoderDrive() encoderStrafe() inches and things are
     * exactly the same.
     *
     * @throws InterruptedException
     */

    public void runCrater(boolean latched, boolean doubleSample) throws InterruptedException {

        runtime.reset();
        boolean turnSuccessful = false;
        int times = 0;
        /** Activate Tensor Flow Object Detection. */
        if (robot.tfod != null) {
            robot.tfod.activate();
        }
        if (latched) {
            robot.deploy(robot.LockServo, robot.LiftUnlock);
            sleep(500);
            do {
                robot.LiftMotor.setPower(-LiftLockPower);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() >= robot.LiftUp + 10);
            robot.LiftMotor.setPower(0);

            robot.encoderDrive(this, .5, -4, -4, 3);
            do {
                robot.LiftMotor.setPower(.5);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() < robot.LiftDown - 5);

            robot.LiftMotor.setPower(0);

            robot.encoderStrafe(this, .5, 0, 8, 3);

        } else {
            robot.encoderStrafe(this, .5, 0, 3, 2);
            robot.encoderDrive(this, .5, -3, -3, 2);
            robot.vuforiaTesting(this);
            telemetry.update();
            sleep (10000);
        }
        robot.encoderDrive(this, .5, 6, 6, 3);

        do {
            turnSuccessful = robot.gyroTurn(this, initialHeading, timeoutS);
            if (turnSuccessful == false) {
                double heading = robot.getHeading();
                switch (times) {
                    case 0:
                        robot.encoderStrafe(this, .75, 2, 0, 2);
                        break;
                    case 1:
                        robot.encoderStrafe(this, .75, 0, 4, 2);
                        break;
                    case 2:
                        if (heading >= 0 && heading <= 90) robot.encoderDrive(this, .75, 4, 4, 2);
                        if (heading >= 91 && heading <= 180)
                            robot.encoderDrive(this, .75, -4, -4, 2);
                        if (heading >= 181 && heading <= 269)
                            robot.encoderDrive(this, .75, 4, 4, 2);
                        if (heading >= 270 && heading <= 359)
                            robot.encoderDrive(this, .75, -4, -4, 2);
                        break;
                }
            }
        }
        while (opModeIsActive() && turnSuccessful == false && times++ < 3);


        robot.encoderDrive(this, .75, 6, 6, 3);
        sleep(100);

        switch (sampleMineralsCrater()) {

            case goldNotFound:

                robot.encoderDrive(this, .75, 35, 35, 5);


                break;

            case goldLeft:
                robot.encoderDrive(this,.75,9, 9, 3);
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderStrafe(this, .5, 15, 0, 3);
                robot.encoderDrive(this, .75, 13, 13, 3);
                robot.encoderDrive(this, .5, -6, -6, 3);
                robot.encoderStrafe(this, .5, 25, 0, 4);
                robot.gyroTurn(this, finalHeading, 3);
                robot.encoderStrafe(this, .5,0,5,3);
                robot.encoderDrive(this, 1, 54, 54, 5);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                double msecs = runtime.milliseconds();
                sleep(1000);
                /*do {
                    robot.gyroTurn(this, finalHeading, timeoutS);
                } while (opModeIsActive() && runtime.milliseconds() < msecs + 1000);*/
                if (doubleSample) {
                    robot.gyroTurn(this, lastHeading, 3);
                    robot.encoderStrafe(this, .5, 17, 0, 3);
                    robot.encoderDrive(this, .75, 28, 28, 4);
                    robot.encoderStrafe(this, .75, 0, 57, 5);
                    robot.gyroTurn(this, lastFinalHeading, 3);
                    robot.encoderDrive(this, .5, 16, 16, 3);
                } else {
                    robot.encoderDrive(this, 1, -76, -76, 5);
                }
                break;

            case goldCenter:
                robot.encoderDrive(this, .25, 21, 21, 3);
                robot.encoderDrive(this, .5, -9, -9, 3);
                robot.encoderStrafe(this, .5, 40, 0, 5);
                robot.gyroTurn(this, finalHeading, 3);
                robot.encoderStrafe(this, .5, 0, 5, 3);
                robot.encoderDrive(this, 1, 52, 52, 5);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                msecs = runtime.milliseconds();
                sleep(1000);
                /*do {
                    robot.gyroTurn(this, finalHeading, timeoutS);
                } while (opModeIsActive() && runtime.milliseconds() < msecs + 1000);*/
                if (doubleSample) {
                    robot.gyroTurn(this, lastHeading, 3);
                    robot.encoderDrive(this, .5, 6, 6, 3);
                    robot.encoderDrive(this, .75, 31, 31, 5);
                    robot.encoderStrafe(this, .5, 0, 42, 5);
                    robot.gyroTurn(this, lastFinalHeading, 3);
                    robot.encoderDrive(this, .5, 16, 16, 3);
                } else {
                    robot.encoderDrive(this, 1, -76, -76, 5);
                }

                break;

            case goldRight:
                robot.encoderDrive(this, .5, 6, 6, 3);
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderStrafe(this, .5, 0, 22, 3);
                robot.encoderDrive(this, .5, 12, 12, 3);
                robot.encoderDrive(this, .5, -7, -7, 3);
                robot.encoderStrafe(this, .75, 55, 0, 6);
                robot.gyroTurn(this, finalHeading, 3);
                robot.encoderStrafe(this, .5, 0, 8, 3);
                robot.encoderDrive(this, 1, 50, 50, 5);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                msecs = runtime.milliseconds();
                sleep(1000);
                /*do {
                    robot.gyroTurn(this, finalHeading, timeoutS);
                } while (opModeIsActive() && runtime.milliseconds() < msecs + 1000);*/
                if (doubleSample) {
                    robot.gyroTurn(this, lastHeading, 3);
                    robot.encoderStrafe(this, .5, 0, 15, 3);
                    robot.encoderDrive(this, .75, 28, 28, 4);
                    robot.encoderStrafe(this, .75, 0, 27, 5);
                    robot.gyroTurn(this, lastFinalHeading, 3);
                    robot.encoderDrive(this, .5, 16, 16, 3);
                } else {
                    robot.encoderDrive(this, 1, -76, -76, 5);
                }
                break;


        }

        // TODO:  DEPOT-STEP-3:  NOW Drive to the Depot and deploy the marker
        if (!opModeIsActive()) {
            return;
        }

/*
        // MEET 1 OVERRIDE:  Don't sample, just drive out of the Depot, grab the left-most mineral and head to crater
        robot.encoderStrafe(this, .37, 35, 0, 5);
        robot.gyroTurn(this, 90, 5);
        robot.encoderDrive(this, .37, 43, 0, 5);
        robot.gyroTurn(this, 0, 5);
        robot.encoderDrive(this, .37, 52, 52, 5);
        //Deploy linear actuator for team marker
        robot.encoderDrive(this, .37, -52, -52, 5);
        robot.encoderStrafe(this, .37, 0, 34, 5);

*/

    } /* runCrater() */

    /*
     * Variables below are for the sampleMinerals() return status codes
     */
    private static final int goldLeft = 1;        // Return from sampleMineral() if gold on left
    private static final int goldCenter = 2;      // Return from sampleMineral() if gold centered
    private static final int goldRight = 3;       // Return from sampleMineral() if gold right
    private static final int goldNotFound = -1;   // Return from sampleMineral() if no gold found

    /**
     * Sample the minerals and push off Gold one, not touching the Silver ones
     *
     * @return 1 if Gold was on Left, 2 if Gold was Center
     * @throws InterruptedException
     */
    private int sampleMineralsCrater() throws InterruptedException {
        if (!opModeIsActive())
            return goldNotFound;
        int goldState = goldCenter;
        boolean goldFound = false;
        int times = 0;
        CameraDevice camera = CameraDevice.getInstance();
        camera.setFlashTorchMode(true);


        do {
            int goldMineralX = -1;
            int goldMineralY = -1;
            double confidence = -1;
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(Constants.LABEL_GOLD_MINERAL)) {
                            goldMineralX = Math.abs((int) recognition.getLeft());
                            goldMineralY = Math.abs((int) recognition.getTop());
                            confidence = recognition.getConfidence();
                            telemetry.addData("Confidence:", confidence);
                            telemetry.addData("Gold Mineral X", goldMineralX)
                                    .addData("Gold Mineral Y", goldMineralY);
                            if (confidence >= .9) {
                                if (goldState == goldCenter) {
                                    if (goldMineralY > 500) {
                                        goldFound = true;
                                        break;
                                    }
                                    //goldFound = true;
                                } else if (goldState == goldLeft) {
                                    if (goldMineralY > 500) {
                                        goldFound = true;
                                        break;
                                    }
                                    //goldFound = true;


                                }
                            }

                        }
                        telemetry.addData("gold Found", goldFound ? "TRUE": "FALSE");
                        telemetry.update();
                        sleep(250);


                    }
                }
            }
            times++;
            if (!goldFound && goldState == goldCenter && times % 3 == 0) {
                goldState = goldLeft;
                robot.gyroTurn(this, leftSampleAngle, 3);
            } else if (!goldFound && goldState == goldLeft && times % 3 == 0) {
                goldState = goldRight;
                goldFound = true;
            }
            sleep(200);
        }while (opModeIsActive() && !goldFound && goldState != goldNotFound && times < 7) ;
        camera.setFlashTorchMode(false);
        return goldState;

    }

    private int sampleMineralsDepot() throws InterruptedException {
        if (!opModeIsActive())
            return goldNotFound;
        int goldState = goldCenter;
        boolean goldFound = false;
        int times = 0;
        CameraDevice camera = CameraDevice.getInstance();
        camera.setFlashTorchMode(true);


        do {
            int goldMineralX = -1;
            int goldMineralY = -1;
            double confidence = -1;
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(Constants.LABEL_GOLD_MINERAL)) {
                            goldMineralX = Math.abs((int) recognition.getLeft());
                            goldMineralY = Math.abs((int) recognition.getTop());
                            confidence = recognition.getConfidence();
                            telemetry.addData("Confidence:", confidence);
                            telemetry.addData("Gold Mineral X", goldMineralX)
                                    .addData("Gold Mineral Y", goldMineralY);
                            if (confidence >= .9) {
                                if (goldState == goldCenter) {
                                    if (goldMineralY > 50) {
                                        goldFound = true;
                                        break;
                                    }
                                    //goldFound = true;
                                } else if (goldState == goldLeft) {
                                    if (goldMineralY > 50) {
                                        goldFound = true;
                                        break;
                                    }
                                    //goldFound = true;


                                }
                            }

                        }
                        telemetry.addData("gold Found", goldFound ? "TRUE": "FALSE");
                        telemetry.update();
                        sleep(250);


                    }
                }
            }
            times++;
            if (!goldFound && goldState == goldCenter && times % 3 == 0) {
                goldState = goldLeft;
                robot.gyroTurn(this, leftSampleAngle, 1.5);
            } else if (!goldFound && goldState == goldLeft && times % 3 == 0) {
                goldState = goldRight;
                goldFound = true;
            }
            sleep(200);
        }while (opModeIsActive() && !goldFound && goldState != goldNotFound && times < 7) ;
        camera.setFlashTorchMode(false);
        return goldState;

    }
    public boolean VuforiaPosition(double xPosition, double yPosition, double TimeOutSeconds) throws InterruptedException {
        do {
            robot.translation.get(0);
            robot.translation.get(1);

            /*if (initialHeading == 45) {

                if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);


                } else if (robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            } else if (initialHeading == 130) {
                if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(1) > 00 && robot.translation.get(0) == 000) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) < 00 && robot.translation.get(0) == 000) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);

                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            } else if (initialHeading == 225) {
                if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);


                } else if (robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            } else if (initialHeading == 310) {
                if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(1) > 00 && robot.translation.get(0) == 000) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) < 00 && robot.translation.get(0) == 000) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);

                } else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            }*/
            if (initialHeading == 45) {

                if (robot.translation.get(0) < xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);


                } else if (robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 0, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            } else if (initialHeading == 130) {
                if (robot.translation.get(0) < xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(1) > yPosition && robot.translation.get(0) == xPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) < yPosition && robot.translation.get(0) == xPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);

                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            } else if (initialHeading == 225) {
                if (robot.translation.get(0) > xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);


                } else if (robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 180, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            } else if (initialHeading == 310) {
                if (robot.translation.get(0) > xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderDrive(this, .25, .5, .5, 3);
                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) > yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) < yPosition) {
                    telemetry.update();
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                } else if (robot.translation.get(1) > yPosition&& robot.translation.get(0) == xPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, -.5, -.5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(1) < yPosition&& robot.translation.get(0) == xPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderDrive(this, .25, .5, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);

                } else if (robot.translation.get(0) > xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, .5, 0, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                } else if (robot.translation.get(0) < xPosition && robot.translation.get(1) == yPosition) {
                    telemetry.update();
                    robot.gyroTurn(this, 90, 3);
                    robot.encoderStrafe(this, .25, 0, .5, 3);
                    robot.gyroTurn(this, initialHeading, 3);
                }
            }
            robot.vuforiaUpdateLocation();
        } while (opModeIsActive() && robot.translation.get(0) != xPosition && robot.translation.get(1) != yPosition && runtime.milliseconds() < TimeOutSeconds);

        return true;
    }

    /**
     * Use Vuforia VuMarks to drive to a specific position on the field, assuming no disruptions
     * or other things on the field blocking the way.  Note that the heading of the robot might
     * change during this maneuver, so if a specific heading is required after the movement is
     * over, the caller must manually set the heading after calling vuDriveTo().
     *
     * @param xposition   The xposition of the Vuforia navigation we should hit
     * @param yposition   The yposition of the Vuforia navigation we should hit
     * @param timeoutS    The number of seconds we should continue trying to hit it
     * @return            True if we hit our mark (within tolerances), False if we didn't
     * @throws InterruptedException
     */

    public boolean vuDriveTo (double xposition, double yposition, int timeoutS) throws InterruptedException {
        double curHeading = robot.getHeading();
        boolean moveComplete = false;
        double curXpos, curYpos, deltaX, deltaY;
        double[] wheelPower = {0.0, 0.0, 0.0, 0.0};

        if (!opModeIsActive())
            return false;

        // The first thing to do is get access to the VuMark, in case it's not displayed
        if ( !robot.targetVisible ) {
            double setHeading = 0.0;

            if ( curHeading > 315 || curHeading <= 45 ) {
                setHeading = 345;
            } else if ( curHeading > 235 ) {
                setHeading = 255;
            } else if ( curHeading > 180 ) {
                setHeading = 165;
            } else if ( curHeading > 135 ) {
                setHeading = 75;
            }
            robot.gyroTurn(this, setHeading, timeoutS > 3 ? 3 : timeoutS ); // Don't care if doesn't work
        }
        if (!opModeIsActive())
                return false;

        do {
            curHeading = robot.getHeading();        // Get each time, in case moved/jostled
            robot.vuforiaUpdateLocation();          // Update the vuMark visibility and location
            telemetry.addData("VuDriveTo", "VuMark Visible:  %s", robot.targetVisible ? "TRUE" : "FALSE");
            curXpos = robot.translation.get(0);
            curYpos = robot.translation.get(1);
            deltaX = Math.abs(curXpos - xposition);
            deltaY = Math.abs(curYpos - yposition);
            if ( curXpos < xposition*.95 ) {
                if (curHeading <= 80 && curHeading >= 70) {
                    robot.encoderStrafe(this, .25, 0, deltaX, 3);
                }
                else if (curHeading <= 170 && curHeading >= 160) {
                    robot.encoderDrive(this,.25, deltaX, deltaX,3);
                }
                else if (curHeading <= 260 && curHeading >= 250){
                    robot.encoderStrafe(this, .25, deltaX, 0, 2);
                }
                else if (curHeading <= 350 && curHeading >= 340){
                    robot.encoderDrive(this,.25, -deltaX, -deltaX,3);
                }
                // Need to move AWAY from the audience
                // TODO: Use curHeading to decide if need to strafe right, left or move forward or backward

            } else if ( curXpos > xposition*.95 ) {
                if (curHeading <= 95 && curHeading >= 85) {
                    robot.encoderStrafe(this, .25, deltaX, 0, 3);
                }
                else if (curHeading <= 185 && curHeading >= 175) {
                    robot.encoderDrive(this, .25, -deltaX, -deltaX, 3);
                }
                else if (curHeading <= 275 && curHeading >= 265){
                    robot.encoderStrafe(this, .25, 0, deltaX, 3);
                }
                else if (curHeading <= 5 && curHeading >= 355){
                    robot.encoderDrive(this, .25, deltaX, deltaX, 3);
                }

                // Need to move TOWARDS the audience
                // TODO:  Use curHeading to decide if need to strafe right, left or move forward or backward

            } else if ( curYpos < yposition*.95 ) {
                if (curHeading <= 95 && curHeading >= 85) {
                    robot.encoderDrive(this,.25, deltaY, deltaY,3);
                }
                else if (curHeading <= 185 && curHeading >= 175) {
                    robot.encoderStrafe(this,.25, deltaY,0,3);
                }
                else if (curHeading <= 275 && curHeading >= 265){
                    robot.encoderDrive(this,.25, -deltaY, -deltaY,3);
                }
                else if (curHeading <= 5 && curHeading >= 355){
                    robot.encoderStrafe(this,.25, -deltaY,0,3);
                }

                    // Need to move AWAY from Red Alliance station
                // TODO:  Use curHeading to decide if need to strafe right, left or move forward or backward

            } else if ( curYpos > yposition*.95 ) {
                if (curHeading <= 95 && curHeading >= 85) {
                    robot.encoderDrive(this,.25, -deltaY, -deltaY,3);
                }
                else if (curHeading <= 185 || curHeading >= 175) {
                    robot.encoderStrafe(this,.25,0, deltaY,3);
                }
                else if (curHeading <= 275 && curHeading >= 265){
                    robot.encoderDrive(this,.25, deltaY, deltaY,3);
                }
                else if (curHeading <= 5 && curHeading >= 355){
                    robot.encoderStrafe(this,.25,deltaY, 0,3);

                }


                    // Need to move towards Red Alliance station
                // TODO:  Use curHeading to decide if need to strafe right, left or move forward or backward
            } else {                                    // Need to STOP all movement, we're done
                wheelPower[0] = 0.0; wheelPower[1] = 0.0;
                wheelPower[2] = 0.0; wheelPower[2] = 0.0;
                moveComplete = true;
            }

            robot.LFront.setPower(wheelPower[0]);
            robot.RFront.setPower(wheelPower[1]);
            robot.LBack.setPower(wheelPower[2]);
            robot.RBack.setPower(wheelPower[3]);
            
            telemetry.update();                     // Print out all the telemetry we did in loop.
        } while (opModeIsActive() && !moveComplete && runtime.seconds() < timeoutS);
        return moveComplete;
    }

    /**
     * Stop the robot and turn off the picture display to save battery.
     */
    public void stopRobot() {
        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
    }

}




