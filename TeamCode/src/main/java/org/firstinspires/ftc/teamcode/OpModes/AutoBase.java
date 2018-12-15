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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Meet2Robot;

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
public abstract class AutoBase extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    protected Meet2Robot robot = new Meet2Robot();
    protected double initialHeading = 0;
    protected double timeoutS = 5;
    protected double finalHeading;
    protected double deployHeading = 0;
    protected double halfHeading = 0;
    protected double lastHeading = 0;
    protected double lastFinalHeading = 0;
    protected double zeroHeading = 0;
    protected double leftSampleAngle = 0;

    /**
     * Initialize the robot for autonomous-specific things (e.g., set encoder mode, initialize IMU)
     */

    public void initRobot() {

        robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LiftMotor.setPower(0);
        robot.LockServo.setPosition(robot.LiftLock);
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
     *
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
                robot.LiftMotor.setPower(-.25);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() >= robot.LiftUp + 10);
            robot.LiftMotor.setPower(0);

            robot.encoderDrive(this, .5, -4, -4, 3);
            do {
                robot.LiftMotor.setPower(.5);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() < robot.LiftDown - 5 );

            robot.LiftMotor.setPower(0);
            robot.encoderStrafe(this, .5, 0, 5, 3);

        }else {
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

        robot.encoderDrive(this, .75, 6, 6, 3);
        sleep(100);

        switch (sampleMinerals()) {
        //switch (goldCenter){
            case goldNotFound:
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderDrive(this, .75, 40,40, 4);
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
                robot.encoderDrive(this, .5, 5, 5, 3);
                robot.encoderStrafe(this, .5, 15, 0, 3);
                robot.encoderDrive(this, .75, 36,36, 5);
                robot.gyroTurn(this, deployHeading, 3);
                robot.encoderDrive(this, .5, 6,6, 2);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                robot.encoderStrafe(this, .5, 4, 0, 3);
                robot.encoderDrive(this, 1, -60, -60, 10);

                break;

            case goldCenter:
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderDrive(this, .5, 40,40, 4);
                telemetry.addData("Deploying Marker", "");
                telemetry.update();
                sleep(1000);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                sleep(100);
                telemetry.addData("Marker Deployed", "");
                telemetry.update();
                sleep(1000);
                if (robot.gyroTurn(this, deployHeading, 5)){
                    telemetry.addData("Turn Successful", "");
                    telemetry.update();
                    sleep(1000);
                    robot.encoderStrafe(this, .5, 8, 0, 10);
                    telemetry.addData("Strafe Completed", "");
                    telemetry.update();
                    sleep(1000);
                    robot.encoderDrive(this, .75, -60, -60, 10);
                }else {
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
                sleep(500);
                robot.encoderDrive(this, 1, -77, -77, 10);

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

    public void runCrater (boolean latched) throws InterruptedException{
        runCrater(latched, false);
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
                robot.LiftMotor.setPower(-.25);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() >= robot.LiftUp + 10);
            robot.LiftMotor.setPower(0);

            robot.encoderDrive(this, .5, -4, -4, 3);
            do {
                robot.LiftMotor.setPower(.5);

            } while (opModeIsActive() && robot.LiftMotor.getCurrentPosition() < robot.LiftDown - 5 );

            robot.LiftMotor.setPower(0);
            robot.encoderStrafe(this, .25, 0, 5, 3);

        }else {
            robot.encoderStrafe(this, .5, 0, 3, 2);
            robot.encoderDrive(this, .5, -2, -2, 2);
        }
        robot.encoderDrive(this, .5, 4, 4, 3);

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


        robot.encoderDrive(this, .75, 6, 6, 3);
        sleep(100);

        switch (sampleMinerals()) {

            case goldNotFound:

                robot.encoderDrive(this, .75, 35,35, 5);


                break;

            case goldLeft:
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderStrafe(this, .5, 15, 0, 3);
                robot.encoderDrive(this, .75, 11, 11, 3);
                robot.encoderDrive(this, .5, -6, -6, 3);
                robot.encoderStrafe(this, .5, 25, 0, 4);
                robot.gyroTurn(this, finalHeading, 3);
                robot.encoderDrive(this, 1, 60, 65, 5);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                if (doubleSample){
                    robot.gyroTurn(this, lastHeading, 3 );
                    robot.encoderStrafe(this, .5, 15, 0, 3);
                    robot.encoderDrive(this, .75, 28, 28, 4);
                    robot.encoderStrafe(this, .75, 0, 57, 5);
                    robot.gyroTurn(this, lastFinalHeading, 3);
                    robot.encoderDrive(this, .5, 16, 16, 3);
                }
                else {
                    robot.encoderDrive(this, 1, -96, -96, 5);
                }
                break;

            case goldCenter:
                robot.encoderDrive(this, .75, 11, 11, 3);
                robot.encoderDrive(this, .5, -6, -6, 3);
                robot.encoderStrafe(this, .5, 40, 0, 5);
                robot.gyroTurn(this, finalHeading, 3);
                robot.encoderStrafe(this, .5, 0, 3, 3);
                robot.encoderDrive(this, 1, 52, 52, 5);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                if (doubleSample){
                    robot.gyroTurn(this, lastHeading, 3);
                    robot.encoderDrive(this, .5, 6, 6, 3);
                    robot.encoderDrive(this, .75, 31, 31, 5);
                    robot.encoderStrafe(this, .5, 0, 42, 5);
                    robot.gyroTurn(this, lastFinalHeading, 3);
                    robot.encoderDrive(this, .5, 16, 16, 3);
                } else {
                    robot.encoderDrive(this, 1, -96, -96, 5);
                }

                break;

            case goldRight:
                robot.encoderDrive(this, .5, 6, 6, 3);
                robot.gyroTurn(this, initialHeading, 3);
                robot.encoderStrafe(this, .5, 0,15, 3);
                robot.encoderDrive(this, .5, 11,11, 3);
                robot.encoderDrive(this, .5, -10, -10, 3);
                robot.encoderStrafe(this, .5, 60, 0, 5);
                robot.gyroTurn(this,finalHeading, 3);
                robot.encoderDrive(this, 1, 65, 65, 5);
                robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
                if(doubleSample){
                    robot.gyroTurn(this, lastHeading, 3 );
                    robot.encoderStrafe(this, .5, 0, 15, 3);
                    robot.encoderDrive(this, .75, 28, 28, 4);
                    robot.encoderStrafe(this, .75, 0, 27, 5);
                    robot.gyroTurn(this, lastFinalHeading, 3);
                    robot.encoderDrive(this, .5, 16, 16, 3);
                } else {
                    robot.encoderDrive(this, 1, -96, -96, 5);
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
     * @return   1 if Gold was on Left, 2 if Gold was Center
     *
     * @throws InterruptedException
     */
    private int sampleMinerals() throws InterruptedException {
        if ( !opModeIsActive() )
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
                            telemetry.update();
                            sleep(500);
                            if (confidence >= .8){
                                if (goldState == goldCenter){
                                    if (goldMineralX > 190 && goldMineralX < 350 && goldMineralY > 720){
                                        goldFound = true;
                                        break;
                                    }
                                }else if (goldState == goldLeft){
                                    if (goldMineralX > 200 && goldMineralX < 370 && goldMineralY > 660){
                                        goldFound = true;                                    }
                                        break;
                                }
                            }

                        }
                    }

                     if (!goldFound && goldState == goldCenter) {
                        robot.gyroTurn(this, leftSampleAngle, 3);
                        goldState = goldLeft;
                    } else if (!goldFound && goldState == goldLeft){
                        //robot.encoderDrive(this,.75,4,-4,5);
                        goldState = goldRight;
                        goldFound = true;

                    } else {
                        goldState = goldNotFound;
                    }
                } else {
                    robot.encoderDrive(this,.75,1,1,3);
                    times++;
                }
            }  else {
                robot.encoderDrive(this,.75,1,1,3);
                times++;
            }
        } while (opModeIsActive() && !goldFound && goldState != goldNotFound && times < 4);
        camera.setFlashTorchMode(false);
        return goldState;
    }

}
