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

import org.firstinspires.ftc.teamcode.Hardware.Meet2Robot;


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
            robot.deploy(robot.LockServo, robot.LiftUnlock);
            do {
                robot.LiftMotor.setPower(-.25);
                sleep(50);
                break;
            } while (robot.LiftMotor.getCurrentPosition() != robot.LiftUp);

            sleep(1000);
            robot.encoderDrive(this, .5, -4, -4, 3);
            do {
                robot.LiftMotor.setPower(.5);
                break;
            } while (robot.LiftMotor.getCurrentPosition() != robot.LiftDown);

            robot.encoderStrafe(this, .25, 0, 5, 3);
            robot.encoderDrive(this, .5, 4, 4, 3);
            sleep(1000);
            turnSuccessful = robot.gyroTurn(this, initialHeading, timeoutS);
            robot.encoderDrive(this, .75, 6,6, 3);
            sleep(100);

            if (turnSuccessful == false) {
                telemetry.addData("TURN STATUS", "UNSUCCESSFUL, ATTEMPTING RECOVERY");
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


        switch (sampleMinerals()) {
            case goldNotFound:

               robot.encoderDrive(this, .75, 35,35, 5);


                break;

            case goldLeft:
                robot.encoderStrafe(this, .5, 15, 15, 3);
                robot.encoderDrive(this, .75, 15,15, 5);
                robot.gyroTurn(this, 90, 3);
                robot.encoderDrive(this, .75, 12, 12, 3);


                break;

            case goldCenter:

                robot.encoderDrive(this, .75, 36,36, 4);


                break;

            case goldRight:

                robot.encoderStrafe(this, .5, 0,15, 3);
                robot.encoderDrive(this, .5, 20,20, 3);


                break;
        }

        // TODO:  DEPOT-STEP-3:  NOW Drive to the Depot and deploy the marker
        if (!opModeIsActive())
            robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
            sleep(100);
            return;

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

        // TODO: CRATER-STEP-1:  Add the code for unlocking the lift and then dropping the robot to the floor and unlatching from lander
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

        // TODO: CRATER-STEP-2: Bring lift back down, position for sampling the minerals

        switch (sampleMinerals()) {
            case goldNotFound:
                // TODO: CRATER-STEP-2.5:  No gold, so positioned still under lander, move to depot
                break;

            case goldLeft:
                // TODO:  CRATER-STEP-2.5:  Gold on left, so positioned on left, move to depot
                break;

            case goldCenter:
                // TODO:  CRATER-STEP-2.5:  Gold at center, so positioned on center, move to depot
                break;

            case goldRight:
                // TODO:  CRATER-STEP-2.5:  Gold on right, so positioned on right, move to depot
                break;
        }

        // TODO:  CRATER-STEP-3:  NOW Drive to the Depot to deploy the Marker
        if (!opModeIsActive())
            return;
        robot.encoderDrive(this, 1, 45, 45, 5);
        robot.encoderStrafe(this, .37, 47, 0, 5);
        robot.gyroTurn(this, deployHeading, 5);
        robot.encoderDrive(this, 1, 36, 36, 5);
        robot.deploy(robot.MarkerServo, robot.MarkerDeploy);
        sleep(2000);

        // TODO:  CRATER-STEP-4:  Drive to the other Alliance crater and park
        if (!opModeIsActive())
            return;
        robot.gyroTurn(this,0, 2);
        robot.encoderDrive(this, 1, -60,-60, 10);

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
        /*if ( !opModeIsActive() )
            return goldNotFound;

        /*robot.encoderDrive(this, .75, 8, 8, 5);
        robot.encoderStrafe(this, 0.75, 0, 15, 4);
        robot.encoderDrive(this, 0.75, 4, 4, 2);

        telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition())
                .addData("is it gold", robot.isGold());
        telemetry.update();
        sleep(1000);*/

        // TODO: Fix this so that you just turn the robot to the left/right (e.g., encoderDrive with -left, +right to turn left and +left, -right to turn right
        // TODO- instead of doing strafing, to determine which of the three places the gold is, and then adjust movements to just push forward to push it off
        /* if (opModeIsActive() && robot.isGold() == false) {  // GOLD isn't in center place, position to left
            robot.gyroTurn(this, 0, timeoutS);
            /*robot.encoderDrive(this, .75, -4, -4, 2);
            robot.encoderStrafe(this, 0.75, 17, 0, 10);
            robot.encoderDrive(this, .75, 4, 4, 2);
            telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                    .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                    .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                    .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
            telemetry.addData("is it gold", robot.isGold());
            telemetry.update();
            sleep(1000);
            if (opModeIsActive() && robot.isGold() == false) { // GOLD isn't in center place, position to right
                robot.encoderDrive(this, .75, -4, -4, 2);
                robot.encoderStrafe(this, 0.75, 17, 0, 10);
                robot.encoderDrive(this, .75, 4, 4, 2);                    telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                        .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                        .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                        .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                telemetry.addData("is it gold", robot.isGold());
                telemetry.update();
                sleep(1000);
                if (opModeIsActive() && robot.isGold() == true) { // GOLD is here in right place
                    telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                            .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                            .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                            .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                    telemetry.addData("is it gold", robot.isGold());
                    telemetry.update();
                    sleep(1000);
                    //robot.encoderDrive(this, .75, -5, -5, 1);
                    robot.encoderStrafe(this, .75, 33, 0, 10);
                    robot.gyroTurn(this, finalHeading, 4);
                    robot.encoderDrive(this, .75, -20,-20, 10);
                    return goldRight;
                } else { // GOLD isn't in right place either, give up!
                    telemetry.addData("IS NOT GOLD", "EXITING");
                    telemetry.update();
                    sleep(2000);
                    return goldNotFound;
                }
            } else {   // GOLD is in the center place!
                robot.encoderStrafe(this, .75, 50, 0, 10);
                robot.gyroTurn(this, finalHeading, 4);
                robot.encoderDrive(this, .75, -20,-20, 10);
                return goldCenter;
            }
        } else { // GOLD is in the left place!
            robot.encoderStrafe(this, .75, 46, 0, 10);
            robot.gyroTurn(this, finalHeading, 4);
            robot.encoderDrive(this, .75, -20,-20, 10);
            return goldLeft; */
        return goldCenter;
    }

}
