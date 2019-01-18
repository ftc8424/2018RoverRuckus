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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public abstract class   AutoBase extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    protected Meet1Robot robot = new Meet1Robot();
    protected double initialHeading = 0;
    protected double timeoutS = 5;
    protected double finalHeading;
     
    public void runDepot() throws InterruptedException {

        runtime.reset();
        boolean turnSuccessful = false;
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
        
        if (turnSuccessful) {
            robot.ColorServo.setPosition(robot.ColorSample);
            robot.LiftMotor.setTargetPosition(0);
            telemetry.addData("TURN STATUS", "SUCCESSFUL!  Scanning with Color Sensor")
                    .addData("Servo Position", robot.ColorServo.getPosition());
            telemetry.update();
            sleep(2000);
            robot.encoderDrive(this, .75, 18, 18, 5);
            robot.encoderStrafe(this, 0.75, 15, 0, 4);
            robot.encoderDrive(this, 0.75, 4, 6, 2);
            telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                    .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                    .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                    .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition())
                    .addData("is it gold", robot.isGold());
            telemetry.update();
            sleep(1000);

            if (robot.isGold() == false) {
                robot.encoderStrafe(this, 0.75, 0, 17, 10);
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
                    robot.encoderStrafe(this, 0.75, 0, 17, 10);  // TODO Strafe right 15 inches
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
                        robot.encoderStrafe(this, .75, 0, 33, 10);
                        robot.gyroTurn(this, finalHeading, 4);
                        robot.encoderDrive(this, .75, -20,-20, 10);
                    } else {
                        robot.ColorServo.setPosition(robot.ColorStart);
                        telemetry.addData("IS NOT GOLD", "EXITING");
                        telemetry.update();
                        sleep(2000);
                        //robot.encoderDrive(this, .75, -5, -5, 1);
                        robot.encoderStrafe(this, .75, 0, 33, 10);
                        robot.gyroTurn(this, finalHeading, 4);
                        robot.encoderDrive(this, .75, -20,-20, 10);
                    }
                } else {
                    robot.ColorServo.setPosition(robot.ColorDeploy);
                    sleep(1000);
                    robot.ColorServo.setPosition(robot.ColorStart);
                    robot.encoderStrafe(this, .75, 0, 50, 10);
                    robot.gyroTurn(this, finalHeading, 4);
                    robot.encoderDrive(this, .75, -20,-20, 10);
                }
            } else {
                robot.ColorServo.setPosition(robot.ColorDeploy);
                sleep(1000);
                robot.ColorServo.setPosition(robot.ColorStart);
                robot.encoderStrafe(this, .75, 0, 67, 10);
                robot.gyroTurn(this, finalHeading, 4);
                robot.encoderDrive(this, .75, -20,-20, 10);
            }
        }
        else {
            // TODO: Put rest of movement, for crater park and/or deploy marker
            telemetry.addData("Can't turn towards target", initialHeading);
            telemetry.update();

        }
    }


    public void runCrater() throws InterruptedException {

        runtime.reset();
        boolean turnSuccessful = false;
        int times = 0;
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

        if (turnSuccessful) {
            robot.ColorServo.setPosition(robot.ColorSample);
            robot.encoderDrive(this, .75, 18, 18, 2);
            robot.encoderStrafe(this, 0.75, 15, 0, 4);
            robot.encoderDrive(this, 0.75, 5, 6, 2);
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
                robot.encoderStrafe(this, 0.75, 0, 17, 10);  // TODO Strafe right 15 inches
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
                    robot.encoderStrafe(this, 0.75, 0, 17, 10);  // TODO Strafe right 15 inches
                    telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                            .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                            .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                            .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                    telemetry.update();
                    if (robot.isGold() == true) {
                        robot.ColorServo.setPosition(robot.ColorDeploy);
                        sleep(1000);
                        robot.ColorServo.setPosition(robot.ColorStart);
                        telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                                .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                                .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                                .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                        telemetry.update();
                        robot.encoderStrafe(this, .75, 0, 10, 2);
                        robot.encoderDrive(this, .75, 10, 10, 3);
                    } else {
                        robot.ColorServo.setPosition(robot.ColorDeploy);
                        sleep(1000);
                        robot.ColorServo.setPosition(robot.ColorStart);
                        robot.encoderStrafe(this, .75, 0, 10, 2);
                        robot.encoderDrive(this, .75, 10, 10, 3);
                        telemetry.addData("IS NOT GOLD", "EXITING");
                        telemetry.update();

                    }
                } else {
                    robot.ColorServo.setPosition(robot.ColorDeploy);
                    sleep(1000);
                    robot.ColorServo.setPosition(robot.ColorStart);
                    robot.encoderDrive(this, 0.75, -5, -5, 10);
                    robot.encoderStrafe(this, .75, 0, 27, 3);
                    robot.encoderDrive(this, .75, 10, 10, 3);
                }
            } else {
                robot.ColorServo.setPosition(robot.ColorDeploy);
                sleep(1000);
                robot.ColorServo.setPosition(robot.ColorStart);
                robot.encoderDrive(this, 0.75, -5, -5, 10);
                robot.encoderStrafe(this, .75, 0, 44, 3);
                robot.encoderDrive(this, .75, 10, 10, 3);
            }
        }
        else {
            // TODO: Put rest of movement, for crater park and/or deploy marker
            telemetry.addData("Can't turn towards target", initialHeading);
            telemetry.update();

        }
    }

    public boolean deployLander(int timeout) throws InterruptedException {
        // TODO: the Zero Power Behavior is already BRAKE in Hardware.Meet1Robot, so you can remove
        robot.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Pull the variable declarations out of this method and put them into encoderLiftMotor
        final int COUNTS_PER_SECOND_MAX = 600;  // REV Core Hex
        final double COUNTS_PER_MOTOR_REV = 1680;  // AndyMark NeveRest 60:1 CPR
        final double DRIVE_GEAR_REDUCTION = 1.0;   // No gears, just motor shafts
        final double WHEEL_DIAMETER_INCHES = 2.5;   // Diameter of spool

        final double encoderInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028841971693993751);

        // TODO: You should do the runtime calculations similar to encoderDrive() to see if you deployed or not
        // TODO: E.g., int stopTime = runtime.seconds() + timeout;
        // TODO: You need to call encoderLiftMotor(-7.25) to get it to drop 7.25 inches instead of doing the set power and stuff yourself
        robot.LiftMotor.setTargetPosition(robot.LiftMotor.getCurrentPosition() - (int)Math.round(7.25*encoderInch));
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LiftMotor.setPower(.6);

        // TODO:  Before you run this, you should do:  if ( !opModeIsActive() || runtime.milliseconds() > stopTime) return false;
        // TODO:  you

        robot.encoderDrive(this, .75, -1, -1 ,1);

        // TODO: You should call encoderLiftMotor(7.25) to get it to pull the lift down the same amount
        // TODO: But before you call it, you need to again do:  if ( !opModeIsActive() || runtime.milliseconds() > stopTime ) return false;
        robot.LiftMotor.setTargetPosition(robot.LiftMotor.getCurrentPosition() + (int)Math.round(7.25*encoderInch));
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LiftMotor.setPower(.6);

        // TODO: You've just created a recursion loop because you're calling yourself within yourself, all you need to do here is
        // TODO:

        if (deployLander(10)) {
            return true;
        }
        else {
            return false;
        }
    }
        public void encoderLiftMotor(LinearOpMode caller,
        double speed,
        double encoderInch,
        double leftInches, double rightInches,
        double timeoutS) throws InterruptedException {

            int newLiftMotorTarget;
            //int getHeading = gyro.getIntegratedZValue();
            long encoderTimeout = 2000;   // Wait no more than two seconds, an eternity, to set

            if ( !caller.opModeIsActive() )
                return;

            robot.setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            boolean LiftMotorEncoderSet = false;

            newLiftMotorTarget = robot.LiftMotor.getCurrentPosition() + (int) Math.round(leftInches * encoderInch);
            LiftMotorEncoderSet = robot.setEncoderPosition(caller, robot.LiftMotor, newLiftMotorTarget, encoderTimeout);

            if ( ! (LiftMotorEncoderSet) ) {
                caller.telemetry.addLine("Encoders CANNOT be set, aborting OpMode");
                caller.telemetry.update();
                caller.sleep(10000);    // Can't go any further, allow telemetry to show, then return
                return;
            }

            // keep looping while we are still active, and there is time left, and motors haven't made position.
            boolean isBusy;
            int LiftMotorCurPos;
            double stopTime = runtime.seconds() + timeoutS;
            double LiftMotorPower;
            double lastSetTime = runtime.milliseconds();
            int HeadingLoop;

            do {
                LiftMotorPower = speed;
                if (LiftMotorPower <= 0.01) {
                    lastSetTime = runtime.milliseconds();
                    LiftMotorPower = speed;
                    LiftMotorPower = speed;
                }

               LiftMotorPower = Range.clip(LiftMotorPower, -1.0, 1.0);
                robot.LiftMotor.setPower(LiftMotorPower);

                caller.telemetry.addData("Power:", "LiftMotorPower %.2f", LiftMotorPower);
                caller.telemetry.update();
                LiftMotorCurPos = robot.LiftMotor.getCurrentPosition();
                isBusy = (Math.abs(LiftMotorCurPos - newLiftMotorTarget) >= 5);
            }
            while (caller.opModeIsActive() && isBusy && runtime.seconds() < stopTime);

            // Stop all motion;
            robot.LiftMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
