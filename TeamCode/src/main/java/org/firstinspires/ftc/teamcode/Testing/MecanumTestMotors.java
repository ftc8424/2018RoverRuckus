/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Meet1Robot;


/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * This test just takes A, B, X and Y and has motors Left Front, Right Front, Left Back
 * and Right Back with positive power respectively
 *
 */
@TeleOp(name="Mecanum Test Motors", group="Test Opmode")

public class MecanumTestMotors extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Meet1Robot robot = new Meet1Robot();
    private double lastpress = 0;
    private double motorStop = 0;
    private boolean motorRunning = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
/*
        robot.LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double[] wheelPower = { 0, 0, 0, 0 };

        telemetry.addData("Status", "Running: " + runtime.toString());

        if ( gamepad1.a && lastpress + 1000 < runtime.milliseconds() && !motorRunning ) {
            robot.LFront.setPower(1);
            robot.RFront.setPower(1);
            robot.LBack.setPower(1);
            robot.RBack.setPower(1);
            lastpress = runtime.milliseconds();
            motorStop = runtime.milliseconds() + 10000;
            motorRunning = true;
        } else if ( gamepad1.b && lastpress + 1000 < runtime.milliseconds() && !motorRunning) {
            robot.LFront.setPower(-1);
            robot.RFront.setPower(-1);
            robot.LBack.setPower(-1);
            robot.RBack.setPower(-1);
            lastpress = runtime.milliseconds();
            motorStop = runtime.milliseconds() + 10000;
            motorRunning = true;

        }
        if ( motorRunning && runtime.milliseconds() >= motorStop ) {
            motorRunning = false;
            robot.LFront.setPower(0);
            robot.RFront.setPower(0);
            robot.LBack.setPower(0);
            robot.RBack.setPower(0);
        }
            /*
            wheelPower = robot.motorPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            robot.LFront.setPower(wheelPower[0]);
            robot.RFront.setPower(wheelPower[1]);
            robot.LBack.setPower(wheelPower[2]);
            robot.RBack.setPower(wheelPower[3]);
*/


        telemetry.addData("Left Front", "Power: %.2f - Encoder: %d", robot.LFront.getPower(), robot.LFront.getCurrentPosition())
                 .addData("Right Front", "Power: %.2f - Encoder: %d", robot.RFront.getPower(), robot.RFront.getCurrentPosition())
                 .addData("Left Back", "Power: %.2f - Encoder: %d", robot.LBack.getPower(), robot.LBack.getCurrentPosition())
                 .addData("Right Back", "Power: %.2f - Encoder: %d", robot.RBack.getPower(), robot.RBack.getCurrentPosition());
        int redValue = robot.color.red();
        int blueValue = robot.color.blue();
        int greenValue = robot.color.green();
        telemetry.addData("color blue", blueValue);
        telemetry.addData("color red", redValue);
        telemetry.addData("color green", greenValue);
        telemetry.addData("isGold", robot.isGold());
        telemetry.addData ("heading", robot.getHeading());

    } // loop

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public    void stop() {
        robot.LFront.setPower(0);
        robot.RFront.setPower(0);
        robot.LBack.setPower(0);
        robot.RBack.setPower(0);
    }
}
