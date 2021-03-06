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

import org.firstinspires.ftc.teamcode.Hardware.Meet2Robot;


/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Mecanum Trollbot", group="Test Opmode")

public class MecanumTrollbot extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Meet2Robot robot = new Meet2Robot();
    private double lastpress = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap, true);
        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


        wheelPower = robot.motorPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        robot.LFront.setPower(wheelPower[0] * .5);
        robot.RFront.setPower(wheelPower[1] * .5);
        robot.LBack.setPower(wheelPower[2] * .5);
        robot.RBack.setPower(wheelPower[3] * .5);

        telemetry.addData("Left Front", "Power: %.2f - Encoder: %d", wheelPower[0], robot.LFront.getCurrentPosition())
                 .addData("Right Front", "Power: %.2f - Encoder: %d", wheelPower[1], robot.RFront.getCurrentPosition())
                 .addData("Left Back", "Power: %.2f - Encoder: %d", wheelPower[2], robot.LBack.getCurrentPosition())
                 .addData("Right Back", "Power: %.2f - Encoder: %d", wheelPower[3], robot.RBack.getCurrentPosition());
     /*   int redValue = robot.color.red();
        int blueValue = robot.color.blue();
        int greenValue = robot.color.green();
        telemetry.addData("color blue", blueValue);
        telemetry.addData("color red", redValue);
        telemetry.addData("color green", greenValue);
        telemetry.addData("isGold", robot.isGold());
        telemetry.addData ("heading", robot.getHeading());
*/
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
