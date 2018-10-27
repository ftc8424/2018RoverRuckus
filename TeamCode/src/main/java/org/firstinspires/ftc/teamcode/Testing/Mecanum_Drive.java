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
@TeleOp(name="Mecanum Trollbot", group="Iterative Opmode")

public class Mecanum_Drive extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive robot = new MecanumDrive();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        robot.LBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        telemetry.addData("Status", "Running: " + runtime.toString());

      /*
       * The three elements needed by drive.motorPower() method are the Magnitude (or force
       * to apply), the angle to drive and then the rotation for the front of the robot.
       * You get these on the driver game pads by using a combination of geometry and trig
       * functions.  The right angle formed by the game pad's left stick can e used for
       * both the magnitude and angle.  The magnitude can be applied by finding the
       * hypotenuse of the right triangle with the x and y values.  That's just done by
       * applying the Pythagorean Theorm so it's H-squared = X-squared + Y+squared.
       *
       * The ANGLE requires trigonometry to figure out.  The TANGENT of an angle of right triangle
       * is the value of the Opposite side over the Adjacent one.  In this case, we don't know
       * the angle but we know the Opposite and Adjacent side lengths and NEED to find the angle.
       * For that, we need the INVERSE of the Tangent function, or the ARCTANGENT.  To get the angle
       * we need to determine the ARC Tangent of the side OPPOSITE the angle (the Y value of the
       * game pad) over the ADJACENT side (the X value of the game pad).  That will give us the
       * ANGLE for that part in 0 - 2*PI values.
       *
       * Nicely, the Math class gives us easy methods to use to compute those:  Math.hypot() and
       * Math.atan2().
       */

        double[] wheelPower = robot.motorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

        robot.LFront.setPower(wheelPower[0]);
        robot.RFront.setPower(wheelPower[1]);
        robot.LBack.setPower(wheelPower[2]);
        robot.RBack.setPower(wheelPower[3]);

        telemetry.addData("Left Front", "Power: %.2f - Encoder: %d", wheelPower[0], robot.LFront.getCurrentPosition())
                 .addData("Right Front", "Power: %.2f - Encoder: %d", wheelPower[1], robot.RFront.getCurrentPosition())
                 .addData("Left Back", "Power: %.2f - Encoder: %d", wheelPower[2], robot.LBack.getCurrentPosition())
                 .addData("Right Back", "Power: %.2f - Encoder: %d", wheelPower[3], robot.RBack.getCurrentPosition());

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
