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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;


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

@Autonomous(name="Auto Crater", group="Linear Opmode")
@Disabled
public abstract class AutoCrater extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    protected MecanumDrive robot = new MecanumDrive();
    boolean yellowValue = false;
    boolean whiteValue = false;
    protected double initialHeading = 0;
    protected double timeoutS = 5;


    public void reallyRunOpMode() throws InterruptedException {

        runtime.reset();
       // initialHeading = 130;

        boolean b = false;
        int times = 0;
        do {
            b = robot.gyroTurn(this, initialHeading, timeoutS);
            if (b == false) {
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
        while (opModeIsActive() && b == false && times++ < 3);

        if (b) {
            robot.encoderDrive(this, .75, 18, 18, 2);
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
                        robot.encoderDrive(this, 0.75, 5, 5, 10);
                        telemetry.addData("Left Front", "Encoder: %d", robot.LFront.getCurrentPosition())
                                .addData("Right Front", "Encoder: %d", robot.RFront.getCurrentPosition())
                                .addData("Left Back", "Encoder: %d", robot.LBack.getCurrentPosition())
                                .addData("Right Back", "Encoder: %d", robot.RBack.getCurrentPosition());
                        telemetry.update();
                        robot.encoderStrafe(this, .75, 0, 10, 2);
                        robot.encoderDrive(this, .75, 10, 10, 3);
                    } else {
                        robot.encoderStrafe(this, .75, 0, 10, 2);
                        robot.encoderDrive(this, .75, 10, 10, 3);
                        telemetry.addData("IS NOT GOLD", "EXITING");
                        telemetry.update();

                    }
                } else {
                    robot.encoderDrive(this, 0.75, 5, 5, 10);
                    sleep(500);
                    robot.encoderDrive(this, 0.75, -5, -5, 10);
                    robot.encoderStrafe(this, .75, 0, 27, 3);
                    robot.encoderDrive(this, .75, 10, 10, 3);
                }
            } else {
                robot.encoderDrive(this, 0.75, 5, 5, 10);
                sleep(500);
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
}




