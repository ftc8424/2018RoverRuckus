///*
//Copyright (c) 2016 Robert Atkinson
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Robert Atkinson nor the names of his contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLTELEOP;
//
///**
// * Created by FTC8424 on 9/15/2016.
// */
//
///**
// * This file contains an example of an iterative (Non-Linear) "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
// * It includes all the skeletal structure that all iterative OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//
//public class TeleOp extends OpMode {
//
//    /* Declare OpMode members. */
//    private ElapsedTime runtime = new ElapsedTime();
//    private HardwareHelper robot = new HardwareHelper(FULLTELEOP);
//
//    long LiftMaxHeight = -1; // 2200 ; //3360 = 19 inches with a 2 inch spool, and a NeveRest 40:1 motor -> (which has 1120 encoder ticks per revolution)
//    long LiftCurrentPosition = 0;
//    double liftSpeed = 0.6;
//    int liftStoneHeight = 691;
//    boolean liftLocked = false;
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        robot.robot_init(hardwareMap);
//        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        telemetry.addData("Status", "Initialized");
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//
//        telemetry.addData("Status", "Running: " + runtime.toString());
//
//        double rightStickVal = -gamepad1.right_stick_y;
//        double leftStickVal = -gamepad1.left_stick_y;
//
//
//        robot.normalDrive(this, leftStickVal, rightStickVal);
//
//        double rightManipVal = -gamepad2.right_stick_y;
//        double leftManipVal = -gamepad2.left_stick_y;
//        robot.leftManip.setPower(leftManipVal);
//        robot.rightManip.setPower(rightManipVal);
//
///* //One Man Drive Team LOLOLOL
//        float rightManipVal2 = -gamepad1.right_trigger;
//        float leftManipVal2 = -gamepad1.left_trigger;
//        robot.rightManip.setPower(rightManipVal2);
//        robot.leftManip.setPower(leftManipVal2);
//
//        if(gamepad1.right_stick_button){
//            robot.rightManip.setPower(1);
//        }
//        if(gamepad1.left_stick_button){
//            robot.leftManip.setPower(1);
//        }
//*/
//
//
//
//
//
//        /*THIS IS WHAT NEEDS TO BE CODED AS PER COACH JERRY:
//         *
//         * When the gamepad2.dpad_up is called, the manipulator should be able to move up as long
//         * as the gamepad2.dpad is held down. When it is not held down, it stops moving. The code
//         * needs to have a restriction, so that the rope doesn't cause the motor to burn out. Also,
//         * when the gamepad2.dpad_up is called, the code should be able to recall how many rotations
//         * were made when going up, and should retrace those rotations downward to get the
//         * manipulator to its starting position, that way no motors get burnt out, and there can
//         * be a single starting point for the simplicity of the code, and the driver controlling
//         * that part of the robot.
//         */
//
//        LiftCurrentPosition = robot.lift.getCurrentPosition();
//        if (gamepad2.right_bumper||gamepad1.right_bumper) {
//            if (liftLocked) {
//                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftLocked = false;
//            }
//            if ((LiftCurrentPosition < LiftMaxHeight - 50) || LiftMaxHeight == -1) {
//                robot.lift.setPower(liftSpeed);
//            } else {
//                robot.lift.setPower(0);
//            }
//        } else if (gamepad2.left_bumper||gamepad1.left_bumper) {
//            if (liftLocked) {
//                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftLocked = false;
//            }
//            if ( LiftCurrentPosition > 0 || LiftMaxHeight == -1 ) {
//                robot.lift.setPower(Range.clip(-liftSpeed-0.25,-1.0,1.0));
//            } else {
//                robot.lift.setPower(0);
//            }
//        } else {
//            if (liftLocked == false){
//                robot.lift.setPower(0);
//            }
//
//        }
//        if (gamepad2.a && liftLocked == false){
//            robot.lift.setTargetPosition(robot.lift.getCurrentPosition() + liftStoneHeight);
//            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lift.setPower(liftSpeed);
//            liftLocked = true;
//        }
//        telemetry.addData("lift position", LiftCurrentPosition);
//        telemetry.addData("lift power", robot.lift.getPower());
//        telemetry.addData("Encoders:", "Left:  %,d, Right:  %,d", robot.leftBackDrive.getCurrentPosition(), robot.rightBackDrive.getCurrentPosition());
//        telemetry.addData("Drive:", "Lft Power %.2f, Rgt Power %.2f", leftStickVal, rightStickVal);
//        telemetry.addData("Status", "Debug 1 at: " + runtime.toString());
//
//        if (gamepad2.x || gamepad1.x) {
//            robot.colorArm.setPosition(robot.cArmStart);
//        }
//    } // loop
//
//    //if (gamepad2.y) {
//    //robot.waitForReset()
//
//
//
//
//
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public    void stop() {
//        robot.normalDrive(this, 0, 0);
//    }
//}
