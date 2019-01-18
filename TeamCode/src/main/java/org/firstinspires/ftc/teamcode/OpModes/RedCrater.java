package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Crater", group="Red OpMode")
public class RedCrater extends AutoBase {


    public void runOpMode() throws InterruptedException {

        initialHeading = 310;
        deployHeading = 270;
        leftSampleAngle = 275;
        lastHeading = 45;
        lastFinalHeading = 90;
        finalHeading = 180;


        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();

            do {

            if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
                telemetry.update();
                robot.encoderDrive(this, .25, -.5, -.5, 3);
            }

            else if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
                telemetry.update();
                robot.encoderDrive(this, .25, .5, .5, 3);
            }

            else if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
                telemetry.update();
                robot.encoderStrafe(this, .25, .5, 0, 3);
            }

            else if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
                telemetry.update();
                robot.encoderStrafe(this, .25, 0, .5, 3);
            }

            else if (robot.translation.get(1) > 00 && robot.translation.get(0) == 000) {
                telemetry.update();
                robot.gyroTurn(this, 90,3);
                robot.encoderDrive(this,.25,-.5,-.5 ,3);
                robot.gyroTurn(this, initialHeading, 3);
            }

            else if (robot.translation.get(1) < 00 && robot.translation.get(0) == 000) {
                telemetry.update();
                robot.gyroTurn(this, 90, 3);
                robot.encoderDrive(this, .25, .5, .5, 3);
                robot.gyroTurn(this,initialHeading,3);

            }

            else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
                telemetry.update();
                robot.gyroTurn(this, 90, 3);
                robot.encoderStrafe(this, .25, .5, 0, 3);
                robot.gyroTurn(this,initialHeading,3);
            }

            else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
                telemetry.update();
                robot.gyroTurn(this, 90, 3);
                robot.encoderStrafe(this, .25, 0, .5, 3);
                robot.gyroTurn(this,initialHeading,3);
            }

            } while (robot.translation.get(0) != 000 && robot.translation.get(1) != 00);

        }

        super.runCrater(true);
        stopRobot();


    }
}