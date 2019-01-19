package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="Red Depot", group="Red OpMode")
public class RedDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 225;
        deployHeading = 270;
        halfHeading = 0;
        leftSampleAngle = 180;
        lastHeading = 45;
        lastFinalHeading = 90;
        robot.robot_init(hardwareMap,true);
        initRobot();
        boolean targetVisible = false;

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");

            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();

        }



        super.runDepot(true);
        stopRobot();


    }
}
