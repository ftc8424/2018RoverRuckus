package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Crater", group="Red OpMode")
public class RedCrater extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 310;
        deployHeading = 270;
        robot.robot_init(hardwareMap,true);
        initRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.runCrater();


    }
}