package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Crater", group="Red OpMode")
public class RedCrater extends AutoCrater {


    public void runOpMode() throws InterruptedException {
        initialHeading = 300;
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.reallyRunOpMode();


    }
}