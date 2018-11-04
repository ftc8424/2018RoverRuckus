package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Crater", group="Blue OpMode")
public class BlueCrater extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 130;
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.runCrater();


    }
}