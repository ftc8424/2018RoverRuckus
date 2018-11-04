package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Depot", group="Red OpMode")
public class RedDepot extends AutoDepot {


    public void runOpMode() throws InterruptedException {
        initialHeading = 210;
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.reallyRunOpMode();


    }
}
