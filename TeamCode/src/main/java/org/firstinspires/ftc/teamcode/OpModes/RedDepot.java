package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Depot", group="Red OpMode")
public class RedDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 220;
        finalHeading = 180;
        robot.robot_init(hardwareMap,true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.runDepot();


    }
}
