package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Depot", group="Red OpMode")
public class RedDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 220;
        deployHeading = 270;
        finalHeading = 180;
        halfHeading = 0;
        lastHeading = 325;
        lastFinalHeading = 335;
        robot.robot_init(hardwareMap,true);
        initRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.runDepot();


    }
}
