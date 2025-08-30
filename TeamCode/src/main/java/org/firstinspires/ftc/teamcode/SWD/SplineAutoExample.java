package org.firstinspires.ftc.teamcode.SWD;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="SplineAutoExample")
public class SplineAutoExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Main drivetrain with starting pose at x=0, y=0, heading=180
        Main robot = new Main(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

        // Add waypoints (last one has slowdown)
        robot.addWaypoint(36, 0, 0.25, 10, false, 0);
        robot.addWaypoint(36, 36, 0.8, 8, false, 0);
        robot.addWaypoint(0, 48, 0.6, 8, false, 0);
        robot.addWaypoint(0, 0, 0.8, 2, true, 20); // slowdown only here

        // Wait for the start of autonomous
        waitForStart();

        // Begin spline sequence
        robot.startSplineSequence();

        // Main loop: keep updating robot until all waypoints are completed
        while (opModeIsActive() && robot.state != Main.State.IDLE) {
            robot.update();
        }

        // Stop drivetrain at the end
        robot.applyDrivetrainPower(0, 0);
    }
}
