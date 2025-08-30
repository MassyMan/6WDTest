package org.firstinspires.ftc.teamcode.SWD;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class Main {
    public static final double inPerTick = 0.001; // ⚠️
    private static final double turnKfwd = 0.01;

    public double waypointIdx = 0;

    public double threshold;

    private boolean slowdown;
    private double slowdownDist; // Threshold for beginning slowdown
    private static final double slowSpeed = 0.5; // slowdown speed

    public double moveSpeed;
    public double targetX, targetY, targetHeading;
    public double currentX, currentY, currentHeading;
    public double leftPower, rightPower;

    Utils utils;
    private final TwoDeadWheelLocalizer localizer;

    DcMotor leftDriveF, leftDriveB, rightDriveF, rightDriveB;


    // =====================================
    // Waypoint handling (cleaner version)
    // =====================================
    public static class Waypoint {
        public double x;
        public double y;
        public double moveSpeed;
        public double threshold;
        public boolean slowdown;

        public Waypoint(double x, double y, double moveSpeed, double threshold, boolean slowdown) {
            this.x = x;
            this.y = y;
            this.moveSpeed = moveSpeed;
            this.threshold = threshold;
            this.slowdown = slowdown;
        }
    }

    private final List<Waypoint> waypoints = new ArrayList<>();

    public enum State {
        FORWARD_SPLINE,
        BACKWARD_SPLINE,
        TURN,
        FINAL_ADJUSTMENT,
        BRAKE,
        IDLE
    }
    public State state = State.IDLE;

    public Main(HardwareMap hardwareMap){
        // Drive Motors
        leftDriveF = hardwareMap.get(DcMotor.class,"leftDriveF"); // TODO: Double-check config names
        leftDriveB = hardwareMap.get(DcMotor.class, "leftDriveB");
        rightDriveF = hardwareMap.get(DcMotor.class, "rightDriveF");
        rightDriveB = hardwareMap.get(DcMotor.class, "rightDriveB");

        leftDriveF.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDriveB.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveB.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // IMU Hardware name (in configuration)
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Start pose at origin
        Pose2d startPose = new Pose2d(0, 0, 0); // TODO: move this to a different location so it can be configurable
        localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, inPerTick, startPose);
    }

    public Pose2d getPose() {
        return localizer.getPose();
    }

    public double splineIdealHeading(double x, double y) {
        return Math.atan2(y, x); // Takes current X & Y
    }

    public double getCurrentX() {
        currentX = getPose().position.x;
        return currentX;
    }

    public double getCurrentY() {
        currentY = getPose().position.y;
        return getPose().position.y;
    }

    public double getCurrentHeading() {
        return Math.toDegrees(getPose().heading.log()); // heading is a Rotation2d object, log() gives angle in radians
    }

    public double deltaHeadingSpline() {
        double rawDelta = splineIdealHeading(targetX - getCurrentX(), targetY - getCurrentY()) - getCurrentHeading();
        return utils.deltaAngleClip(rawDelta);
    }

    public void motorPowersFWD(double speed) {
        leftPower = speed + (deltaHeadingSpline() * turnKfwd);
        rightPower = speed + (deltaHeadingSpline() * -turnKfwd);

        applyDrivetrainPower(leftPower, rightPower);
        // TODO: apply motor powers
    }

    public void applyDrivetrainPower(double leftPower, double rightPower) { // Apply motor powers
        leftDriveF.setPower(leftPower);
        leftDriveB.setPower(leftPower);
        rightDriveF.setPower(rightPower);
        rightDriveB.setPower(rightPower);
    }

    // ==============================
    // Waypoint management
    // ==============================
    public void addWaypoint(double targetX, double targetY, double moveSpeed, double threshold, boolean slowdown) {
        waypoints.add(new Waypoint(targetX, targetY, moveSpeed, threshold, slowdown));
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    // ==============================
    // Spline driving
    // ==============================
    public void SplineToPointFWD(double targetX, double targetY, double moveSpeed, double threshold, boolean slowdown, double slowdownDist) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.moveSpeed = moveSpeed;
        this.threshold = threshold;
        this.slowdown = slowdown;
        this.slowdownDist = slowdownDist;

        state = State.FORWARD_SPLINE;
    }

    public boolean atThreshold() {
        return Math.abs(targetX - getCurrentX() + targetY - getCurrentY()) <= threshold;
    }

    public boolean atSlowDown() {
        return Math.abs(targetX - getCurrentX() + targetY - getCurrentY()) <= slowdownDist;
    }

    // =====================================
    // Main update loop
    // =====================================
    public void update() {
        localizer.update(); // Update Field Pose Estimate

        switch (state) {
            case FORWARD_SPLINE:
                deltaHeadingSpline(); // Fetch delta heading to target hypotenuse

                if (atThreshold()) {
                    state = State.IDLE;

                } else if (slowdown && atSlowDown()) { // If the motor powers need to be changed
                    moveSpeed *= slowSpeed; // Reduce target speed (at slow down dist)
                }

                motorPowersFWD(moveSpeed); // Set motor powers

                break;

            case BRAKE:
                applyDrivetrainPower(0, 0);
                state = State.IDLE;
                break;

            case IDLE:
                break;
        }
    }
}
