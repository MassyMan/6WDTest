package org.firstinspires.ftc.teamcode.SWD;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class Main {
    public static final double inPerTick = 0.001; // ⚠️ Tune this
    private static final double turnKfwd = 0.01;
    private static final double slowSpeed = 0.5;

    private int waypointIdx = 0;

    public double moveSpeed;
    public double targetX, targetY;
    public double currentX, currentY, currentHeading;
    public double leftPower, rightPower;

    Utils utils;
    private final TwoDeadWheelLocalizer localizer;

    DcMotor leftDriveF, leftDriveB, rightDriveF, rightDriveB;

    // =====================================
    // Waypoint class
    // =====================================
    public static class Waypoint {
        public double x;
        public double y;
        public double moveSpeed;
        public double threshold;
        public boolean slowdown;
        public double slowdownDist; // now per waypoint

        public Waypoint(double x, double y, double moveSpeed, double threshold, boolean slowdown, double slowdownDist) {
            this.x = x;
            this.y = y;
            this.moveSpeed = moveSpeed;
            this.threshold = threshold;
            this.slowdown = slowdown;
            this.slowdownDist = slowdownDist;
        }
    }

    private final List<Waypoint> waypoints = new ArrayList<>();

    public enum State {
        FORWARD_SPLINE,
        BRAKE,
        IDLE
    }
    public State state = State.IDLE;

    // =====================================
    // Constructors
    // =====================================
    public Main(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Main(HardwareMap hardwareMap, Pose2d startPose) {
        // Drive Motors
        leftDriveF = hardwareMap.get(DcMotor.class,"leftDriveF");
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

        // IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Localizer
        localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, inPerTick, startPose);
    }

    // =====================================
    // Pose helpers
    // =====================================
    public Pose2d getPose() {
        return localizer.getPose();
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
        return Math.toDegrees(getPose().heading.log());
    }

    public void resetPose(Pose2d newPose) {
        localizer.setPose(newPose);
    }

    // =====================================
    // Driving helpers
    // =====================================
    public double splineIdealHeading() {
        Waypoint wp = waypoints.get(waypointIdx);
        double dx = wp.x - getCurrentX();
        double dy = wp.y - getCurrentY();
        return Math.atan2(dy, dx);
    }

    public double deltaHeadingSpline() {
        double rawDelta = splineIdealHeading() - Math.toRadians(getCurrentHeading());
        return utils.deltaAngleClip(rawDelta);
    }

    public void motorPowersFWD(double speed) {
        leftPower = speed + (deltaHeadingSpline() * turnKfwd);
        rightPower = speed - (deltaHeadingSpline() * turnKfwd);
        applyDrivetrainPower(leftPower, rightPower);
    }

    public void applyDrivetrainPower(double leftPower, double rightPower) {
        leftDriveF.setPower(leftPower);
        leftDriveB.setPower(leftPower);
        rightDriveF.setPower(rightPower);
        rightDriveB.setPower(rightPower);
    }

    // =====================================
    // Waypoint management
    // =====================================
    public void addWaypoint(double targetX, double targetY, double moveSpeed, double threshold, boolean slowdown, double slowdownDist) {
        waypoints.add(new Waypoint(targetX, targetY, moveSpeed, threshold, slowdown, slowdownDist));
    }

    private Waypoint getCurrentWaypoint() {
        return waypoints.get(waypointIdx);
    }

    public boolean atThreshold(Waypoint wp) {
        double dx = wp.x - getCurrentX();
        double dy = wp.y - getCurrentY();
        double dist = Math.hypot(dx, dy);
        return dist <= wp.threshold;
    }

    public boolean atSlowDown(Waypoint wp) {
        double dx = wp.x - getCurrentX();
        double dy = wp.y - getCurrentY();
        double dist = Math.hypot(dx, dy);
        return wp.slowdown && dist <= wp.slowdownDist;
    }

    // =====================================
    // Start spline sequence
    // =====================================
    public void startSplineSequence() {
        waypointIdx = 0;
        if (!waypoints.isEmpty()) {
            state = State.FORWARD_SPLINE;
        }
    }

    // =====================================
    // Main update loop
    // =====================================
    public void update() {
        localizer.update();

        switch (state) {
            case FORWARD_SPLINE:
                if (waypointIdx >= waypoints.size()) {
                    state = State.IDLE;
                    break;
                }

                Waypoint wp = getCurrentWaypoint();

                if (atThreshold(wp)) {
                    waypointIdx++;
                    if (waypointIdx >= waypoints.size()) {
                        state = State.BRAKE;
                        break;
                    }
                    wp = getCurrentWaypoint();
                }

                double speed = wp.moveSpeed;
                if (atSlowDown(wp)) {
                    speed *= slowSpeed;
                }

                motorPowersFWD(speed);
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
