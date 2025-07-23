package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeSoloLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.OneHundredMSIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTurret;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain {
    public enum State {
        FOLLOW_SPLINE,
        GO_TO_POINT,
        ADJUST,
        DRIVE,
        FINAL_ADJUSTMENT,
        BRAKE,
        WAIT_AT_POINT,
        IDLE
    }
    public State state = State.IDLE;

    public PriorityMotor leftFront, leftRear, rightRear, rightFront;
    public nPriorityServo brakePad;
    private final List<PriorityMotor> motors;

    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;

    public Localizer[] localizers;
    public Vision vision;
    public Robot robot;

    public boolean intakeDriveMode = false;
    public static double slowSpeed = 0.6;
    public static double pval = 1.5;
    public static double raiseAngle = 1.945;
    public static double downAngle = 0;

    public Drivetrain(Robot robot) {
        HardwareMap hardwareMap = robot.hardwareMap;
        this.hardwareQueue = robot.hardwareQueue;
        this.sensors = robot.sensors;
        this.robot = robot;

        leftFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftFront"),
            "leftFront",
            4, 5, 1.0, sensors
        );

        leftRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear",
            4, 5, -1.0, sensors
        );
        rightRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear",
            4, 5, sensors
        );
        rightFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront",
            4, 5, sensors
        );

        brakePad = new nPriorityServo(
            new Servo[] {robot.hardwareMap.get(Servo.class, "brakePad")},
            "brakePad",
            nPriorityServo.ServoType.HITEC,
            0, 1, 0,
            new boolean[] {false},
            1.0, 5
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        hardwareQueue.addDevice(brakePad);
        setBrakePad(false);

        configureMotors();

        localizers = new Localizer[]{
                new IMUMergeSoloLocalizer(hardwareMap, sensors, this, "#0000ff", "#ff00ff"),
                new IMULocalizer(hardwareMap, sensors, this, "#ff0000", "#00ff00"),
                new IMUMergeLocalizer(hardwareMap, sensors, this, "#ffff00", "#00ffff"),
                new OneHundredMSIMULocalizer(hardwareMap, sensors, this, "#aa0000", "#00ee00"),
                new TwoWheelLocalizer(hardwareMap, sensors, this, "#aaaa00", "#00aaaa"),
                new Localizer(hardwareMap, sensors, this, "#0000aa", "#aa00aa")
        };
//        setMinPowersToOvercomeFriction();
    }

    // leftFront, leftRear, rightRear, rightFront
    double[] minPowersToOvercomeFriction = {
            0.2413194940290,
            0.2337791473,
            0.246122952040818,
            0.2737912666227906
    };

    public void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0]);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1]);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2]);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3]);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195);
        }
    }

    public void setFineMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0] * 3 / 4);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1] * 3 / 4);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2] * 3 / 4);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3] * 3 / 4);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195 / 2);
        }
    }

    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        leftRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightFront.setMinimumPowerToOvercomeStaticFriction(0.0);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0);
        }
    }

    Spline path = null;
    public int pathIndex = 0;
    public Pose2d targetPoint = new Pose2d(0,0,0);
    Pose2d lastTargetPoint = new Pose2d(0,0,0);
    public static int pathRadius = 20;

    double xError = 0.0;
    double yError = 0.0;
    double turnError = 0.0;

    public static double xSlowdown = 20;
    public static double ySlowdown = 20;
    public static double turnSlowdown = 60;

    public static double kAccelX = 0.0;
    public static double kAccelY = 0.0;
    public static double kAccelTurn = 0.0;

    public static double xBrakingDistanceThreshold = 5;
    public static double xBrakingSpeedThreshold = 20;
    public static double xBrakingPower = -0.22;

    public static double yBrakingDistanceThreshold = 5;
    public static double yBrakingSpeedThreshold = 16;
    public static double yBrakingPower = -0.1;

    public static double centripetalTune = 0.5;
    public static double finalPIDThreshold = 9;
    public static double slowdownPoints = 3;
    public static double strafeTune = 0.15;
    boolean slowDown = false;

    public static double turnBrakingAngleThreshold = 20; // in degrees
    public static double turnBrakingSpeedThreshold = 135; // in degrees
    public static double turnBrakingPower = -0.15;

    double targetForwardPower = 0;
    double targetStrafePower = 0;
    double targetTurnPower = 0;
    public static double maxVelocity = 78.77995865593925; // TODO: Tune me!

    long perfectHeadingTimeStart = System.currentTimeMillis();

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        Pose2d estimate = sensors.getOdometryPosition();
        ROBOT_POSITION = new Pose2d(estimate.x, estimate.y,estimate.heading);
        ROBOT_VELOCITY = sensors.getVelocity();

        if (path != null) {
            //update initial variables + states. pathIndex is the index of the next defined point in the spline
            pathIndex = Math.min(path.poses.size()-1, pathIndex);
            state = State.FOLLOW_SPLINE;
            maxPower = path.poses.get(pathIndex).power;

            // lastRadius --> previous radii. have a Math.max(0, pathIndex - 1) to account for currently at first start. radiusToPath --> current
            double lastRadius = path.poses.get(Math.max(0,pathIndex-1)).getDistanceFromPoint(estimate);
            double radiusToPath = path.poses.get(pathIndex).getDistanceFromPoint(estimate);

            // determine index of next pose needed by finding ideal distance away(i.e. not a past point and not a too far ahead point)
            while (radiusToPath < pathRadius && pathIndex != path.poses.size()) {
                radiusToPath = path.poses.get(pathIndex).getDistanceFromPoint(estimate);
                if (lastRadius > radiusToPath && radiusToPath > pathRadius/3.0) {
                    break;
                }
                lastRadius = radiusToPath;
                pathIndex ++;
            }

            // pathTarget --> current target location
            SplinePose2d pathTarget = path.poses.get(Math.min(path.poses.size()-1,pathIndex));
            lastTargetPoint = targetPoint;
            targetPoint = pathTarget.clone();

            // If last point, activate slowdown
            if (path.poses.size() - 1 - pathIndex < slowdownPoints) {
                slowDown = true;
            }

            // Essentially, if it is close enough then just do goToPoint. Otherwise spline
            if (pathIndex == path.poses.size() && path.poses.get(path.poses.size()-1).getDistanceFromPoint(estimate) < finalPIDThreshold) {
                state = State.GO_TO_POINT;
                maxPower/=2;
                path = null;
                pathIndex = 0;
            } else {
                targetPoint.heading = Math.atan2(targetPoint.y - ROBOT_POSITION.y, targetPoint.x - ROBOT_POSITION.x);
            }

            // Why the heck is this here???
            targetPoint.heading += pathTarget.reversed ? Math.PI : 0;
        }

        calculateErrors();
        updateTelemetry();

        switch (state) {
            case FOLLOW_SPLINE:
                double radius = (xError*xError + yError*yError) / (2*yError);

                Log.e("radius", ""+radius);
                if (Math.abs(radius) < 50) {
                    Canvas canvas = TelemetryUtil.packet.fieldOverlay();
                    canvas.strokeCircle(estimate.x - radius * Math.sin(estimate.heading), estimate.y + radius * Math.cos(estimate.heading), Math.abs(radius));
                }

                double speed = 0.70 + 0.30 * Math.min(Math.abs(radius), 200)/200.0;

                double targetFwd = maxPower*speed*Math.signum(xError);
                if (slowDown) {
                    targetFwd *= 0.3;
                }
                //3.88193
                double targetTurn = ((pval * Math.exp(-3.94484 * Math.abs(targetFwd)) + 0.725107) * (ROBOT_WIDTH)/ radius) * targetFwd;

                double centripetal = centripetalTune * targetFwd * targetFwd / radius;

                double lastDist = estimate.getDistanceFromPoint(targetPoint);
                int index = Math.max(pathIndex-1,0);
                double dist = estimate.getDistanceFromPoint(path.poses.get(index));
                while (dist < lastDist && index > 0) {
                    index--;
                    lastDist = dist;
                    dist = estimate.getDistanceFromPoint(path.poses.get(index));
                }

                Pose2d point = path.poses.get(index);
                double erX = point.x-estimate.x;
                double erY = point.y-estimate.y;
                double relY = -Math.sin(estimate.heading)*erX + Math.cos(estimate.heading)*erY;

                double strafe = Math.abs(relY) > 2 ? relY*strafeTune : 0;
                strafe = Math.max(Math.min(strafe, 0.2), -0.2);

                double fwd = targetFwd;
                double turn = targetTurn;
                double[] motorPowers = {
                        fwd - turn - centripetal - strafe,
                        fwd - turn + centripetal + strafe,
                        fwd + turn - centripetal - strafe,
                        fwd + turn + centripetal + strafe,
                };
                normalizeArray(motorPowers);
                setMotorPowers(motorPowers[0],motorPowers[1],motorPowers[2],motorPowers[3]);

                break;
            case GO_TO_POINT:
                setMinPowersToOvercomeFriction();
                PIDF();

                if (atPoint()) {
                    if(moveNear) {
                        state = State.ADJUST;
                    }

                    if (finalAdjustment) {
                        finalTurnPID.resetIntegral();
                        perfectHeadingTimeStart = System.currentTimeMillis();
                        state = State.FINAL_ADJUSTMENT;
                    } else {
                        state = State.BRAKE;
                    }
                }
                break;
            case ADJUST:
                setMinPowersToOvercomeFriction();
                PIDF();

                if (!atPoint()) {
                    state = State.GO_TO_POINT;
                }

                if (atHeading()) {
                    state = State.BRAKE;
                }
                break;
            case FINAL_ADJUSTMENT:
                finalAdjustment();

                if (Math.abs(turnError) < Math.toRadians(finalTurnThreshold)) {
                    if (System.currentTimeMillis() - perfectHeadingTimeStart > 150) {
                        state = State.BRAKE;
                    }
                } else {
                    perfectHeadingTimeStart = System.currentTimeMillis();
                }
                break;
            case BRAKE:
                stopAllMotors();
                slowDown = false;
                state = State.WAIT_AT_POINT;
                break;
            case WAIT_AT_POINT:
                if (!atPoint()) state = State.GO_TO_POINT;
                else if (moveNear && !atHeading()) state = State.ADJUST;

                break;
            case DRIVE:
                break;
            case IDLE:
                break;
        }
    }

    private void calculateErrors() {
        double deltaX = (targetPoint.x - sensors.getOdometryPosition().x);
        double deltaY = (targetPoint.y - sensors.getOdometryPosition().y);

        // convert error into direction robot is facing
        xError = Math.cos(sensors.getOdometryPosition().heading)*deltaX + Math.sin(sensors.getOdometryPosition().heading)*deltaY;
        yError = -Math.sin(sensors.getOdometryPosition().heading)*deltaX + Math.cos(sensors.getOdometryPosition().heading)*deltaY;
        turnError = targetPoint.heading - sensors.getOdometryPosition().heading;

        if (moveNear) {
            turnError = Math.atan2(yError, xError);
        }

        // make this like, a good value thats not insanely large from the robot oouiiaaeeaoiaaieee ing
        while(Math.abs(turnError) > Math.PI) {
            turnError -= Math.PI * 2 * Math.signum(turnError);
        }
    }

    public boolean finalAdjustment = false;
    private boolean moveNear = false, willGrab = false;
    boolean stop = true;
    double maxPower = 1.0;

    public static double xThreshold = 1.5;
    public static double yThreshold = 1.5;
    public static double turnThreshold = 5;

    public static double eaThresh = 32;

    public static PID xPID = new PID(0.0225,0.0,0.003);
    public static PID yPID = new PID(0.1,0.0,0.015);
    public static PID turnPID = new PID(0.25,0.0,0.01);
    public static PID turnEAPID = new PID(0.025, 0.0, 0.0015);

    public static PID finalXPID = new PID(0.007, 0.0,0.0012);
    public static PID finalYPID = new PID(0.05, 0.0,0.005);
    public static PID finalTurnPID = new PID(0.002, 0.0,0.001);

    public static PID rotateTeleopPID = new PID(1.0,0.0001,0.01);

    public boolean slidesUp = false;

    public static PID slidesUpXPID = new PID(0.015, 0, 0.002); //PIDs for when the slides are up
    public static PID slidesUpYPID = new PID(0.04, 0, 0);
    public static PID slidesUpTurnPID = new PID(0.2, 0, 0.01);

    public static double finalXThreshold = 5;
    public static double finalYThreshold = 5;
    public static double finalTurnThreshold = 3;

    double fwd, strafe, turn, turnAdjustThreshold, finalTargetPointDistance;

    private void PIDF() {
//        double globalExpectedXError = (targetPoint.x - localizers[0].expected.x);
//        double globalExpectedYError = (targetPoint.y - localizers[0].expected.y);
//
//        if (path != null) {
//            finalTargetPointDistance = Math.abs(Utils.calculateDistanceBetweenPoints(localizers[0].getPoseEstimate(), finalTargetPoint));
//        } else {
//            finalTargetPointDistance = 0;
//        }
//
//        // converting from global to relative
//        double relExpectedXError = globalExpectedXError*Math.cos(localizers[0].heading) + globalExpectedYError*Math.sin(localizers[0].heading);
//        double relExpectedYError = globalExpectedYError*Math.cos(localizers[0].heading) - globalExpectedXError*Math.sin(localizers[0].heading);
//
//        if (Math.abs(finalTargetPointDistance) < 20) { // if we are under threshold switch to predictive PID
//            fwd = Math.abs(relExpectedXError) > xThreshold/2 ? xPID.update(relExpectedXError, -maxPower, maxPower) + 0.05 * Math.signum(relExpectedXError) : 0;
//            strafe = Math.abs(relExpectedYError) > yThreshold/2 ? yPID.update(relExpectedYError, -maxPower, maxPower) + 0.05 * Math.signum(relExpectedYError) : 0;
//        } else {
//            fwd = Math.abs(xError) > xThreshold/2 ? xPID.update(xError, -maxPower, maxPower) + 0.05 * Math.signum(xError) : 0;
//            strafe = Math.abs(yError) > yThreshold/2 ? yPID.update(yError, -maxPower, maxPower) + 0.05 * Math.signum(yError) : 0;
//        }
//        // turn does not have predictiveError
//        turnAdjustThreshold = (Math.abs(xError) > xThreshold/2 || Math.abs(yError) > yThreshold/2) ? turnThreshold/3.0 : turnThreshold;
//        turn = Math.abs(turnError) > Math.toRadians(turnAdjustThreshold)/2? turnPID.update(turnError, -maxPower, maxPower) : 0;
        if (slidesUp) { //if the slides are up, use the up PIDs
            fwd = slidesUpXPID.update(xError - (moveNear ? eaThresh / 2 : 0), -maxPower, maxPower);
            strafe = slidesUpYPID.update(yError, -maxPower, maxPower);
            turn = slidesUpTurnPID.update(turnError, -maxPower, maxPower);
        }
        else {
            fwd = xPID.update(xError - (moveNear ? eaThresh / 2 : 0), -maxPower, maxPower);
            strafe = yPID.update(yError, -maxPower, maxPower);
            turn = turnPID.update(turnError, -maxPower, maxPower);
        }
        if (moveNear) {
            strafe *= 0.25;
            if (atPoint()) {
                setFineMinPowersToOvercomeFriction();
                fwd = 0.0;
                strafe = 0.0;
                turn = Math.abs(turnError) > Math.toRadians(finalTurnThreshold) / 2 ? turnEAPID.update(turnError, -maxPower, maxPower) : 0;
            } else {
                setMinPowersToOvercomeFriction();
            }
        }
        TelemetryUtil.packet.put("TURN", turn);
        Vector2 move = new Vector2(fwd, strafe) /*new Vector2(0, 0)*/;
        setMoveVector(move, turn);

        TelemetryUtil.packet.put("fwd", fwd);
        TelemetryUtil.packet.put("strafe", strafe);
    }

    private void finalAdjustment() {
        double fwd = Math.abs(xError) > finalXThreshold/2 ? finalXPID.update(xError, -maxPower, maxPower) : 0;
        double strafe = Math.abs(yError) > finalYThreshold/2 ? finalYPID.update(yError, -maxPower, maxPower) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(finalTurnThreshold)/2 ? finalTurnPID.update(turnError, -maxPower, maxPower) : 0;

        Vector2 move = new Vector2(fwd, strafe) /*new Vector2(0, 0)*/;
        setMoveVector(move, turn);
    }

    boolean prevAtPoint = false;
    private boolean atPoint() {
        if (moveNear) {
            boolean at = xError * xError + yError * yError <= Math.pow(eaThresh + (prevAtPoint ? 3 : 0), 2);
            //TelemetryUtil.packet.put("at point", (prevAtPoint ? "prev" : "new") + (at ? " at" : " not"));
            prevAtPoint = at;
            return at;
        } else {
            prevAtPoint = false;
            //TelemetryUtil.packet.put("at point", "no near");
        }

        if (finalAdjustment && state != State.GO_TO_POINT) {
            return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < Math.toRadians(finalTurnThreshold);
        }
        if (!stop) {
            return Math.abs(xError) < xThreshold*3 && Math.abs(yError) < yThreshold*3 && Math.abs(turnError) < Math.toRadians(turnThreshold);
        }

        return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < Math.toRadians(turnThreshold);
    }

    private boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(turnError) < Math.toRadians(headingThresh);
    }

    public static double lateralIntakeThresh = 0.3, lateralOuttakeThresh = 3.0;
    private boolean atHeading() {
        return Math.abs(yError) <= (willGrab ? lateralIntakeThresh : lateralOuttakeThresh) && Math.abs(turnError) <= Math.toRadians(90);
    }

    public void resetIntegrals() {
        xPID.resetIntegral();
        yPID.resetIntegral();
        turnPID.resetIntegral();
        turnEAPID.resetIntegral();
        finalTurnPID.resetIntegral();
    }

    public Vector2 vdrive;
    public double vturn;

    public void setMoveVector(Vector2 moveVector, double turn) {
        TelemetryUtil.packet.put("Drivetrain : motor x", moveVector.x);
        TelemetryUtil.packet.put("Drivetrain : motor y", moveVector.y);
        TelemetryUtil.packet.put("Drivetrain : motor turn", turn);

        vdrive = moveVector;
        vturn = turn;

        double[] powers = {
                moveVector.x - turn - moveVector.y,
                moveVector.x - turn + moveVector.y,
                moveVector.x + turn - moveVector.y,
                moveVector.x + turn + moveVector.y
        };
        normalizeArray(powers);
        if (slowDown && !(state == State.FINAL_ADJUSTMENT)) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] = powers[i]*0.3;
            }
        }
        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setTargetPowerSmooth(lf);
        leftRear.setTargetPowerSmooth(lr);
        rightRear.setTargetPowerSmooth(rr);
        rightFront.setTargetPowerSmooth(rf);
    }

    public void stopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setTargetPower(0);
        }
    }

    public void normalizeArray(double[] arr) {
        double largest = 1;
        for (int i = 0; i < arr.length; i++) {
            largest = Math.max(largest, Math.abs(arr[i]));
        }
        for (int i = 0; i < arr.length; i++) {
            arr[i] /= largest;
        }
    }

    // Drive Methods
    public void goToPoint(Pose2d targetPoint, boolean finalAdjustment, boolean stop, double maxPower) {
        this.finalAdjustment = finalAdjustment;
        this.stop = stop;
        this.maxPower = Math.abs(maxPower);
        this.moveNear = false;
        willGrab = false;
        finalTargetPoint = targetPoint;

        if (targetPoint.x != lastTargetPoint.x || targetPoint.y != lastTargetPoint.y || targetPoint.heading != lastTargetPoint.heading) { // if we set a new target point we reset integral
            this.targetPoint = targetPoint;
            lastTargetPoint = targetPoint;

            resetIntegrals();

            state = State.GO_TO_POINT;
        }
    }

    public void goToPoint(Pose2d targetPoint, boolean moveNear, boolean slowDown, boolean stop, double maxPower, boolean grab) {
        this.moveNear = moveNear;
        this.slowDown = slowDown;
        this.stop = stop;
        this.maxPower = Math.abs(maxPower);
        willGrab = grab;

        if (targetPoint.x != lastTargetPoint.x || targetPoint.y != lastTargetPoint.y || targetPoint.heading != lastTargetPoint.heading) {
            this.targetPoint = targetPoint;
            lastTargetPoint = targetPoint;

            resetIntegrals();

            state = State.GO_TO_POINT; /*should use state = Drivetrain.DriveState.GO_TO_POINT*/
        }
    }

    // TeleOp
    public void drive(Gamepad gamepad) {
        resetMinPowersToOvercomeFriction();
        state = State.DRIVE;

        double forward = smoothControls(-1 * gamepad.left_stick_y);
        double strafe = smoothControls(-1 * gamepad.left_stick_x);
        double turn = smoothControls(-gamepad.right_stick_x);

        if (intakeDriveMode) {
            forward *= slowSpeed;
            strafe *= slowSpeed;

            // lock heading
            turn = rotateTeleopPID.update(turnError, -maxPower, maxPower);
            if (Math.abs(turnError) <= Math.toRadians(finalTurnThreshold) / 2) turn = 0;
        } else {
            targetPoint = robot.sensors.getOdometryPosition().clone();
            rotateTeleopPID.resetIntegral();
        }

        Vector2 drive = new Vector2(forward,strafe);
        if (drive.mag() <= 0.05) {
            drive.mul(0);
        }
        setMoveVector(drive,turn);
    }

    public double smoothControls(double value) {
        return 0.7 * Math.tan(0.96 * value);
    }

    // Getters & Setters
    public double getTurnError() {
        return turnError;
    }

    private final double intakeOffset = 9.0;
    public double getExtension() {
        return xError - intakeOffset;
    }

    Pose2d finalTargetPoint;
    public Spline getPath() {
        return path;
    }
    public void setPath(Spline path) {
        pathIndex = 0;
        this.path = path;

        if (path != null) {
            finalTargetPoint = path.poses.get(path.poses.size()-1);
        }
    }

    public void setFinalAdjustment(boolean finalAdjustment) {
        this.finalAdjustment = finalAdjustment;
    }

    public void setStop(boolean stop) {
        this.stop = stop;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    public double getMaxPower() {return maxPower;}

    public Pose2d getPoseEstimate() {
        return ROBOT_POSITION;
    }

    public void setPoseEstimate(Pose2d pose2d) {
        sensors.setOdometryPosition(pose2d);
    }

    public double calcExtension(Pose2d start, Pose2d target) {
        double deltaX = (target.x - start.x);
        double deltaY = (target.y - start.y);
        double angle = Math.atan2(deltaY, deltaX);
        // convert error into direction robot is facing
        xError = Math.cos(angle)*deltaX + Math.sin(angle)*deltaY;
        return xError - intakeOffset + IntakeTurret.extendoOffset;
    }

    public boolean isBusy() {
        return state != State.WAIT_AT_POINT && state != State.IDLE;
    }

    public void configureMotors() {
        for (PriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            hardwareQueue.addDevice(motor);
        }

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void resetSlidesMotorRightFront() {
        Log.e("RESETTTING", "*****RESTETING RIGHT FRONT *************");

        rightFront.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("Drivetrain : state", state);
        TelemetryUtil.packet.put("driveState", state);
        LogUtil.driveState.set(state.toString());

        TelemetryUtil.packet.put("Drivetrain : atPoint", atPoint());
        TelemetryUtil.packet.put("Drivetrain : intakeDriveMode", intakeDriveMode);
        TelemetryUtil.packet.put("Drivetrain : xError", xError);
        TelemetryUtil.packet.put("Drivetrain : yError", yError);
        TelemetryUtil.packet.put("Drivetrain : turnError (deg)", Math.toDegrees(turnError));
        TelemetryUtil.packet.put("Drivetrain : xTarget", targetPoint.x);
        TelemetryUtil.packet.put("Drivetrain : yTarget", targetPoint.y);
        TelemetryUtil.packet.put("Drivetrain : turnTarget (deg)", Math.toDegrees(targetPoint.heading));
        LogUtil.driveTargetX.set(targetPoint.x);
        LogUtil.driveTargetY.set(targetPoint.y);
        LogUtil.driveTargetAngle.set(targetPoint.heading);

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();

        DashboardUtil.drawRobot(canvas, targetPoint, "#ff00ff");
        //canvas.setStroke("red");
        //canvas.strokeCircle(targetPoint.x, targetPoint.y, xThreshold);

        if (path != null) {
            Pose2d last = path.poses.get(0);
            for (int i = 1; i < path.poses.size(); i++) {
                Pose2d next = path.poses.get(i);
                canvas.strokeLine(last.x, last.y, next.x, next.y);
                last = next;
            }
        }
    }

    public void setBrakePad(boolean down) {
        brakePad.setTargetAngle(down ? downAngle : raiseAngle);
    }

    // CURRENTLY UNUSED v
/*
    public void setBreakFollowingThresholds(Pose2d thresholds) {
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        turnThreshold = thresholds.getHeading();
    }

    public void updateLocalizer() {
`        for (Localizer l : localizers) {
            l.updateEncoders(sensors.getOdometryPosition());
            l.update();
        }
        //oldLocalizer.update();`
    }

    public void forceStopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setPowerForced(0.0);
        }
    }

    public void feedforward() {
        targetForwardPower = Math.min(Math.pow(Math.abs(xError)/xSlowdown, 2),1.0)*Math.signum(xError);
        targetStrafePower = Math.min(Math.pow(Math.abs(yError)/ySlowdown, 0.5),1.0)*Math.signum(yError);
        targetTurnPower = Math.min(Math.pow(Math.abs(turnError)/Math.toRadians(turnSlowdown), 2),1.0)*Math.signum(turnError);

        double fwd = Math.abs(xError) > xThreshold/2 ? targetForwardPower + kAccelX*(targetForwardPower - ROBOT_VELOCITY.x/(Globals.MAX_X_SPEED*(12/sensors.getVoltage()))) : 0;
        double strafe = Math.abs(yError) > yThreshold/2 ? targetStrafePower + kAccelY*(targetStrafePower - ROBOT_VELOCITY.y/(Globals.MAX_Y_SPEED*(12/sensors.getVoltage()))) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(turnThreshold/2) ? targetTurnPower + kAccelTurn*(targetTurnPower - ROBOT_VELOCITY.heading/(Globals.MAX_HEADING_SPEED*(12/sensors.getVoltage()))) : 0;

        setMinPowersToOvercomeFriction();

        // electronic braking (turn off min power to overcome friction if we are braking)
        if (Math.abs(xError) < xBrakingDistanceThreshold && Math.abs(ROBOT_VELOCITY.x) > xBrakingSpeedThreshold) {
            fwd = xBrakingPower * Math.signum(xError);
            resetMinPowersToOvercomeFriction();
        }
//        if (Math.abs(yError) < yBrakingDistanceThreshold && Math.abs(ROBOT_VELOCITY.y) > yBrakingSpeedThreshold) {
//            strafe = yBrakingPower * Math.signum(yError);
//            resetMinPowersToOvercomeFriction();
//        }
        if (Math.abs(turnError) < Math.toRadians(turnBrakingAngleThreshold) && Math.abs(ROBOT_VELOCITY.heading) > Math.toRadians(turnBrakingSpeedThreshold)) {
            turn = turnBrakingPower * Math.signum(turnError);
            resetMinPowersToOvercomeFriction();
        }

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, turn);
    }

    public static PID rotateTeleopPID = new PID(1.0,0.,0.01);

    public void rotate(Gamepad gamepad, double heading, double threshold, double maxPower) {
        if (heading != lastTargetPoint.heading) { // if we set a new target point we reset integral
            this.targetPoint = new Pose2d(localizers[0].x, localizers[0].y, Math.toRadians(heading));
            this.maxPower = Math.abs(maxPower);

            lastTargetPoint = targetPoint;

            resetIntegrals();
        }

        state = State.DRIVE;

        double forward = smoothControls(gamepad.left_stick_y);
        double strafe = smoothControls(gamepad.left_stick_x);
        double turn = Math.abs(turnError) > Math.toRadians(threshold)? rotateTeleopPID.update(turnError, -maxPower, maxPower) : 0;

        Log.e("turn", turn + "");

        Vector2 drive = new Vector2(forward,strafe);
        if (drive.mag() <= 0.05) {
            drive.mul(0);
        }
        setMoveVector(drive,turn);
    }
 */
}