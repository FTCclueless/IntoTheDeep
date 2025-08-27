package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class IntakeTurret {
    private final Robot robot;
    public final IntakeExtension intakeExtension;
    public final nPriorityServo clawRotation, claw, turretArm, turretRotation;

    private double targetLength, clawRotationTarget, turretArmTarget, turretRotationTarget;

    private boolean closed;
    public static double clawOpenAngle = 0.0;
    public static double clawCloseAngleLoose = 1.05;
    public static double clawCloseAngleTight = 1.18;

    public IntakeTurret(Robot robot) {
        this.robot = robot;

        intakeExtension = new IntakeExtension(robot);

        claw = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClaw")},
                "intakeClaw",
                nPriorityServo.ServoType.AXON_MINI,
                0.492, 0.755, 0.492,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(claw);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeClawRotation")},
                "intakeClawRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.06,0.67,0.37,
                new boolean[] {false},
                1, 5
        );
        robot.hardwareQueue.addDevice(clawRotation);

        turretArm = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeTurretArm")},
                "intakeTurretArm",
                nPriorityServo.ServoType.AXON_MAX,
                0, 1, 0.179,
                new boolean[] {false},
                3.0, 7.0
        );
        robot.hardwareQueue.addDevice(turretArm);

        turretRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "intakeTurretRotation")},
                "intakeTurretRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.07, 1.0, 0.07,
                new boolean[] {false},
                1.0, 5
        );
        robot.hardwareQueue.addDevice(turretRotation);

        claw.setTargetAngle(clawOpenAngle, 1.0);

        targetLength = 0.0;
        clawRotationTarget = 0.0;
        turretArmTarget = 0.0;
        turretRotationTarget = 0.0;

        closed = false;
    }

    public void update() {
        intakeExtension.setTargetLength(targetLength);
        intakeExtension.update();
    }

    public void setIntakeExtension(double t) {
        targetLength = t;
    }

    // Target is given in robot centric coordinates
    public static double turretLengthTip = 4.8;
    public static double turretLengthLL = 2.3;
    public static double extendoOffset = 6;
    public void intakeAt(Pose2d target) {
        double xError = target.x;
        double yError = target.y;

        if (Math.abs(yError) <= turretLengthTip) {
            targetLength = xError - extendoOffset - Math.sqrt(turretLengthTip * turretLengthTip - yError * yError)/* - yError / turretLength*/;
            setTurretRotation(Math.PI + Math.atan2(yError, Math.sqrt(turretLengthTip * turretLengthTip - yError * yError)));
            setClawRotation(AngleUtil.mirroredClipAngleTolerence(target.heading - getTurretRotation(), Math.toRadians(20)));
            setTurretArmTarget(nClawIntake.turretGrabAngle, 0.7);
        }
    }

    public void extendTo(Vector2 target) {
        double xError = target.x;
        double yError = target.y;

        if(Math.abs(yError) < turretLengthLL) {
            targetLength = xError - extendoOffset - Math.sqrt(turretLengthLL * turretLengthLL - yError * yError);
            setTurretRotation(Math.PI + Math.atan2(yError, Math.sqrt(turretLengthLL * turretLengthLL - yError * yError)));
        }
    }

    public void setClawState(boolean closed) {
        this.closed = closed;
        claw.setTargetAngle(this.closed ? (tight ? clawCloseAngleTight : clawCloseAngleLoose) : clawOpenAngle, 1.0);
        TelemetryUtil.packet.put("intakeClawGrab", closed);
        LogUtil.intakeClawGrab.set(closed);
    }

    private boolean tight = true;
    public void setTight(boolean b){
        tight = b;
    }

    public void setClawRotation(double t) {
        clawRotation.setTargetAngle(t);
        //clawRotationTarget = t;
        TelemetryUtil.packet.put("intakeClawRotationAngle", t);
        LogUtil.intakeClawRotationAngle.set(t);
    }

    public void setTurretArmTarget(double t) {
        turretArm.setTargetAngle(t);
        //turretArmTarget = t;
    }

    public void setTurretArmTarget(double t, double power){
        turretArm.setTargetAngle(t, power);
    }

    public void setTurretRotation(double t) {
        if (intakeExtension.getLength() < nClawIntake.minExtension)
            t = Utils.minMaxClip(t, 2.50419, 3.951);

        turretRotation.setTargetAngle(t * (3.524 / Math.PI));
        //turretRotationTarget = t;
        TelemetryUtil.packet.put("intakeTurretRotationAngle", t);
        LogUtil.intakeTurretRotationAngle.set(t);
    }

    public double getTurretRotation() {
        return turretRotation.getCurrentAngle() / (3.524 / Math.PI);
    }

    public double getTargetTurretRotation() {
        return turretRotation.getTargetAngle() / (3.524 / Math.PI);
    }

    public double getClawRotation() {
        return clawRotation.getCurrentAngle();
    }

    public boolean isClosed() {
        return closed && claw.inPosition();
    }

    public boolean inPosition() {
        return intakeExtension.inPosition() && clawRotation.inPosition() && turretArm.inPosition() && turretRotation.inPosition();
    }

    public boolean inPosition(double t) {
        return intakeExtension.inPosition() && clawRotation.inPosition(t) && turretArm.inPosition(t) && turretRotation.inPosition(t);
    }

    public boolean extendoInPosition() {
        return intakeExtension.inPosition();
    }

    public boolean rotInPosition() {
        return clawRotation.inPosition();
    }

    public boolean rotInPosition(double t) {
        return clawRotation.inPosition(t);
    }

    public boolean grabInPosition() {
        return claw.inPosition();
    }

    public boolean turretRotInPosition() {
        return turretRotation.inPosition();
    }

    public boolean turretRotInPosition(double t) {
        return turretRotation.inPosition(t);
    }

    public boolean turretAngInPosition() {
        return turretArm.inPosition();
    }

    public boolean turretAngInPosition(double t) {
        return turretArm.inPosition(t);
    }

    public boolean clawInPosition() {
        return claw.inPosition();
    }

}
