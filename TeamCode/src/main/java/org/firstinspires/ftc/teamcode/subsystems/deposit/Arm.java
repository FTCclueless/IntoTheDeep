package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Arm {
    public final Sensors sensors;

    public final nPriorityServo armRotation;
    public final nPriorityServo clawRotation;
    public final nPriorityServo claw;

    public static double openRad = 1.2456, closeRad = 0.01, closeLooseRad = 0.15;

    public Arm(Robot robot) {
        this.sensors = robot.sensors;

        armRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "depositArmRotationL"), robot.hardwareMap.get(Servo.class, "depositArmRotationR")},
                "depositArmRotation",
                nPriorityServo.ServoType.AXON_MINI,
                0.335,
                0.982,
                0.871,
                new boolean[] {false, true},
                1.0,
                2.0 // Tee hee - eric
        );
        robot.hardwareQueue.addDevice(armRotation);

        clawRotation = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "depositClawRotation")},
                "depositClawRotation",
                nPriorityServo.ServoType.AXON_MAX,
                0,
                1,
                0.32,
                new boolean[] {true},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(clawRotation);

        claw = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "depositClaw")},
                "depositClaw",
                nPriorityServo.ServoType.AXON_MINI,
                0.01,
                0.24,
                0.02,
                new boolean[] {false},
                1.0,
                2.0
        );
        robot.hardwareQueue.addDevice(claw);
    }

    public void setArmRotation(double targetRad, double power) { armRotation.setTargetAngle(targetRad, power); }

    public void setClawRotation(double targetRad, double power) { clawRotation.setTargetAngle(targetRad, power); }

    public void clawClose() { claw.setTargetAngle(closeRad, 1.0); }
    public void clawCloseLoose() { claw.setTargetAngle(closeLooseRad, 1.0); }
    public void clawOpen() { claw.setTargetAngle(openRad, 1.0); }

    public boolean inPosition() { return armRotation.inPosition() && clawRotation.inPosition(); }

    public boolean clawInPosition() { return claw.inPosition(); }

    public boolean armInPosition() { return armRotation.inPosition(); }
    public boolean clawRotInPosition() { return clawRotation.inPosition(); }
}
