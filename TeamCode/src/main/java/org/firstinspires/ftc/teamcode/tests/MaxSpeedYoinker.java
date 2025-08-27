package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;

@TeleOp
public class MaxSpeedYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = false;
        Robot robot = new Robot(hardwareMap);
        double maxXSpeed = 0;
        double maxYSpeed = 0;
        double maxHeadingSpeed = 0;
        double maxMag = 0;

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);
            Pose2d velocity = robot.sensors.getVelocity();
            Vector3 sV = new Vector3(
                    velocity.x,
                    velocity.y,
                    velocity.heading * Globals.ROBOT_WIDTH / 2
            );

            if (sV.x > maxXSpeed) {
                maxXSpeed = sV.x;
            }
            if (sV.y > maxYSpeed) {
                maxYSpeed = sV.y;
            }
            if (sV.z > maxHeadingSpeed) {
                maxHeadingSpeed = sV.z;
            }
            if (sV.getMag() > maxMag) {
                maxMag = sV.getMag();
            }

            TelemetryUtil.packet.put("maxXSpeed", maxXSpeed);
            TelemetryUtil.packet.put("maxYSpeed", maxYSpeed);
            TelemetryUtil.packet.put("maxHeadingSpeed", maxHeadingSpeed);
            TelemetryUtil.packet.put("maxMag", maxMag);

            robot.update();
        }
    }
}
