package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        Vision vision = new Vision(hardwareMap, telemetry, false, false, false);
        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        Globals.RUNMODE = RunMode.TESTER;
        ButtonToggle bty = new ButtonToggle();

        waitForStart();

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));


        File file = AppUtil.getInstance().getSettingsFile("deceldata.csv");
        FileWriter fw;
        try {
            fw = new FileWriter(file);
        } catch (IOException e) {
            System.out.println("BAD BAD BAD BAD BAD");
            return;
        }

        while(!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);
            Pose2d pos = robot.drivetrain.getPoseEstimate();
            TelemetryUtil.packet.put("x: ", pos.x);
            TelemetryUtil.packet.put("y: ", pos.y);
            TelemetryUtil.packet.put("heading: ", pos.heading);

            robot.update();

            String buffer = "";

            try {
                fw.write(buffer);
            } catch (IOException e) {
                System.out.println("bad :(");
                return;
            }
            if (bty.isClicked(gamepad1.y)) {
                try {
                    fw.flush();
                } catch (IOException e) {
                    System.out.println("BAD BAD BAD BAD BAD");
                }
            }
        }

    }
}
