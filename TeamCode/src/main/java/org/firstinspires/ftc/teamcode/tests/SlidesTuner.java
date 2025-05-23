package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class SlidesTuner extends LinearOpMode {
    public static double length = 0;

    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        waitForStart();

        while (!isStopRequested()) {
            robot.ndeposit.state = nDeposit.State.TEST;
            robot.ndeposit.setDepositHeight(length);

            //TelemetryUtil.packet.put("Slides: Error", targetSlidesHeight - robot.ndeposit.slides.getLength());
            //TelemetryUtil.packet.put("Slides: Position", robot.ndeposit.slides.getLength());
            //TelemetryUtil.packet.put("Slides: Target Position", targetSlidesHeight);
            TelemetryUtil.packet.put("motor check0", motor.getCurrentPosition());
            TelemetryUtil.packet.put("motor check1", motor1.getCurrentPosition());

            robot.update();
        }
    }
}
