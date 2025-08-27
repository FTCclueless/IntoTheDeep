package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp(name = "B. IntakeWithLL")
@Config
public class IntakeWithLL extends LinearOpMode {
    public static boolean grab = false;
    public static boolean useKnown = false;
    public static double keepX = 5;
    public static double keepY = 3;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetH = 0;
    public static nClawIntake.GrabMethod grabMethod = nClawIntake.GrabMethod.SEARCH_HOVER_MG;
    public static boolean setTarget = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = false;
        Robot robot = new Robot(hardwareMap);
        LogUtil.init();

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        robot.nclawIntake.setAutoEnableCamera(true);
        waitForStart();

        //robot.nclawIntake.extend();
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();

        robot.nclawIntake.setTargetType(nClawIntake.Target.GLOBAL);
        robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_TARGET);

        while (opModeIsActive()) {
            robot.nclawIntake.setGrabMethod(grabMethod);
            //robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());

/*
            TelemetryUtil.packet.put("blockPos.x", blockPos.x);
            TelemetryUtil.packet.put("blockPos.y", blockPos.y);
            TelemetryUtil.packet.put("blockPos.heading", blockPos.heading);
            RobotLog.e("blockPos.heading " + Math.toDegrees(blockPos.heading));
*/

            Canvas c = TelemetryUtil.packet.fieldOverlay();

            if (y1.isClicked(gamepad1.y)) robot.nclawIntake.extend();

            if (x1.isClicked(gamepad1.x)) robot.ndeposit.startSampleDeposit();

            if (a1.isClicked(gamepad1.a)) robot.ndeposit.deposit();

            if (b1.isClicked(gamepad1.b)) robot.nclawIntake.retract();

            double intakeControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
            //robot.nclawIntake.setIntakeLength(robot.nclawIntake.getIntakeTargetPos() + 0.4 * intakeControl1);
            if (!useKnown) {
                robot.nclawIntake.removeKnown();
                robot.nclawIntake.setExtendoTargetPos(10);
            } else {
                robot.nclawIntake.setKnownIntakePose(new Pose2d(keepX, keepY, 0));
            }

            if (setTarget) {
                robot.nclawIntake.setTargetPose(new Pose2d(targetX, targetY, targetH));
                setTarget = false;
            }

            robot.nclawIntake.setGrab(grab);

            /*if (setExtend) {
                robot.vision.startDetection();
                robot.nclawIntake.extend();
                setExtend = false;
            }

            if (setRetract) {
                robot.vision.stopDetection();
                robot.nclawIntake.retract();
                setRetract = false;
            }

            if (transferRequest) {
                robot.ndeposit.startTransfer();
                transferRequest = false;
            }*/

            if (robot.nclawIntake.isTransferReady() && robot.ndeposit.isTransferReady()) {
                robot.nclawIntake.finishTransfer();
                robot.ndeposit.finishTransfer();
            }

            /*if (setGrab) {
                robot.nclawIntake.setTargetPose(new Pose2d(
                        blockPos.x,
                        blockPos.y,
                        AngleUtil.mirroredClipAngleTolerence(-blockPos.heading, Math.toRadians(20))
                ));
                robot.nclawIntake.grab(grab);
                setGrab = false;
            }*/

            //robot.nclawIntake.setIntakeLength(extensionLength);
            robot.drivetrain.drive(gamepad1);

            robot.update();
        }
    }
}
