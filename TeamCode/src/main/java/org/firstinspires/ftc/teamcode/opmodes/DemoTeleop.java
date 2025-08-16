package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeTurret;
import org.firstinspires.ftc.teamcode.subsystems.intake.nClawIntake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

@Config
@TeleOp(name = "Demo Teleop")
public class DemoTeleop extends LinearOpMode {
    public static double slidesInc = 0.4;
    //public static double extendoInc = 0.4;
    public static double extendoXSpeed = 0.5;
    public static double extendoYSpeed = 0.35;
    public static double intakeClawRotationInc = 0.1;
    //public static double intakeTurretRotationInc = -0.1;
    public static double extensionPreset = 15;
    public static double transitionDelay = 750;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Globals.TESTING_DISABLE_CONTROL = false;
        Globals.hasSamplePreload = false;
        Globals.hasSpecimenPreload = false;

        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        // Gamepad 1
        ButtonToggle lb_1 = new ButtonToggle();
        ButtonToggle rb_1 = new ButtonToggle();
        ButtonToggle a_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle y_1 = new ButtonToggle();
        ButtonToggle lsb_1 = new ButtonToggle();
        ButtonToggle rsb_1 = new ButtonToggle();

        //Gamepad 2
        //ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        final double triggerThresh = 0.2;
        final double triggerHardThresh = 0.7;
        //double turretAngle = 0;
        //double clawAngle = 0;
        boolean speciMode = false;
        boolean intakeMode = false;
        boolean high = true;
        boolean autoGrab = false;
        boolean forceBrakePad = false;
        boolean manualBrake = false;

        final double opmodeStart = System.currentTimeMillis();

        robot.sensors.update();

        robot.ndeposit.arm.setArmRotation(0.02, 1);
        // robot.nclawIntake.setExtendoTargetPos(robot.sensors.getExtendoPos() <= 4 ? robot.sensors.getExtendoPos() : 12);
        robot.nclawIntake.setExtendoTargetPos(0.0);
        robot.nclawIntake.intakeTurret.setTurretArmTarget(nClawIntake.restrictedHoverAngle);
        robot.nclawIntake.intakeTurret.setTurretRotation(nClawIntake.turretTransferRotation);
        while (opModeInInit()) {
            if (System.currentTimeMillis() < opmodeStart + transitionDelay) robot.nclawIntake.state = nClawIntake.State.TEST;
            else if (robot.nclawIntake.state == nClawIntake.State.TEST) robot.nclawIntake.state = nClawIntake.State.RETRACT;
            robot.ndeposit.presetDepositHeight(speciMode, high, false);
            robot.update();
        }
        if (robot.nclawIntake.state == nClawIntake.State.TEST) robot.nclawIntake.state = nClawIntake.State.RETRACT;
        robot.nclawIntake.setTargetPose(new Pose2d(extensionPreset, 0, 0));
        robot.nclawIntake.setAutoEnableCamera(false);
        robot.nclawIntake.disableRestrictedHoldPos();

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        while (!isStopRequested()) {
            robot.update();

            if (autoGrab) {
                robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.AUTOGRAB);
                robot.nclawIntake.setTargetType(nClawIntake.Target.RELATIVE);
            } else {
                robot.nclawIntake.setGrabMethod(nClawIntake.GrabMethod.MANUAL_AIM);
                robot.nclawIntake.setTargetType(nClawIntake.Target.RELATIVE);
            }
            robot.nclawIntake.setRetryGrab(false);

            if (x_1.isClicked(gamepad1.x)) {
                // speciMode = !speciMode;
                intakeMode = false;
                robot.ndeposit.presetDepositHeight(speciMode, high, false);
//                if (speciMode) gamepad1.rumble(200);
//                else gamepad1.rumble(100);
            }
            if (b_1.isClicked(gamepad1.b)) {
                high = !high;
                robot.ndeposit.presetDepositHeight(speciMode, high, false);
            }

            if (y_1.isClicked(gamepad1.y)) {
                autoGrab = !autoGrab;
                if (autoGrab) gamepad1.rumble(200);
                else gamepad1.rumble(100);
            }
            if (lsb_1.isClicked(gamepad1.left_stick_button)) {
                if (robot.ndeposit.holdSlides) {
                    robot.ndeposit.holdSlides = false;
                    gamepad1.rumble(250);
                } else {
                    intakeMode = !intakeMode && !speciMode;
                    if (intakeMode) gamepad1.rumble(200);
                    else gamepad1.rumble(100);
                }
            }

            /*
            if (a_1.releasedAndNotHeldPreviously(gamepad1.a && !gamepad1.start, 200)) {
                manualBrake = !manualBrake;
                forceBrakePad = true;
                if (manualBrake) gamepad1.rumble(200);
                else gamepad1.rumble(100);
            }
            if (a_1.isHeld(gamepad1.a && !gamepad1.start, 200)) {
                if (forceBrakePad) gamepad1.rumble(250);
                manualBrake = false;
                forceBrakePad = false;
            }
            */

            if (lb_1.isClicked(gamepad1.left_bumper)) {
                if (speciMode) {
                    // Begin specimen grab
                    if (robot.ndeposit.state == nDeposit.State.IDLE) robot.ndeposit.startSpecimenIntake();
                        // Finish specimen grab
                    else if (robot.ndeposit.state == nDeposit.State.SPECIMEN_INTAKE_WAIT) robot.ndeposit.grab();
                    else if (robot.ndeposit.state == nDeposit.State.HOLD) robot.ndeposit.startSpecimenDeposit();
                    else {
                        robot.ndeposit.deposit();
                        robot.ndeposit.startSpecimenIntake();
                    }
                } else {
                    if (robot.ndeposit.state == nDeposit.State.SPECIMEN_INTAKE_WAIT) robot.ndeposit.returnToIdle();
                    else if (robot.ndeposit.isDepositingSample()) robot.ndeposit.deposit();
                        // Begin sample intake
                    else if (robot.nclawIntake.state == nClawIntake.State.READY || robot.nclawIntake.state == nClawIntake.State.TRANSFER_WAIT) {
                        robot.nclawIntake.extend();
                        intakeMode = true;
                        robot.nclawIntake.target.heading = robot.nclawIntake.target.y = 0;
                        //turretAngle = 0;
                        //clawAngle = 0;
                    }
                    // Manual retract
                    else robot.nclawIntake.retract();
                }
            }

            if (robot.nclawIntake.state == nClawIntake.State.START_RETRACT) intakeMode = false;

            // Deposit sample/speci. The deposit FSM takes care of which one
            if (robot.nclawIntake.isOut()) {
                rb_1.isClicked(gamepad1.right_bumper);
                robot.nclawIntake.setGrab(gamepad1.right_bumper);

                // robot.drivetrain.setBrakePad(robot.drivetrain.vdrive.mag() <= 0.05 && (intakeMode || Math.abs(gamepad1.right_stick_y) <= 0.05));
            } else {
                robot.drivetrain.setBrakePad(false);
                if (rb_1.isClicked(gamepad1.right_bumper)) {
                    if (robot.ndeposit.state == nDeposit.State.SPECIMEN_INTAKE_WAIT) robot.ndeposit.returnToIdle();
                    else if (robot.ndeposit.isHolding()) robot.ndeposit.deposit();
                    else {
                        robot.ndeposit.startSampleDeposit();
                        robot.nclawIntake.finishTransfer();
                        robot.ndeposit.finishTransfer();
                    }
                }
            }

            /*
            if (forceBrakePad) robot.drivetrain.setBrakePad(manualBrake);
            */

            if (intakeMode) {
                double t = robot.nclawIntake.intakeTurret.getTargetTurretRotation();
                robot.nclawIntake.setTargetPose(new Pose2d(
                        Utils.minMaxClip(robot.nclawIntake.target.x - gamepad1.right_stick_y * extendoXSpeed, 0, IntakeTurret.extendoOffset + IntakeExtension.maxExtendoLength + IntakeTurret.turretLengthTip * -Math.cos(t)),
                        Utils.minMaxClip(robot.nclawIntake.target.y - gamepad1.right_stick_x * extendoYSpeed, -IntakeTurret.turretLengthTip, IntakeTurret.turretLengthTip),
                        robot.nclawIntake.target.heading + intakeClawRotationInc * (gamepad1.left_trigger - gamepad1.right_trigger))
                );
            } else if (robot.ndeposit.state == nDeposit.State.SAMPLE_RAISE || robot.ndeposit.state == nDeposit.State.SAMPLE_WAIT) {
                // Manualy adjust the slides height during deposit
                double slidesControl1 = robot.drivetrain.smoothControls(-gamepad1.right_stick_y);
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() + slidesInc * slidesControl1);
            }

            //robot.nclawIntake.setManualClawAngle(clawAngle - turretAngle);
            //robot.nclawIntake.setManualTurretAngle(turretAngle);

            // Reset encoders in case something breaks
            if (rsb_1.isClicked(gamepad1.right_stick_button)) {
                if (gamepad1.left_trigger >= triggerHardThresh && gamepad1.right_trigger >= triggerHardThresh) {
                    robot.sensors.setOdometryPosition(0, 48, 0);
                    LogUtil.drivePositionReset = true;
                } else {
                    robot.sensors.hardwareResetSlidesEncoders();
                }
                gamepad1.rumble(250);
            }

            // Driving
            robot.drivetrain.intakeDriveMode = intakeMode;
            robot.drivetrain.drive(gamepad1);

            if (gamepad2.dpad_up) {
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() + slidesInc);
            } else if (gamepad2.dpad_down) {
                robot.ndeposit.setDepositHeight(robot.ndeposit.getDepositHeight() - slidesInc);
            }

            /*
            // hang
            if (b_2.isClicked(gamepad2.b && !gamepad2.start)) {
                robot.ndeposit.hangSafety = !robot.ndeposit.hangSafety;
                if (robot.ndeposit.hangSafety) {
                    robot.ndeposit.setDepositHeight(robot.ndeposit.slides.getLength());
                    robot.ndeposit.holdSlides = true;
                    gamepad2.rumble(200);
                } else {
                    gamepad2.rumble(100);
                }
            }

            int hangLeftDir = 0, hangRightDir = 0;
            //for (Gamepad gamepad : new Gamepad[]{gamepad1, gamepad2}) {
            if (gamepad1.dpad_up) { ++hangLeftDir; ++hangRightDir; }
            if (gamepad1.dpad_down) { --hangLeftDir; --hangRightDir; }
            if (gamepad1.dpad_left) { --hangLeftDir; ++hangRightDir; }
            if (gamepad1.dpad_right) { ++hangLeftDir; --hangRightDir; }
            //}
            if (-gamepad2.right_stick_y >= triggerThresh) ++hangLeftDir;
            if (-gamepad2.right_stick_y <= -triggerThresh) --hangLeftDir;
            if (-gamepad2.left_stick_y >= triggerThresh) ++hangRightDir;
            if (-gamepad2.left_stick_y <= -triggerThresh) --hangRightDir;
            if (gamepad2.y) { ++hangLeftDir; ++hangRightDir; }
            if (gamepad2.a) { --hangLeftDir; --hangRightDir; }

            if (hangLeftDir > 0) robot.hang.leftUp();
            else if (hangLeftDir < 0) robot.hang.leftPull();
            else robot.hang.leftOff();
            if (hangRightDir > 0) robot.hang.rightUp();
            else if (hangRightDir < 0) robot.hang.rightPull();
            else robot.hang.rightOff();

            if (gamepad2.right_bumper) robot.hang.l3Pull();
            else if (gamepad2.left_bumper) robot.hang.l3Up();
            else robot.hang.l3Off();
            if (gamepad2.right_trigger >= triggerHardThresh) {
                robot.ndeposit.hangState = nDeposit.HangState.PULL;
                robot.hang.l3Pull();
            } else if (gamepad2.left_trigger >= triggerHardThresh || gamepad1.touchpad) {
                robot.ndeposit.hangState = nDeposit.HangState.OUT;
            }


            // Used to keep extendo in during hang
            if (robot.ndeposit.hangSafety != (gamepad1.back || gamepad2.back)) robot.nclawIntake.intakeTurret.intakeExtension.forcePullIn();

            telemetry.addData("* speciMode", speciMode);
            telemetry.addData("* intakeMode", intakeMode);
            telemetry.addData("high", high);
            telemetry.addData("autoGrab", autoGrab);
            telemetry.addData("* holdSlides", robot.ndeposit.holdSlides);
            telemetry.addData("* hangSafety", robot.ndeposit.hangSafety);
            telemetry.addData("* ClawIntake target", robot.nclawIntake.target);
            telemetry.addData("ClawIntake isOut", robot.nclawIntake.isOut());
            telemetry.addData("Extendo current length", robot.sensors.getExtendoPos());
            telemetry.addData("Extendo target pos", robot.nclawIntake.getExtendoTargetPos());
            telemetry.addData("Slides current length", robot.sensors.getSlidesPos());
            telemetry.addData("Deposit height", robot.ndeposit.getDepositHeight());
            //telemetry.addData("claw angle", clawAngle);
            //telemetry.addData("turret angle", turretAngle);
            //telemetry.addData("isRed", Globals.isRed);
            */

            telemetry.update();
        }
    }
}
