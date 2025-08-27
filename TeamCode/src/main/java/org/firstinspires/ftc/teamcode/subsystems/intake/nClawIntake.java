package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.nDeposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.LLBlockDetectionPostProcessor;

@Config
public class nClawIntake {
    private final Robot robot;

    public final IntakeTurret intakeTurret;

    private final REVColorSensorV3 colorSensorV3;

    public final DigitalChannel intakeLight;

    private double intakeSetTargetPos;

    public static double transferClawRotation = 0, turretSearchRotation = 3.165;

    public static double restrictedHoverAngle = 1.0645, normalHoverAngle = 0.232, turretRetractedAngle = 2.4345, turretSearchAngle = 0.64, turretBufferStart = 1.2616, turretBufferEnd = 1.7797,turretTransferAngle = 2.4387, turretGrabAngle = -0.4557;
    public static double hoverAngle = normalHoverAngle;
    private long hoverStart = 0;
    private long lowerStart = 0;
    public static double hoverDelay = 150;
    public static double lowerDelay = 300;

    public static double turretTransferRotation = 3.165;

    public static double minExtension = 2; // What we require before giving full range of motion

    public static double transferExtension = 0, bufferExtension = 7;

    public static LLBlockDetectionPostProcessor.Block visionTargetBlock = null;
    public static double dodgeAngle = 0.4;

    private boolean submersibleIntake = false, safeConfirm = false;
    private boolean grab = false;
    private boolean sampleStatus = false;
    private boolean finishTransferRequest = false;
    private boolean extendRequest = false;
    //private boolean useCamera = true
    //public boolean manualGrab = false;
    public Pose2d target;
    public static int blockPickUpPSThreashold = 263;
    private int psReads = 0;
    private int consecutivePSPositives = 0;
    private Target targetType = Target.RELATIVE;
    private GrabMethod grabMethod = GrabMethod.CONTINUOUS_SEARCH_MG;
    private Pose2d known = null;
    private boolean autoEnableCamera = false;
    private boolean retryGrab = false;
    private int retryCounter = 0;

    private double manualTurretAngle, manualClawAngle;

    public enum Target {
        RELATIVE,
        GLOBAL,
        MANUAL
    }

    public enum GrabMethod {
        MANUAL_AIM(false, true),
        MANUAL_TARGET(false, true),
        CONTINUOUS_SEARCH_MG(true, true),
        SEARCH_HOVER_MG(true, true),
        AUTOGRAB(true, false);

        private final boolean useCamera, manualGrab;

        GrabMethod(boolean useCamera, boolean manualGrab) {
            this.useCamera = useCamera;
            this.manualGrab = manualGrab;
        }
    }

    public enum State {
        DODGE,
        SEARCH,
        HOVER,
        LOWER,
        GRAB_CLOSE,
        START_RETRACT,
        RELEASE_BUFFER,
        RETRACT,
        READY,
        TRANSFER_WAIT,
        TRANSFER_END,
        TEST
    }

    public State state = State.READY;

    public nClawIntake(Robot robot) {
        this.robot = robot;

        intakeTurret = new IntakeTurret(this.robot);

        intakeLight = robot.hardwareMap.get(DigitalChannel.class, "intakeLight");
        colorSensorV3 = robot.hardwareMap.get(REVColorSensorV3.class, "intakeClawColorSensor");
        colorSensorV3.configurePS(REVColorSensorV3.PSResolution.EIGHT, REVColorSensorV3.PSMeasureRate.m6p25s);
        colorSensorV3.sendControlRequest(new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.PROX_SENSOR_ENABLED)
        );
        intakeLight.setMode(DigitalChannel.Mode.OUTPUT);
        intakeLight.setState(false);

        intakeSetTargetPos = 15;

        target = new Pose2d(0, 0, 0);
    }

    public void update() {
        switch (state) {
            case DODGE:
                intakeTurret.setTurretArmTarget(dodgeAngle);

                if(extendRequest){
                    doExtend();
                    extendRequest = false;
                }

                if(safeConfirm){
                    if(grabMethod.useCamera){
                        state = State.SEARCH;
                    }else{
                        state = State.HOVER;
                    }
                }
                break;
            case SEARCH:
                aimAtKnown();

                if (intakeTurret.turretAngInPosition() && !robot.vision.isDetecting() && intakeTurret.intakeExtension.inPosition() && autoEnableCamera) {
                    manualEnableCamera();
                }

                intakeTurret.setTight(true);
                intakeTurret.setClawState(false);

                Log.i("CHECKING IT", intakeTurret.rotInPosition() + " rot in pos");
                switch (grabMethod) {
                    case CONTINUOUS_SEARCH_MG:
                        if (!grab)
                            break;
                    case SEARCH_HOVER_MG:
                    case AUTOGRAB:
                        LLBlockDetectionPostProcessor.Block b = robot.vision.getClosestValidBlock();
                        if (!(b != null && intakeTurret.inPosition() && intakeTurret.extendoInPosition()))
                            break;
                        visionTargetBlock = b;

                        hoverStart = System.currentTimeMillis();

                        robot.vision.stopDetection();
                        state = State.HOVER;
                        break;
                }
                break;

            case HOVER:
                aimAtTarget();
                intakeTurret.setTurretArmTarget(hoverAngle);
                intakeTurret.setTight(true);
                if (grabMethod != GrabMethod.MANUAL_AIM || intakeTurret.intakeExtension.inPosition(1) && intakeTurret.turretAngInPosition()) intakeTurret.setClawState(false);

                if (intakeTurret.inPosition(Math.toRadians(5)) && (grabMethod == GrabMethod.MANUAL_AIM || System.currentTimeMillis() - hoverStart > hoverDelay)) {
                    if (!grabMethod.manualGrab || grab) {
                        state = State.LOWER;
                        lowerStart = System.currentTimeMillis();
                    }
                }

                break;

            case LOWER: // Slam it down sometimes so we need to hover
                aimAtTarget();

                intakeTurret.setClawState(false);

                // everything in position before grabbing
                if (intakeTurret.inPosition(Math.toRadians(2)) && System.currentTimeMillis() - lowerStart >= lowerDelay) {
                    consecutivePSPositives = psReads = 0;
                    state = State.GRAB_CLOSE;
                    robot.vision.removeBlock(visionTargetBlock);
                }
                break;
            case GRAB_CLOSE:
                if (visionTargetBlock != null) Log.i("ERIC LOG", visionTargetBlock.getArea() + " is area of target block with x and y " + visionTargetBlock.getGlobalPose().getX() + " " + visionTargetBlock.getGlobalPose().getY());
                aimAtTarget();

                if (grabMethod == GrabMethod.MANUAL_AIM) {
                    sampleStatus = grab;
                    intakeTurret.setClawState(grab);
                    if (intakeTurret.clawInPosition()) intakeTurret.setTurretArmTarget(hoverAngle);
                    if (!grab) state = State.HOVER;
                    break;
                }

                if (psReads >= 35) {
                    if (Globals.RUNMODE != RunMode.TELEOP) {
                        retryCounter++;
                    }
                    if (!retryGrab) grab = false;
                    if (grabMethod.useCamera) {
                        state = State.SEARCH;
                        robot.vision.startDetection();
                        intakeLight.setState(true);
                    } else {
                        hoverStart = System.currentTimeMillis();
                        state = State.HOVER;
                    }
                    break;
                }

                int val = colorSensorV3.readPS();
                Log.e("colorSensorPS Value", val + "");
                if (val > blockPickUpPSThreashold) {
                    consecutivePSPositives++;
                } else
                    consecutivePSPositives = 0;
                psReads++;

                // grab
                grab = true;
                intakeTurret.setClawState(grab);

                // begin retract once grab finished
                if (consecutivePSPositives >= 20) {
                    sampleStatus = true;
                }

                if (sampleStatus && intakeTurret.clawInPosition()) {
                    known = null;
                    visionTargetBlock = null;
                    state = State.START_RETRACT;
                    robot.ndeposit.startTransfer();
                    intakeLight.setState(false);
                }

                break;
            case START_RETRACT:
                intakeTurret.intakeExtension.disableIgnoreKeepIn();
                // Get the arm in a proper angle for a full retract
                intakeTurret.setIntakeExtension(bufferExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretBufferStart);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setTight(false);
                intakeTurret.setClawState(grab);

                // if grab failed go back to search
                if (intakeTurret.turretAngInPosition(Math.toRadians(30)) && intakeTurret.turretRotInPosition(Math.toRadians(30))) {
                    // If we have a sample, transfer otherwise just retract into it
                    if (sampleStatus)
                        state = State.RELEASE_BUFFER;
                    else {
                        state = State.RETRACT;
                        grab = false;
                        if (robot.ndeposit.state == nDeposit.State.TRANSFER_BUFFER) robot.ndeposit.returnToIdle();
                    }
                }
                break;
            case RETRACT:
                // full retract into transfer
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretRetractedAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                // true grab -> holding a sample
                if (intakeTurret.extendoInPosition()) {
                    state = State.READY;
                    this.intakeLight.setState(false);
                }

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }
                break;
            case READY:
                // hold in start position, everything tucked in while moving so defense can be played. no sample ver
                intakeTurret.setIntakeExtension(0.0);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretRetractedAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setTight(true);
                intakeTurret.setClawState(false);

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }
                break;
            case RELEASE_BUFFER:
                intakeTurret.setIntakeExtension(minExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretBufferEnd, 0.6);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setTight(false);
                intakeTurret.setClawState(true);

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }

                if (intakeTurret.turretArm.inPosition()) state = State.TRANSFER_WAIT;

                break;
            case TRANSFER_WAIT:
                // hold in transfer position
                intakeTurret.setIntakeExtension(transferExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setTight(false);
                intakeTurret.setClawState(true);

                if (extendRequest) {
                    doExtend();
                    extendRequest = false;
                }
                // Complete transfer can only be called in TRANSFER_WAIT, must have everything correct
                // used to release intake grip on sample, should be called in deposit after the deposit has a firm grip
                // TODO: check endAffector.inPosition()
                else if (Globals.RUNMODE == RunMode.TELEOP ? finishTransferRequest && robot.ndeposit.isHolding() : finishTransferRequest && intakeTurret.inPosition() && intakeTurret.extendoInPosition() && robot.ndeposit.isHolding()) {
                    state = State.TRANSFER_END;
                    finishTransferRequest = false;
                    sampleStatus = false;
                }
                break;
            case TRANSFER_END:
                intakeTurret.setIntakeExtension(transferExtension);
                intakeTurret.setClawRotation(transferClawRotation);
                intakeTurret.setTurretArmTarget(turretTransferAngle);
                intakeTurret.setTurretRotation(turretTransferRotation);

                intakeTurret.setClawState(false);

                // once the grab is finished, send back to RETRACT. false grab changes from HOLD to READY
                // no need to worry about whacking stuff b/c both states require rotation to be in the turretTransferRot value
                if (robot.ndeposit.retractReady()) {
                    state = State.RETRACT;
                    grab = false;
                }
                break;

            case TEST:
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                break;
        }

        if (robot.vision.isDetecting()) {
            robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());
            robot.vision.setNewOrientation(intakeTurret.getTurretRotation() - Math.PI);
        }

        intakeTurret.update();
        updateTelemetry();
    }

    public int getRetryCounter() { return retryCounter; }
    public void resetRetryCounter() { retryCounter = 0; }
    public void forceDryCycle() { sampleStatus = true; }
    public void finishTransfer() { finishTransferRequest = state == State.TRANSFER_WAIT; }

    public void setKnownIntakePose(Pose2d k) {
        known = k.clone();
    }

    public void removeKnown() {
        known = null;
    }

    public void setTargetPose(Pose2d t) {
        target = t.clone();
    }

    public void extend() {
        extendRequest = true;
    }

    public boolean isOut() {
        return state == State.DODGE || state == State.SEARCH || state == State.HOVER || state == State.LOWER ||  state == State.GRAB_CLOSE;
    }

    public boolean isExtended() {
        return isOut() && intakeTurret.extendoInPosition();
    }

    public void retract() {
        if (state != State.READY) {
            this.state = State.START_RETRACT;
            intakeLight.setState(false);
            if (grab && grabMethod == GrabMethod.MANUAL_AIM) robot.ndeposit.startTransfer();
        }
    }

    public boolean isRetracted() {
        return state == State.READY;
    }

    public void setGrab(boolean state) {
        Log.i("JAMES", "setGrab " + state);
        grab = state;
    }

    // Confirm pickup
    public void confirmGrab() {
        sampleStatus = true;
    }

    public boolean hasSample() {
        return sampleStatus;
    }

    public boolean isTransferReady() {
        //RobotLog.e("TSPMO " + intakeTurret.inPosition() + " " + intakeTurret.extendoInPosition() + " " + (state == State.TRANSFER_WAIT));
        return intakeTurret.inPosition() && state == State.TRANSFER_WAIT;
    }

    /*public void release() {
        if (this.state == State.HOLD) {
            this.state = State.READY;
            grab = false;
        }
    }*/

    //public boolean dropFinished() {
    //    return state == State.SEARCH;
    //}

    public double getExtendoTargetPos() { return this.intakeSetTargetPos; }
    public void setExtendoTargetPos(double targetPos) {
        this.intakeSetTargetPos = Utils.minMaxClip(targetPos, 0, IntakeExtension.maxExtendoLength);
    }

    public double getManualTurretAngle() { return this.manualTurretAngle; }

    public void setManualTurretAngle(double targetPos) {
        while (targetPos < -1.8) targetPos += 1.8 * 2;
        while (targetPos > 1.8) targetPos -= 1.8 * 2;
        this.manualTurretAngle = targetPos;
    }

    public double getManualClawAngle() { return this.manualClawAngle; }

    public void setManualClawAngle(double targetPos) {
        while (targetPos < -1.8) targetPos += 1.8 * 2;
        while (targetPos > 1.8) targetPos -= 1.8 * 2;
        this.manualClawAngle = targetPos;
    }

    public double getIntakeLength() {
        return intakeTurret.intakeExtension.getLength();
    }

    public Vector2 getIntakeRelativeToRobot() {
        return new Vector2(
            getIntakeLength() + IntakeTurret.extendoOffset + Math.sin(intakeTurret.getTurretRotation() - Math.toRadians(90)) * IntakeTurret.turretLengthLL,
            -IntakeTurret.turretLengthLL * Math.cos(intakeTurret.getTurretRotation() - Math.toRadians(90))
        );
    }


    public void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : clawRotation angle", intakeTurret.getClawRotation());
        TelemetryUtil.packet.put("Intake : manualTurretAngle", manualTurretAngle);
        TelemetryUtil.packet.put("Intake : manualClawAngle", manualClawAngle);
        TelemetryUtil.packet.put("Intake : grab", grab);
        TelemetryUtil.packet.put("Intake : grabMethod", grabMethod);
        TelemetryUtil.packet.put("Intake : targetType", targetType);
        TelemetryUtil.packet.put("Intake : sampleStatus", sampleStatus);
        TelemetryUtil.packet.put("Intake : retryCounter", retryCounter);
        TelemetryUtil.packet.put("Intake : state", this.state);
        TelemetryUtil.packet.put("Intake : Target X", target.x);
        TelemetryUtil.packet.put("Intake : Target Y", target.y);
        TelemetryUtil.packet.put("Intake : Target Heading", target.heading);
        TelemetryUtil.packet.put("Intake : visionTargetBlock", visionTargetBlock);
        LogUtil.intakeState.set(this.state.toString());
    }

    public void setGrabMethod(GrabMethod grabMethod) {
        this.grabMethod = grabMethod;
    }

    public int readPS() {
        return colorSensorV3.readPS();
    }

    public void setTargetType(Target targetType) {
        this.targetType = targetType;
    }

    public void aimAtKnown() {
        Log.e("THIS THING IS KNOWN", known == null ? "null" : known.toString());
        if (known != null) {
            // Begin Search, dynamic correction
            Pose2d curr = robot.sensors.getOdometryPosition();
            double deltaX = (known.x - curr.x);
            double deltaY = (known.y - curr.y);

            double relX = Math.cos(curr.heading)*deltaX + Math.sin(curr.heading)*deltaY;
            double relY = -Math.sin(curr.heading)*deltaX + Math.cos(curr.heading)*deltaY;

            intakeTurret.extendTo(new Vector2(relX, relY));
            intakeTurret.setTurretArmTarget(turretSearchAngle);
            intakeTurret.setClawRotation(target.heading);
        } else {
            // Begin Search, just hold positions
            intakeTurret.setIntakeExtension(intakeSetTargetPos);
            intakeTurret.setClawRotation(target.heading);
            intakeTurret.setTurretArmTarget(turretSearchAngle);
            intakeTurret.setTurretRotation(turretSearchRotation);
        }
    }

    public void aimAtTarget() {
        TelemetryUtil.packet.put("ClawIntake : target", target);

        switch (targetType) {
            case RELATIVE: {
                intakeTurret.intakeAt(target);
                break;
            }
            case GLOBAL: {
                double deltaX = (target.x - robot.sensors.getOdometryPosition().x);
                double deltaY = (target.y - robot.sensors.getOdometryPosition().y);

                Canvas canvas = TelemetryUtil.packet.fieldOverlay();
                canvas.setFill("#c000c0");
                canvas.fillCircle(target.x, target.y, 1);

                // convert error into direction robot is facing
                intakeTurret.intakeAt(new Pose2d(
                    Math.cos(robot.sensors.getOdometryPosition().heading) * deltaX + Math.sin(robot.sensors.getOdometryPosition().heading) * deltaY,
                    -Math.sin(robot.sensors.getOdometryPosition().heading) * deltaX + Math.cos(robot.sensors.getOdometryPosition().heading) * deltaY,
                    target.heading - robot.sensors.getOdometryPosition().heading
                ));
                break;
            }
            case MANUAL: {
                intakeTurret.setIntakeExtension(intakeSetTargetPos);
                intakeTurret.setTurretRotation(Math.PI + manualTurretAngle);
                intakeTurret.setClawRotation(manualClawAngle);
                intakeTurret.setTurretArmTarget(turretGrabAngle);
                break;
            }
        }

        if (visionTargetBlock != null) {
            target = new Pose2d(
                visionTargetBlock.getX(),
                visionTargetBlock.getY(),
                -visionTargetBlock.getHeading()
            );
            targetType = Target.RELATIVE; // Kind of needed here or else its weird
        }
    }

    public void setRetryGrab(boolean retryGrab) { this.retryGrab = retryGrab; }

    public void setSubmersibleIntake(boolean b){
        submersibleIntake = b;
    }

    private void doExtend() {
        intakeTurret.intakeExtension.ignoreKeepIn();

        if(submersibleIntake){
            state = State.DODGE;
        } else if (grabMethod.useCamera) {
            state = State.SEARCH;
            robot.vision.setOffset(robot.nclawIntake.getIntakeRelativeToRobot());
        } else {
            visionTargetBlock = null;
            hoverStart = System.currentTimeMillis();
            state = State.HOVER;
        }
    }

    public void enableRestrictedHoldPos() {
        hoverAngle = restrictedHoverAngle;
    }

    public void disableRestrictedHoldPos() {
        hoverAngle = normalHoverAngle;
    }

    public void setAutoEnableCamera(boolean state) {
        autoEnableCamera = state;
    }

    public void manualEnableCamera() {
        robot.vision.startDetection();
        intakeLight.setState(true);
    }
}
