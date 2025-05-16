package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * SpecimenAuto is an autonomous OpMode that uses a series of PathChainTasks.
 * It extends PathChainAutoOpMode so that you only need to override buildPathChains() and buildTaskList(),
 * plus the dummy path-follower methods.
 */
@Autonomous(name = "specgiveonly")
public class specgiveonly extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose = new Pose(9, -7.3, Math.toRadians(0));
    private final Pose scorePose = new Pose(38, 69, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(28, 19, Math.toRadians(309));
    private final Pose pickup2Pose = new Pose(27, 8, Math.toRadians(309));
    private final Pose pickup3Pose = new Pose(27, 1, Math.toRadians(310));
    private final Pose depositPose = new Pose(23, 30, Math.toRadians(150));

    private final Pose intake = new Pose(12.3, 40, Math.toRadians(0));
    private final Pose scoreControl1 = new Pose(30, 68, Math.toRadians(0));

    private final Pose finalpark = new Pose(12, 62, Math.toRadians(270));


    // -------- PathChains --------
    private PathChain grabPickup1, grabPickup2, grabPickup3;
    private PathChain depositHP1, depositHP2, depositHP3;

    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.3, ()->run(motorActions.extendo.setTargetPosition(400)))
                .addParametricCallback(0.4, () -> run(new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.intakePivot.Grab(),
                        motorActions.intakeArm.Grab())))
                .addParametricCallback(0.8, ()->run(motorActions.extendo.setTargetPosition(500)))

                .build();

        depositHP1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup1Pose),
                        new Point(depositPose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), depositPose.getHeading())
                .addParametricCallback(0, ()->run(motorActions.extendo.setTargetPosition(0)))
                .addParametricCallback(0.95, ()->run(motorActions.extendo.setTargetPosition(300)))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(depositPose),
                        new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0, () -> motorControl.spin.setPower(-0.5))
                .addParametricCallback(0.1, ()->run(motorActions.extendo.setTargetPosition(0)))
                .addParametricCallback(0.4, ()->run(motorActions.extendo.setTargetPosition(100)))
                .addParametricCallback(0.8, ()->run(motorActions.spin.eat()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0.94, ()->run(motorActions.extendo.setTargetPosition(450)))
                .build();

        depositHP2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup2Pose),
                        new Point(depositPose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), depositPose.getHeading())
                .setPathEndTValueConstraint(1)
                .addParametricCallback(0.01, ()->run(motorActions.extendo.setTargetPosition(0)))
                .addParametricCallback(0.85, ()->run(motorActions.extendo.setTargetPosition(500)))

                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(depositPose),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0, () -> motorControl.spin.setPower(-0.5))
                .addParametricCallback(0, ()->run(motorActions.extendo.setTargetPosition(0)))
                .addParametricCallback(0.4, ()->run(motorActions.extendo.setTargetPosition(200)))
                .addParametricCallback(0.8, ()->run(motorActions.spin.eat()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0.9, ()->run(motorActions.extendo.setTargetPosition(500
                )))
                .build();


        depositHP3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickup3Pose),
                        new Point(depositPose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), depositPose.getHeading())
                .setPathEndTValueConstraint(1)
                .addParametricCallback(0.1, ()->run(motorActions.extendo.setTargetPosition(0)))
                .addParametricCallback(0.93, ()->run(motorActions.extendo.setTargetPosition(520)))
                .addParametricCallback(0.99, ()->run(motorActions.spin.slowpoop()))
                .build();





    }

    // -------- Override buildTaskList() --------
    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask pickUpTask1 = new PathChainTask(grabPickup1, 0.1);

        tasks.add(pickUpTask1);


        PathChainTask depositTask1 = new PathChainTask(depositHP1, 0.2)
                .addWaitAction(0.01, motorActions.spin.slowpoop());;

        tasks.add(depositTask1);

        // Pickup task 2.
        PathChainTask pickUpTask2 = new PathChainTask(grabPickup2, 0.1);
        tasks.add(pickUpTask2);

        // Deposit task 2.
        PathChainTask depositTask2 = new PathChainTask(depositHP2, 0.22)
                .addWaitAction(0.01, motorActions.spin.slowpoop());
        tasks.add(depositTask2);

        // Pickup task 3.
        PathChainTask pickUpTask3 = new PathChainTask(grabPickup3, 0.15);
        tasks.add(pickUpTask3);

        // Deposit task 3.
        PathChainTask depositTask3 = new PathChainTask(depositHP3, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(0))
                .addWaitAction(0.01, motorActions.spin.poop());
        tasks.add(depositTask3);

    }

    // -------- Override dummy follower methods --------
    @Override
    protected boolean isPathActive() {
        return follower.isBusy();
    }

    @Override
    protected double getCurrentTValue() {
        return follower.getCurrentTValue();
    }

    @Override
    protected void startPath(PathChainTask task) {
        // Cast the task's pathChain to PathChain and command the follower to follow it.
        follower.followPath((PathChain) task.pathChain, false);
    }

    // -------- Standard OpMode Lifecycle Methods --------
    @Override
    public void init() {
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);
        run(motorActions.outTakeClaw.Close());

        // Build the paths and tasks.
        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        currentTaskIndex = 0;
        taskPhase = 0;
        pathTimer.resetTimer();
        run(motorActions.intakePivot.Transfer());
        run(motorActions.intakeArm.Intake());
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase", (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.update();
    }
}
