package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
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
@Autonomous(name = "spiketest")
public class spiketest extends PathChainAutoOpMode {

    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private Timer opModeTimer;

    private final Pose START_POSE = new Pose(9, 58, Math.toRadians(0));
    private final Pose PRELOAD_POSE = new Pose(40, 72, Math.toRadians(0));
    private final Pose PARK_POSE = new Pose(12, 60, Math.toRadians(250));
    private final Pose SCORE_POSE_SAMPLE = new Pose(17, 128, Math.toRadians(315));
    private final Pose SPIKE_MARK_ONE = new Pose(20, 124, Math.toRadians(0));
    private final Pose SPIKE_MARK_TWO = new Pose(22, 130, Math.toRadians(0));

    private PathChain preloadChain;
    private PathChain parkChain;
    private PathChain scoreSampleChain;
    private PathChain spikeMarkOneChain;
    private PathChain returnFromSpikeOneChain;
    private PathChain spikeMarkTwoChain;
    private PathChain returnFromSpikeTwoChain;

    @Override
    protected void buildPathChains() {

        preloadChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(START_POSE), new Point(PRELOAD_POSE)))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), PRELOAD_POSE.getHeading())
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakePivot.Transfer(),
                        motorActions.outtakeSpecimen())))
                .addParametricCallback(.099, () -> motorActions.depositSpecimen())
                .build();

        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PRELOAD_POSE), new Point(PARK_POSE)))
                .setLinearHeadingInterpolation(PRELOAD_POSE.getHeading(), PARK_POSE.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .addParametricCallback(0, () -> run(motorActions.outtakeTransfer()))
                .addParametricCallback(0.1, () -> run(new SequentialAction(
                        motorActions.outtakeTransfer(),
                        motorActions.intakeArm.Grab(),
                        motorActions.intakePivot.Grab(),
                        motorActions.spin.eat())))
                .addParametricCallback(0.6, () -> run(motorActions.extendo.setTargetPosition(550)))
                .build();

        scoreSampleChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PARK_POSE), new Point(SCORE_POSE_SAMPLE)))
                .setLinearHeadingInterpolation(PARK_POSE.getHeading(), SCORE_POSE_SAMPLE.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0, () -> run(motorActions.outtakeTurret.up()))
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0.1, () -> run(motorActions.outtakeSample()))
                .addParametricCallback(0.4, () -> run(motorActions.outtakeTurret.down()))
                .addParametricCallback(0.9, () -> run(motorActions.extendo.setTargetPosition(200)))
                .addParametricCallback(1, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .build();


        spikeMarkOneChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SCORE_POSE_SAMPLE), new Point(SPIKE_MARK_ONE)))
                .setLinearHeadingInterpolation(SCORE_POSE_SAMPLE.getHeading(), SPIKE_MARK_ONE.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(1, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .build();

        returnFromSpikeOneChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SPIKE_MARK_ONE), new Point(SCORE_POSE_SAMPLE)))
                .setLinearHeadingInterpolation(SPIKE_MARK_ONE.getHeading(), SCORE_POSE_SAMPLE.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0.3, () -> run(motorActions.outtakeSampleAuto()))
                .build();
        spikeMarkTwoChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SCORE_POSE_SAMPLE), new Point(SPIKE_MARK_TWO)))
                .setLinearHeadingInterpolation(SCORE_POSE_SAMPLE.getHeading(), SPIKE_MARK_TWO.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0.1, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1, () -> run(motorActions.extendo.setTargetPosition(400)))
                .build();

        returnFromSpikeTwoChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SPIKE_MARK_TWO), new Point(SCORE_POSE_SAMPLE)))
                .setLinearHeadingInterpolation(SPIKE_MARK_TWO.getHeading(), SCORE_POSE_SAMPLE.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0.1, () -> run(motorActions.outtakeSampleAuto()))
                .addParametricCallback(0.4, () -> run(motorActions.outtakeTurret.down()))
                .build();
    }

    // -------- Override buildTaskList() --------
    @Override
    protected void buildTaskList() {
        tasks.clear();

        tasks.add(new PathChainTask(preloadChain, 0.1)
                .addWaitAction(0.05, motorActions.depositSpecimen()));

        tasks.add(new PathChainTask(parkChain, 0.1));

        tasks.add(new PathChainTask(scoreSampleChain, 0.2)
                .addWaitAction(0.1, motorActions.outtakeTransfer()));

        tasks.add(new PathChainTask(spikeMarkOneChain, 0.1)
                .addWaitAction(0.1, motorActions.extendo.setTargetPosition(350)));

        tasks.add(new PathChainTask(returnFromSpikeOneChain, 0.2)
                .addWaitAction(() -> motorControl.lift.closeEnough(760),
                        new SequentialAction(
                                motorActions.outTakeLinkage.sample(),
                                new SleepAction(0.35),
                                motorActions.outtakeTransfer()
                        ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(760)));

        tasks.add(new PathChainTask(spikeMarkTwoChain, 0.1));

        tasks.add(new PathChainTask(returnFromSpikeTwoChain, 0.2)
                .addWaitAction(() -> motorControl.lift.closeEnough(760),
                        new SequentialAction(
                                motorActions.outTakeLinkage.sample(),
                                new SleepAction(0.35),
                                motorActions.outtakeTransfer()
                        ))
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(760)));

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

        follower.setStartingPose(START_POSE);
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
