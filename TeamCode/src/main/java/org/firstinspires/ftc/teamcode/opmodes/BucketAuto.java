package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.PathChainAutoOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "Bucket Sample")
public class BucketAuto extends PathChainAutoOpMode {

    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;

    private final Pose startPose   = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose   = new Pose(17, 128, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(20, 122, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(24, 128, Math.toRadians(20));
    private final Pose parkPose        = new Pose(62, 97, Math.toRadians(315));
    private final Pose parkControlPose = new Pose(64.5, 116, Math.toRadians(270));

    private PathChain scorePreload;
    private PathChain intake1, intake2, intake3;
    private PathChain score1, score2, score3;
    private PathChain parkChain;

    @Override
    protected void buildPathChains() {
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                /*
                .addParametricCallback(0.6, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1,   () -> run(motorActions.extendo.setTargetPosition(200)))
                */
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                /*
                .addParametricCallback(0.6, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1,   () -> run(motorActions.extendo.setTargetPosition(350)))
                */
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                /*
                .addParametricCallback(0.6, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(1,   () -> run(motorActions.extendo.setTargetPosition(400)))
                */
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                /*
                .addParametricCallback(0,   () -> run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(0),
                        motorActions.outtakeSampleAuto()
                )))
                */
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                /*
                .addParametricCallback(0,   () -> run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(0),
                        motorActions.outtakeSampleAuto()
                )))
                */
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                /*
                .addParametricCallback(0,   () -> run(motorActions.spin.slow()))
                .addParametricCallback(0.3, () -> run(new SequentialAction(
                        motorActions.lift.waitUntilFinished(0),
                        motorActions.outtakeSampleAuto()
                )))
                */
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePose)))
                /*
                .addParametricCallback(0, () -> run(motorActions.outtakeSampleAuto()))
                */
                .build();

        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)))
                .build();
    }

    @Override
    protected void buildTaskList() {
        tasks.clear();

        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.5)
                .addWaitAction(
                        () -> motorControl.lift.closeEnough(770)
                    /*
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                    */
                )
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(preloadTask);

        PathChainTask pickup1Task = new PathChainTask(intake1, 0.5)
                .setMaxWaitTime(1.25)
                /*
                .addWaitAction(0.5, motorActions.extendo.setTargetPosition(500))
                .addWaitAction(1,   motorActions.intakeGrabUntil(Enums.DetectedColor.UNKNOWN))
                */
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup1Task);

        PathChainTask score1Task = new PathChainTask(score1, 0.5)
                /*
                .addWaitAction(
                    () -> motorControl.lift.closeEnough(770),
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                )
                */
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score1Task);

        PathChainTask pickup2Task = new PathChainTask(intake2, 0.5)
                .setMaxWaitTime(1.25)
                /*
                .addWaitAction(
                    0,
                    () -> motorControl.getDetectedColor() == Enums.DetectedColor.UNKNOWN,
                    motorActions.extendo.setTargetPosition(500)
                )
                .addWaitAction(1, motorActions.intakeGrabUntil(Enums.DetectedColor.UNKNOWN))
                */
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup2Task);

        PathChainTask score2Task = new PathChainTask(score2, 0.5)
                /*
                .addWaitAction(
                    () -> motorControl.lift.closeEnough(770),
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                )
                */
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score2Task);

        PathChainTask pickup3Task = new PathChainTask(intake3, 0.5)
                .setMaxWaitTime(1.25)
                /*
                .addWaitAction(1, motorActions.intakeGrabUntil(Enums.DetectedColor.UNKNOWN))
                */
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickup3Task);

        PathChainTask score3Task = new PathChainTask(score3, 0.5)
                /*
                .addWaitAction(
                    () -> motorControl.lift.closeEnough(770),
                    new SequentialAction(
                        motorActions.outtakeLinkage.Retract(),
                        new SleepAction(0.4),
                        motorActions.outtakeClaw.Open(),
                        motorActions.intakePivot.Transfer(),
                        motorActions.intakeArm.Transfer()
                    )
                )
                */
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(770));
        tasks.add(score3Task);
    }

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
        follower.followPath((PathChain) task.pathChain, true);
    }

    @Override
    public void init() {
        super.init();
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
        limelight     = new MotorControl.Limelight(hardwareMap, telemetry);
        follower      = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        /*
        run(motorActions.outtakeClaw.Close());
        */

        buildPathChains();
        buildTaskList();
    }

    @Override
    public void start() {
        super.start();
        currentTaskIndex = 0;
        taskPhase        = 0;
        pathTimer.resetTimer();

        /*
        run(motorActions.intakePivot.Transfer());
        run(motorActions.outtakeLinkage.Extend());
        run(motorActions.intakeArm.Transfer());
        run(motorActions.outtakeSampleAuto());
        */
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        runTasks();
        motorControl.update();

        telemetry.addData("Task Index", currentTaskIndex + "/" + tasks.size());
        telemetry.addData("Phase",      (taskPhase == 0) ? "DRIVE" : "WAIT");
        telemetry.addData("T Value",    follower.getCurrentTValue());
        telemetry.addData("Wait Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Running Actions", runningActions.size());
        telemetry.update();
    }
}
