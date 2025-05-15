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
@Autonomous(name = "5+1")
public class Funtime extends PathChainAutoOpMode {

    // -------- Hardware & Helper Fields --------
    private Follower follower;
    private MotorActions motorActions;
    private MotorControl motorControl;
    private MotorControl.Limelight limelight;
    private Timer opModeTimer;  // additional timer if desired

    // -------- Poses --------
    private final Pose startPose = new Pose(9, 58, Math.toRadians(0));
    private final Pose preloadPose = new Pose(40, 72, Math.toRadians(0));
    private final Pose scorePose = new Pose(41, 69, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(41, 72, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(41, 70, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(41, 69, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(41, 68, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(28, 45, Math.toRadians(311));
    private final Pose pickup1Control = new Pose(22, 76, Math.toRadians(311));
    private final Pose pickup2Pose = new Pose(26, 35, Math.toRadians(315));
    private final Pose pickup3Pose = new Pose(30, 30, Math.toRadians(305));
    private final Pose depositPose1 = new Pose(25, 44, Math.toRadians(250));
    private final Pose depositPose2 = new Pose(25, 40, Math.toRadians(250));
    private final Pose intake = new Pose(11, 40, Math.toRadians(0));
    private final Pose intakeControl1 = new Pose(30, 65, Math.toRadians(0));
    private final Pose intakeControl2 = new Pose(5, 74, Math.toRadians(0));
    private final Pose intakeControl3 = new Pose(30, 34, Math.toRadians(0));
    private final Pose parkPose = new Pose(11, 64, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(12, 74, Math.toRadians(300));
    private final Pose sampleScorePose   = new Pose(12, 122, Math.toRadians(280));

    private final Pose finalpark = new Pose(12, 62, Math.toRadians(270));

    // -------- PathChains --------
    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3;
    private PathChain depositHP1, depositHP2, depositHP3;
    private PathChain intake1, intake2, intake3, intake4;
    private PathChain  score1, score2, score3, score4;
    private PathChain pickupsample;
    private PathChain finalparking;
    private PathChain ScoreBasket;

    // -------- Override buildPathChains() --------
    @Override
    protected void buildPathChains() {


        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(new SequentialAction(
                        motorActions.intakePivot.Transfer(),
                        //motorActions.intakeArm.Extended(),
                        motorActions.outtakeSpecimen())))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(pickup1Control),
                        new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.6, ()->run(motorActions.extendo.setTargetPosition(300)))
                .addParametricCallback(0.4, () -> run(new SequentialAction(
                        motorActions.spin.eat(),
                        motorActions.intakeArm.Grab())))

                .build();

        depositHP1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup1Pose),
                        new Point(depositPose1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), depositPose1.getHeading())
                .addParametricCallback(0.4, ()->run(motorActions.extendo.setTargetPosition(300)))
                .addParametricCallback(0.8, () -> motorControl.spin.setPower(-1.0))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(depositPose1.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.4, ()->run(motorActions.extendo.setTargetPosition(200)))
                .addParametricCallback(0.95, ()->run(motorActions.extendo.setTargetPosition(500)))

                .build();

        depositHP2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(pickup2Pose),
                        new Point(depositPose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), depositPose2.getHeading())
                .addParametricCallback(0.8, () -> motorControl.spin.setPower(-1.0))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(depositPose2.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.05, ()->run(motorActions.extendo.setTargetPosition(300)))
                .addParametricCallback(0.8, ()->run(motorActions.extendo.setTargetPosition(500)))
                .build();


        depositHP3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickup3Pose),
                        new Point(depositPose2)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), depositPose2.getHeading())
                .addParametricCallback(0.8, () -> motorControl.spin.setPower(-1.0))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(depositPose1),
                        //new Point(intakeControl3),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(depositPose1.getHeading()),
                        Math.toRadians(intake.getHeading()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .addParametricCallback(0, () -> run(new ParallelAction(motorActions.outtakeTurret.up(),motorActions.intakeSpecimen())))
                .build();


        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(scorePose1)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose1.getHeading()))
                .addParametricCallback(0.6, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        //motorActions.intakeArm.Extended(),
                        motorActions.outtakeTurret.up()
                )))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0, () -> motorControl.spin.setPower(0))
                .build();
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose1),
                        new Point(intakeControl1),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()),
                        Math.toRadians(intake.getHeading()))
                .addParametricCallback(0, () -> motorActions.intakeArm.Up())
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(6)
                .addParametricCallback(0.2, () -> run(new ParallelAction(motorActions.intakeSpecimen(), motorActions.outtakeTurret.down())))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(scorePose2)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose2.getHeading()))
                .addParametricCallback(0.6, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        //motorActions.intakeArm.Extended(),
                        motorActions.outtakeTurret.up()
                )))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .build();


        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose2),
                        new Point(intakeControl1),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()),
                        Math.toRadians(intake.getHeading()))
                .addParametricCallback(0, () -> motorActions.intakeArm.Up())
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(6)
                .addParametricCallback(0.2, () -> run(new ParallelAction(motorActions.intakeSpecimen(), motorActions.outtakeTurret.down())))
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(scorePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose3.getHeading()))
                .addParametricCallback(0.6, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        //motorActions.intakeArm.Extended(),
                        motorActions.outtakeTurret.up())))
                .setZeroPowerAccelerationMultiplier(7)
                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .addParametricCallback(0.5, () -> motorControl.spin.setPower(0))
                .build();

        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose3),
                        new Point(intakeControl1),
                        new Point(intake)))
                .setLinearHeadingInterpolation(Math.toRadians(scorePose.getHeading()),
                        Math.toRadians(intake.getHeading()))
                .addParametricCallback(0, () -> motorActions.intakeArm.Up())
                .addParametricCallback(0.2, () -> motorControl.spin.setPower(0))
                .setZeroPowerAccelerationMultiplier(6)
                .addParametricCallback(0.2, () -> run(new ParallelAction(motorActions.intakeSpecimen(), motorActions.outtakeTurret.down())))
                .build();


        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(intake),
                        new Point(intakeControl1),
                        new Point(scorePose4)))
                .setLinearHeadingInterpolation(Math.toRadians(intake.getHeading()),
                        Math.toRadians(scorePose4.getHeading()))
                .addParametricCallback(0.6, () -> run(new ParallelAction(
                        motorActions.intakePivot.Transfer(),
                        //motorActions.intakeArm.Extended(),
                        motorActions.outtakeTurret.up())))
                .setZeroPowerAccelerationMultiplier(7)

                .addParametricCallback(0, () -> run(motorActions.outtakeSpecimen()))
                .build();



        pickupsample = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose4),
                        new Point(parkControlPose),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .addParametricCallback(0.7, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(0.8, () -> run(motorActions.extendo.setTargetPosition(400)))
                .build();

        ScoreBasket = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(parkPose),      // Start from the park position
                        //new Point(parkControlPose), // Intermediate control point (optional)
                        new Point(sampleScorePose)))     // End at scorePose
                .setLinearHeadingInterpolation(parkPose.getHeading(), sampleScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(4) // Adjust acceleration to control speed

                .addParametricCallback(0, () -> run(motorActions.outtakeTurret.up()))
                .addParametricCallback(0, () -> run(motorActions.intakeTransfer()))
                .addParametricCallback(0.2, () -> run(motorActions.outtakeSampleAuto()))
                //.addParametricCallback(0.7, () -> run(motorActions.extendo.setTargetPosition(200)))
                //.addParametricCallback(1 , () -> run(motorActions.drop()))// Run actions if needed
                .build();

        finalparking = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sampleScorePose),
                        new Point(finalpark)))
                .setLinearHeadingInterpolation(sampleScorePose.getHeading(), finalpark.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0.1, () -> run(motorActions.intakeGrabUntil(Enums.DetectedColor.YELLOW)))
                .addParametricCallback(0.2, () -> run(motorActions.extendo.setTargetPosition(400)))
                .build();

    }

    // -------- Override buildTaskList() --------
    @Override
    protected void buildTaskList() {
        tasks.clear();

        // Preload task.
        PathChainTask preloadTask = new PathChainTask(scorePreload, 0.1)
                .addWaitAction(0.05, motorActions.depositSpecimen());
        tasks.add(preloadTask);

        PathChainTask pickUpTask1 = new PathChainTask(grabPickup1, 0.1)
                .setMaxWaitTime(0.05)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(500))
                .addWaitAction(0.18, motorActions.extendo.setTargetPosition(450))
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);

        tasks.add(pickUpTask1);

        // Deposit task 1.
        PathChainTask depositTask1 = new PathChainTask(depositHP1, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(200));
        tasks.add(depositTask1);

        // Pickup task 2.
        PathChainTask pickUpTask2 = new PathChainTask(grabPickup2, 0.05)
                .setMaxWaitTime(0.2)
                .addWaitAction(0, new ParallelAction(
                        motorActions.intakeArm.Grab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.setTargetPosition(500)
                ))
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickUpTask2);

        // Deposit task 2.
        PathChainTask depositTask2 = new PathChainTask(depositHP2, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(0));
        tasks.add(depositTask2);

        // Pickup task 3.
        PathChainTask pickUpTask3 = new PathChainTask(grabPickup3, 0.1)
                .setMaxWaitTime(0.2)
                .addWaitAction(0, new ParallelAction(
                        motorActions.intakeArm.Grab(),
                        motorActions.spin.eat(),
                        motorActions.extendo.setTargetPosition(550)
                ))
                .setWaitCondition(() -> motorControl.getDetectedColor() != Enums.DetectedColor.UNKNOWN);
        tasks.add(pickUpTask3);

        // Deposit task 3.
        PathChainTask depositTask3 = new PathChainTask(depositHP3, 0.2)
                .addWaitAction(0, motorActions.extendo.setTargetPosition(0));
        tasks.add(depositTask3);

        // Intake task 1.
        PathChainTask intakeTask1 = new PathChainTask(intake1, 0.2)
                .addWaitAction(0.05, motorActions.outTakeClaw.Close())
                .addWaitAction(0.1, motorActions.outtakeArm.Specimen());
        tasks.add(intakeTask1);

        // Score task 1.
        tasks.add(new PathChainTask(score1, 0.1)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Intake task 2.
        tasks.add(new PathChainTask(intake2, 0.2)
                .addWaitAction(0.05, motorActions.outTakeClaw.Close())
                .addWaitAction(0.1, motorActions.outtakeArm.Specimen()));

        // Score task 2.
        tasks.add(new PathChainTask(score2, 0.1)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Intake task 3.
        tasks.add(new PathChainTask(intake3, 0.2)
                .addWaitAction(0.05, motorActions.outTakeClaw.Close())
                .addWaitAction(0.1, motorActions.outtakeArm.Specimen()));
        // Score task 3.
        tasks.add(new PathChainTask(score3, 0.1)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Intake task 4.
        tasks.add(new PathChainTask(intake4, 0.2)
                .addWaitAction(0.05, motorActions.outTakeClaw.Close())
                .addWaitAction(0.1, motorActions.outtakeArm.Specimen()));

        // Score task 4.
        tasks.add(new PathChainTask(score4, 0.1)
                .addWaitAction(0.01, motorActions.depositSpecimen()));

        // Park task.
        tasks.add(new PathChainTask(pickupsample, 0.5));

        // New task: Move from park to scorePose
        PathChainTask Score = new PathChainTask(ScoreBasket, 0.4)
                .addWaitAction(() -> motorControl.lift.closeEnough(700),
                        new SequentialAction(
                                motorActions.outTakeLinkage.sample(),
                                new SleepAction(0.15),
                                motorActions.outtakeTransfer(),
                                motorActions.intakePivot.Grab(),
                                motorActions.intakeArm.Grab()
                        )
                )
                .setMaxWaitTime(2)
                .setWaitCondition(() -> motorControl.lift.closeEnough(700));

        tasks.add(Score);

        tasks.add(new PathChainTask(finalparking, 0.5));
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
