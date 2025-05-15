package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;


public class MotorActions {
    public final MotorControl motorControl;
    public Enums.Intake intakePosition = Enums.Intake.Transfer;
    public Enums.OutTake outtakePosition = Enums.OutTake.Transfer;

    public final Extendo extendo;
    public final Lift lift;
    public final Spin spin;
    public final IntakeArm intakeArm;
    public final IntakePivot intakePivot;
    public final OuttakePivot outtakePivot;
    public final OutTakeClaw outTakeClaw;
    public final OuttakeLinkage outTakeLinkage;
    public final OuttakeArm outtakeArm;
    public final OuttakeTurret outtakeTurret;
    public final Hang hang;

    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.extendo = new Extendo();
        this.lift = new Lift();
        this.intakeArm = new IntakeArm();
        this.intakePivot = new IntakePivot();
        this.outtakePivot = new OuttakePivot();
        this.outTakeClaw = new OutTakeClaw();
        this.spin = new Spin(motorControl);
        this.outTakeLinkage = new OuttakeLinkage();
        this.outtakeArm = new OuttakeArm();
        this.outtakeTurret = new OuttakeTurret();
        this.hang = new Hang();
    }



    public Action outtakeTransfer(){
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Transfer;
                    return false;
                },
                outTakeClaw.Open(),
                new SleepAction(0.2),
                outtakeArm.newtransferpre(),
                outTakeLinkage.Transfer(),
                new SleepAction(0.2),
                outtakePivot.Transfer(),
                outtakeTurret.up(),
                lift.transfer(),
                lift.waitUntilFinished()
        );
    }
    public Action outtakeTransferforauto(){
        return new ParallelAction(
                t -> {
                    outtakePosition = Enums.OutTake.Transfer;
                    return false;
                },
                outTakeClaw.Open(),
                outtakeArm.newtransferpre(),
                outTakeLinkage.Transfer(),
                outtakePivot.Transfer(),
                outtakeTurret.up(),
                lift.transfer()
        );
    }



    public Action intakeExtend(double Position) {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Extended;
                    return false;
                },
                intakeArm.Extended(),
                intakePivot.Grab(),
                extendo.setTargetPosition(Position),
                extendo.waitUntilFinished());
    }

    public Action intakeGrabUntil(Enums.DetectedColor allianceColor) {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Spin;
                    return false;
                },
                lift.setTargetPosition(30),
                intakeArm.Grab(),
                intakePivot.Grab(),
                //outtakeTransfer(),
                spin.eatUntil(allianceColor, motorControl),
                intakeTransfer(),
                spin.slow()

        );
    }

    public Action edgegrabuntil(Enums.DetectedColor allianceColor) {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Spin;
                    return false;
                },
                lift.setTargetPosition(30),
                intakeArm.edge(),
                intakePivot.edge(),
                outtakeTransfer(),
                spin.eatUntil(allianceColor, motorControl),
                intakeTransfer(),
                spin.slow()

        );
    }

    public Action autointakeGrabUntil(Enums.DetectedColor allianceColor) {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Spin;
                    return false;
                },
                lift.setTargetPosition(30),
                intakeArm.Grab(),
                intakePivot.Grab(),
                outtakeTransfer(),
                spin.eatUntil(allianceColor, motorControl),
                newintakeTransfer(),
                spin.slow()

        );
    }

    public Action autointakeTransfer() {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Transfer;
                    return false;
                },

                outtakeTurret.up(),
                intakeArm.Retracted(),
                intakePivot.Extend(),
                extendo.retracted(),
                extendo.waitUntilFinished(),
                intakePivot.Transfer(),
                outTakeClaw.Open(),
                intakeArm.Transfer(),
                spin.slow()

        );
    }

    public Action newintakeTransfer() {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Transfer;
                    return false;
                },

                outtakeTurret.up(),
                intakeArm.newtransfer(),
                intakePivot.newtransfer(),
                extendo.setTargetPosition(5),
                extendo.waitUntilFinished(),
                outTakeClaw.Open(),
                spin.slow()

        );
    }

    public Action autointakeTransferauto() {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Transfer;
                    return false;
                },
                outtakeTransferforauto(),
                outtakeTurret.up(),
                intakeArm.Retracted(),
                intakePivot.Extend(),
                extendo.retracted(),
                outTakeClaw.Open(),
                extendo.waitUntilFinished(),
                intakeArm.Transfer(),
                spin.eat(),
                new SleepAction(0.1),
                intakePivot.Transferauto(),
                new SleepAction(0.25),
                outTakeClaw.Close(),
                new SleepAction(0.1),
                intakeArm.Extended(),
                spin.slow()

        );
    }






    public Action intakeTransfer() {
        return new SequentialAction(
                t -> {
                    intakePosition = Enums.Intake.Transfer;
                    return false;
                },
                outtakeTurret.up(),
                intakeArm.Extended(),
                intakePivot.Extend(),
                extendo.retracted(),
                extendo.waitUntilFinished(),
                outTakeClaw.Open(),
                spin.slow()
        );
    }

    public Action outtakeSample() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },

                intakeArm.Transfer(),
                new SleepAction(0.1),
                intakePivot.Transfer(),
                new SleepAction(0.2),
                outtakeArm.Transfer2(),
                outTakeClaw.Close(),
                outtakePivot.TRANSFER2(),
                intakePivot.Transfer(),
                new SleepAction(0.2),
                intakeArm.Extended(),
                outTakeLinkage.sample(),
                outtakeArm.sample(),
                outtakePivot.Deposit(),
                lift.setTargetPosition(790),
                new SleepAction(0.1),
                spin.slowpoop(),
                new SleepAction(0.2),
                spin.stop(),
                outtakeTurret.half(),
                intakePivot.Extend(),
                lift.waitUntilFinished()
        );
    }
    public Action outtakeSampleautomatic() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },
                intakeArm.Transfer(),
                intakePivot.Transfer(),
                new SleepAction(0.1),
                outtakeArm.Transfer2(),
                outTakeClaw.Close(),
                outtakePivot.TRANSFER2(),
                intakeArm.Transfer(),
                intakePivot.Transfer(),
                new SleepAction(0.1),
                intakeArm.Extended(),
                outTakeLinkage.sample(),
                outtakeArm.sample(),
                outtakePivot.DepositSample(),
                lift.setTargetPosition(790),
                new SleepAction(0.1),
                spin.slowpoop(),
                new SleepAction(0.2),
                spin.stop(),
                outtakeTurret.half(),
                intakePivot.Extend(),
                lift.waitUntilFinished()
        );
    }

    public Action outtakesamplenew() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },
                outtakeArm.newtransferpost(),
                outtakePivot.newtransfer2(),
                new SleepAction(0.1),
                outTakeClaw.Close(),
                new SleepAction(0.1),
                intakeArm.Extended(),
                outTakeLinkage.sample(),
                outtakeArm.predepo(),
                outtakePivot.DepositSample(),
                lift.setTargetPosition(790),
                new SleepAction(0.1),
                spin.slowpoop(),
                new SleepAction(0.2),
                spin.stop(),
                outtakeTurret.half(),
                intakePivot.Extend(),
                extendo.retracted(),
                lift.waitUntilFinished(),
                outtakeArm.sample()
        );
    }


    public Action outtakesampleslow() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },
                outtakeArm.newtransferpost(),
                outtakePivot.newtransfer2(),
                new SleepAction(0.5),
                outTakeClaw.Close(),
                new SleepAction(0.1),
                intakeArm.Extended(),
                outTakeLinkage.sample(),
                outtakeArm.predepo(),
                outtakePivot.DepositSample(),
                lift.setTargetPosition(790),
                new SleepAction(0.1),
                spin.slowpoop(),
                new SleepAction(0.2),
                spin.stop(),
                outtakeTurret.half(),
                intakePivot.Extend(),
                extendo.retracted(),
                lift.waitUntilFinished(),
                outtakeArm.sample()
        );
    }

    public Action outtakeSample(double Position) {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },

                intakeArm.Transfer(),
                intakePivot.Transfer(),
                outtakeArm.Transfer2(),
                intakePivot.Transfer(),
                outTakeClaw.Close(),
                outtakePivot.TRANSFER2(),
                new SleepAction(0.1),
                intakeArm.Extended(),
                outTakeLinkage.sample(),
                outtakeArm.sample(),
                outtakePivot.DepositSample(),
                lift.setTargetPosition(Position),
                new SleepAction(0.1),
                spin.slowpoop(),
                new SleepAction(0.2),
                outtakeTurret.down(),
                intakePivot.Extend(),
                spin.stop(),
                lift.waitUntilFinished()

        );
    }



    public Action outtakeSampleAuto() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },

                intakeArm.Transfer(),
                intakePivot.Transfer(),
                //new SleepAction(0.01),

                new SleepAction(0.2),
                outtakeArm.Transfer2(),
                outTakeClaw.Close(),
                outtakePivot.TRANSFER2(),
                new SleepAction(0.3),
                intakeArm.Extended(),
                outtakeArm.sample(),
                outtakePivot.Deposit(),
                lift.setTargetPosition(750),
                new SleepAction(0.1),
                spin.slowpoop(),
                outtakeTurret.down(),
                new SleepAction(0.2),
                spin.stop(),
                intakePivot.Extend(),
                lift.waitUntilFinished(730),
                outTakeLinkage.sample(),
                new SleepAction(0.35),
                outTakeClaw.Open()
        );
    }

    public Action outtakeSampleFullAuto() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Deposit;
                    return false;
                },
                autointakeTransferauto(),
                outtakeArm.sample(),
                outtakePivot.Deposit(),
                lift.setTargetPosition(750),
                new SleepAction(0.1),
                spin.slowpoop(),
                new SleepAction(0.2),
                spin.stop(),
                intakePivot.Extend(),
                outtakeTurret.down(),
                lift.waitUntilFinished(750),
                outTakeLinkage.sample(),
                new SleepAction(0.35),
                outTakeClaw.Open()
        );
    }




    public Action intakeSpecimen(){
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.wall;
                    return false;
                },
                lift.setTargetPosition(10),
                outtakeTurret.down(),
                new SleepAction(0.1),
                outTakeLinkage.wall(),
                outtakeArm.wall(),
                outtakePivot.wall(),
                outTakeClaw.Open()
        );
    }


    public Action outtakeSpecimen() {
        return new SequentialAction(
                t -> {
                    outtakePosition = Enums.OutTake.Specimen;
                    return false;
                },
                spin.stop(),
                outTakeClaw.Close(),
                intakeArm.Extended(),
                new SleepAction(0.15),
                intakeArm.Up(),
                intakePivot.Extend(),
                outTakeLinkage.Specimen(),
                outtakeArm.Specimen(),
                outtakePivot.Deposit(),
                lift.secondTruss(),
                new SleepAction(0.25),
                outtakeTurret.up()

        );
    }

    public Action depositSpecimen(){
        return new SequentialAction(
                intakeArm.Extended(),
                outTakeLinkage.Specimen(),
                outtakePivot.Deposit2(),
                outTakeClaw.Open(),
                new SleepAction(0.3),
                intakeSpecimen()
        );
    }


    public Action update() {
        return t -> {
            motorControl.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }

        public class Extendo {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.extendo.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.extendo.closeEnough();
                }
            };
        }

            public Action waitUntilFinished(double position) {
                return new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket t) {
                        return !motorControl.extendo.closeEnough(position);
                    }
                };
            }

        public Action findZero() {
            return new SequentialAction(t -> {motorControl.extendo.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !motorControl.extendo.isResetting()));
        }



        public Action retracted() {
            return setTargetPosition(40);
        }
        public Action extended() {
            return setTargetPosition(590);
        }
    }

    public class Lift {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.lift.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.lift.closeEnough();
                }
            };
        }

        public Action waitUntilFinished(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.lift.closeEnough();
                }
            };
        }

        public Action findZero() {
            return new SequentialAction(t -> {motorControl.lift.findZero();return false;},
                    new ActionHelpers.WaitUntilAction(() -> !motorControl.lift.isResetting()));
        }



        public Action transfer() {
            return setTargetPosition(30);
        }
        public Action secondBucket() {
            return setTargetPosition(670);
        }
        public Action firstTruss() {
            return setTargetPosition(100);
        }
        public Action secondTruss() {
            return setTargetPosition(310);
        }
    }


    public class IntakeArm {
        private static final double GRAB_POSITION = 0.05;
        private static final double INTAKE_POSITION = 0.2;
        private static final double EXTENDED_POSITION = 0.25;

        private static final double TRANSFER_POSITION = 0.45;

        private static final double newtransfer = 0.3;

        private static final double Retracted = 0.35;
        private static final double Up = 0.58;

        private static final double edge = 0.38;


        public Action setTargetPosition(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    motorControl.armLeft.setTargetPosition(position);
                    motorControl.armRight.setPosition(position);
                    new SleepAction(0.1);
                    return false;
                }
            };
        }

        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.armLeft.closeEnough(); // Returns true when both servos are close enough
                }
            };
        }

        public Action Grab() {
            return setTargetPosition(GRAB_POSITION);
        }

        public Action Extended() {
            return setTargetPosition(EXTENDED_POSITION);
        }

        public Action Intake() {
            return setTargetPosition(INTAKE_POSITION);
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Up() {
            return setTargetPosition(Up);
        }

        public Action edge() {
            return setTargetPosition(edge);
        }

        public Action Retracted() {
            return setTargetPosition(Retracted);
        }
        public Action newtransfer() {
            return setTargetPosition(newtransfer);
        }
    }

    public class IntakePivot {

        private static final double GRAB_POSITION = 0.74;
        private static final double EXTENDED_POSITION = 0.74;
        private static final double TRANSFER_POSITION = 0.58;
        private static final double DOWN = 0.3;
        private static final double edge = 0.18;
        private static final double newtransfer = 0.7;
        private static final double tauto = 0.55;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.intakePivot.setPosition(position);
                return false;
            };
        }

        public Action Grab() {
            return setTargetPosition(GRAB_POSITION);
        }

        public Action Extend() {
            return setTargetPosition(EXTENDED_POSITION);
        }

        public Action Transfer(){
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Down(){
            return setTargetPosition(DOWN);
        }

        public Action edge(){
            return setTargetPosition(edge);
        }

        public Action Transferauto(){
            return setTargetPosition(tauto);
        }
        public Action newtransfer(){
            return setTargetPosition(newtransfer);
        }

    }




    public class OuttakePivot {
        private static final double TRANSFER_POSITION = 0.2;

        private static final double newtransfer1 = 0.3;

        private static final double newtransfer2 = 0.23;
        private static final double DEPOSIT = 0.6;
        private static final double DEPOSIT2 = 0.3;
        private static final double DEPOSITSample = 0.6;

        private static final double depositbetter = 0.65;

        private static final double WALL_INTAKE = 0.48;
        private static final double TRANSFER = 0.16;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakePivot.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Deposit() {
            return setTargetPosition(DEPOSIT);
        }
        public Action Deposit2() {
            return setTargetPosition(DEPOSIT2);
        }

        public Action wall(){
            return setTargetPosition(WALL_INTAKE);
        }
        public Action TRANSFER2(){
            return setTargetPosition(TRANSFER);
        }

        public Action DepositSample(){
            return setTargetPosition(DEPOSITSample);
        }

        public Action depositbetter(){
            return setTargetPosition(depositbetter);
        }
        public Action newtransfer1(){
            return setTargetPosition(newtransfer1);
        }
        public Action newtransfer2(){
            return setTargetPosition(newtransfer2);
        }

    }
    public class Hang {
        public Action up() {
            return t -> {
                motorControl.hangr.setPower(-1);
                motorControl.hangl.setPower(-1);
                return false;
            };
        }

        public Action down() {
            return t -> {
                motorControl.hangr.setPower(1);
                motorControl.hangl.setPower(1);
                return false;
            };
        }

        public Action autohang() {
            return t -> {
            motorControl.hangr.setPower(1);
            motorControl.hangl.setPower(1);
            new SleepAction(3.0);
            motorControl.hangr.setPower(0);
            motorControl.hangl.setPower(0);
            return false;
            };
        }

        public Action stop() {
            return t -> {
                motorControl.hangr.setPower(0);
                motorControl.hangl.setPower(0);
                return false;
            };
        }
    }

    public class OuttakeLinkage {
        private static final double TRANSFER_POSITION = 0.73;
        private static final double SPECIMEN_DEPOSIT = 0.73;
        private static final double WALL_INTAKE = 0.5;
        private static final double SAMPLE_DEPOSIT = 0.14;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakeLinkage.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }

        public Action Specimen() {
            return setTargetPosition(SPECIMEN_DEPOSIT);
        }

        public Action wall(){
            return setTargetPosition(WALL_INTAKE);
        }
        public Action sample(){
            return setTargetPosition(SAMPLE_DEPOSIT);
        }

    }


    public class OuttakeArm {
        private static final double TRANSFER_POSITION = 0.81;
        private static final double TRANSFER_POSITION2 = 0.85;
        private static final double newtransferpre = 0.75;
        private static final double newtransferpost = 0.92;

        private static final double SPECIMEN_DEPOSIT = 0.95;
        private static final double WALL_INTAKE = 0;
        private static final double SAMPLE_DEPOSIT = 0.4;

        private static final double fiveone = 0.65;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakeRotation.setPosition(position);
                return false;
            };
        }

        public Action Transfer() {
            return setTargetPosition(TRANSFER_POSITION);
        }


        public Action Transfer2() {
            return setTargetPosition(TRANSFER_POSITION2);
        }

        public Action Specimen() {
            return setTargetPosition(SPECIMEN_DEPOSIT);
        }

        public Action wall(){
            return setTargetPosition(WALL_INTAKE);
        }
        public Action sample(){
            return setTargetPosition(SAMPLE_DEPOSIT);
        }

        public Action predepo(){
            return setTargetPosition(fiveone);
        }
        public Action newtransferpre(){
            return setTargetPosition(newtransferpre);
        }
        public Action newtransferpost(){
            return setTargetPosition(newtransferpost);
        }
    }

    public class OuttakeTurret {
        private static final double UP = 0.9;

        private static final double half = 0.5;
        private static final double DOWN = 0.1;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.turret.setPosition(position);
                return false;
            };
        }

        public Action up() {
            return setTargetPosition(UP);
        }
        public Action half() {
            return setTargetPosition(half);
        }

        public Action down() {
            return setTargetPosition(DOWN);
        }
    }




    public class OutTakeClaw {
        private static final double CLOSE_POSITION = 0.02;
        private static final double OPEN_POSITION = 0.36;
        private static final double PARTIAL_CLOSE = 0.1;
        private static final double PARTIAL_OPEN = 0.25;

        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.outtakeClaw.setPosition(position);
                return false;
            };
        }

        public Action Close() {
            return setTargetPosition(CLOSE_POSITION);
        }

        public Action Open() {
            return setTargetPosition(OPEN_POSITION);
        }

        public Action PartialClose() {
            return setTargetPosition(PARTIAL_CLOSE);
        }

        public Action PartialOpen() {
            return setTargetPosition(PARTIAL_OPEN);
        }
    }

    public static class TurnAction implements Action {
        private final Follower follower;
        private final double angle;
        private boolean started = false;

        /**
         * Creates a new TurnAction.
         * @param follower The follower instance to command.
         * @param angle The angle in radians to turn.
         */
        public TurnAction(Follower follower, double angle) {
            this.follower = follower;
            this.angle = angle;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            if (!started) {
                // Issue the turn command only once.
                follower.turnTo(angle);
                started = true;
            }
            // Optionally, add telemetry:
            // p.put("TurnAction", "Turning to angle: " + angle);
            // Return true while still turning; false once turn is complete.
            return follower.isTurning();
        }

        @Override
        public void preview(Canvas fieldOverlay) {
            // Optionally, draw a preview of the turn on the fieldOverlay.
        }
    }

    public class Spin {
        private final MotorControl motorControl;


        public Spin(MotorControl motorControl) {
            this.motorControl = motorControl;
        }


        public Action eatUntil(DetectedColor allianceColor, MotorControl motorControl) {
            final Enums.IntakeState[] currentState = {Enums.IntakeState.SEARCHING};
            final boolean[] started = {false};
            // Add a variable to store when the rejecting state was entered
            final long[] rejectingStartTime = {0};
            // Define the delay in milliseconds (adjust as needed)
            final long REJECT_DELAY_MS = 400;

            return telemetryPacket -> {
                if (!started[0]) {
                    started[0] = true;
                    motorControl.spin.setPower(1); // Start spinning forward
                }

                // Read the sensor
                DetectedColor color = motorControl.getDetectedColor();

                // If yellow is detected, task is finished
                if (color == Enums.DetectedColor.YELLOW) {
                    motorControl.spin.setPower(0); // Stop motor
                    return false; // Action complete
                }

                boolean correctColorSeen = false;
                if (allianceColor == Enums.DetectedColor.RED) {
                    // Accept RED or YELLOW
                    correctColorSeen = (color == Enums.DetectedColor.RED || color == Enums.DetectedColor.YELLOW);
                } else if (allianceColor == Enums.DetectedColor.BLUE) {
                    // Accept BLUE or YELLOW
                    correctColorSeen = (color == Enums.DetectedColor.BLUE || color == Enums.DetectedColor.YELLOW);
                }

                switch (currentState[0]) {
                    case SEARCHING:
                        if (correctColorSeen) {
                            motorControl.spin.setPower(0); // Stop motor
                            return false; // Action complete
                        }
                        // If an unexpected (rejectable) color is seen, transition to REJECTING
                        if (color != Enums.DetectedColor.BLACK
                                && color != Enums.DetectedColor.UNKNOWN
                                && !correctColorSeen) {
                            currentState[0] = Enums.IntakeState.REJECTING;
                            // Record the time when rejecting starts
                            rejectingStartTime[0] = System.currentTimeMillis();
                            motorControl.spin.setPower(-1); // Spin backward
                        }
                        return true; // Continue searching

                    case REJECTING:
                        long now = System.currentTimeMillis();
                        // Only check for resuming forward spinning after the delay
                        if (now - rejectingStartTime[0] >= REJECT_DELAY_MS) {
                            if (color == Enums.DetectedColor.BLACK || color == Enums.DetectedColor.UNKNOWN) {
                                // Done rejecting; resume forward spinning
                                motorControl.spin.setPower(1);
                                currentState[0] = Enums.IntakeState.SEARCHING;
                                // Reset the timer (optional cleanup)
                                rejectingStartTime[0] = 0;
                            }
                        }
                        return true; // Continue in the REJECTING state until conditions are met
                }

                // Default (should not be reached)
                return false;
            };

    }


        /**
         * Simple "spin forward" action that runs indefinitely unless you remove it.
         * You can adapt the return value to end immediately if desired.
         */
        public Action slow() {
            return telemetryPacket -> {
                motorControl.spin.setPower(0.5);
                return false;
            };
        }


        public Action slowpoop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(-0.45);
                return false;
            };
        }

        public Action eat() {
            return telemetryPacket -> {
                motorControl.spin.setPower(1);
                return false;
            };
        }

        /**
         * Simple "spin backward" action that runs indefinitely.
         */
        public Action poop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(-1.0);
                return false;
            };
        }

        /**
         * Stop the spin motor (one-shot).
         */
        public Action stop() {
            return telemetryPacket -> {
                motorControl.spin.setPower(0.0);
                return false; // done immediately
            };
        }





    }






}
