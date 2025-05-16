package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.data.Enums;
import org.firstinspires.ftc.teamcode.helpers.data.Enums.DetectedColor;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Advanced Teleop V2 BLUE", group = "Examples")
public class TeleopBlue extends ActionOpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private MotorControl motorControl;
    private MotorActions motorActions;

    private DetectedColor allianceColor = DetectedColor.BLUE;
    private boolean autoOuttake = true;
    private boolean secondBucket = false;

    private boolean gamepad1XPressed = false;
    private boolean gamepad2XPressed = false;
    private boolean gamepad2YPressed = false;
    private boolean gamepad2APressed = false;
    private boolean poopPressed = false;
    private boolean dpadDownPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean hangAutoClimbStarted = false;
    private boolean dpadLeftPressed = false;

    private boolean dpadrightPressed = false;


    private boolean hangAutoDropStarted = false;




    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        run(new SequentialAction(motorActions.intakeTransfer(),
                motorActions.outtakeTransfer()));
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        if (gamepad1.start && !gamepad2YPressed) {
            autoOuttake = !autoOuttake;
            gamepad2YPressed = true;
        } else if (!gamepad1.start) {
            gamepad2YPressed = false;
        }

        if (gamepad1.x && !gamepad1XPressed) {
            if (allianceColor == DetectedColor.RED) {
                allianceColor = DetectedColor.BLUE;
            } else {
                allianceColor = DetectedColor.RED;
            }
            gamepad1XPressed = true;
        } else if (!gamepad1.x) {
            gamepad2XPressed = false;
        }
        double liftTarget = motorControl.lift.getTargetPosition();

// RIGHT BUMPER
        if (gamepad1.right_bumper && !rightBumperPressed) {
            if (liftTarget > 50) {
                run(motorActions.outtakeTurret.down());
                run(motorActions.outtakePivot.depositbetter());
            } else {
                if (motorActions.intakePosition == Enums.Intake.Transfer) {
                    run(motorActions.intakeExtend(300));
                } else if (motorActions.intakePosition == Enums.Intake.Spin) {
                    run(motorActions.intakeExtend(0));
                }
            }
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }

// LEFT BUMPER
        if (gamepad1.left_bumper && !leftBumperPressed) {
            if (liftTarget > 50) {
                run(motorActions.outtakeTurret.half());
            } else {
                run(motorActions.spin.eat());

                if (autoOuttake) {
                    if (secondBucket) {
                        double extendoPos = motorControl.extendo.motor.getCurrentPosition();
                        double sleepTime = (extendoPos < 50) ? 0.4 : 0.2;
                        run(new SequentialAction(
                                motorActions.autointakeGrabUntil(allianceColor),
                                new SleepAction(sleepTime),
                                motorActions.outtakeSample(330)
                        ));
                    } else {
                        double extendoPos = motorControl.extendo.motor.getCurrentPosition();
                        double sleepTime = (extendoPos < 60) ? 0.3 : 0.2;
                        run(new SequentialAction(
                                motorActions.autointakeGrabUntil(allianceColor),
                                new SleepAction(sleepTime),
                                motorActions.outtakesamplenew()
                        ));
                    }
                } else {
                    run(motorActions.intakeGrabUntil(allianceColor));
                }
            }
            leftBumperPressed = true;
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }

        DetectedColor color = motorControl.getDetectedColor();

        if (color == allianceColor || color == DetectedColor.YELLOW || motorControl.lift.getTargetPosition() > 70) {
            // Correct color detected → Triggers do outtake actions
            if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                if (motorActions.intakePosition == Enums.Intake.Transfer) {
                    if (secondBucket) {
                        run(motorActions.outtakeSample(330));
                    } else {
                        run(motorActions.outtakesamplenew());
                    }
                }
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                rightTriggerPressed = false;
            }

            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                if (motorActions.outtakePosition == Enums.OutTake.Deposit) {
                    run(motorActions.outtakeTransfer());
                }
                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                leftTriggerPressed = false;
            }
        } else {
            // Wrong color detected → Triggers control extendo movement
            if (gamepad1.right_trigger > 0 && motorControl.extendo.motor.getCurrentPosition() < 550) {
                int newTarget = Math.min(motorControl.extendo.motor.getCurrentPosition() + 100, 550);
                run(new ParallelAction(
                        motorActions.extendo.setTargetPosition(newTarget),
                        motorActions.extendo.waitUntilFinished()
                ));
            }

            if (gamepad1.left_trigger > 0 && motorControl.extendo.motor.getCurrentPosition() > 40) {
                int newTarget = Math.max(motorControl.extendo.motor.getCurrentPosition() - 100, 40);
                run(new ParallelAction(
                        motorActions.extendo.setTargetPosition(newTarget),
                        motorActions.extendo.waitUntilFinished()
                ));
            }
        }


        if (gamepad1.dpad_up) {
            if (motorActions.intakePosition == Enums.Intake.Transfer) {
                if (secondBucket) {
                    run(motorActions.outtakeSample(330));
                } else {
                    run(motorActions.outtakesamplenew());
                }
            }
        }


        if (gamepad1.dpad_down && !dpadDownPressed) {
            poopPressed = !poopPressed;
            DetectedColor down = motorControl.getDetectedColor();

            double extendPosition;
            if (down == DetectedColor.UNKNOWN || down == DetectedColor.BLACK) {
                extendPosition = 200; // If nothing in intake, extend to 500
            } else if (down == allianceColor || down == DetectedColor.YELLOW) {
                extendPosition = 200; // Correct color → Extend to 500
            } else {
                extendPosition = 200; // Wrong color → Extend to 200
            }

            if (!poopPressed) {
                run(new SequentialAction(
                        motorActions.intakeExtend(extendPosition),
                        motorActions.extendo.waitUntilFinished(extendPosition),
                        (down == allianceColor || down == DetectedColor.YELLOW) ? motorActions.spin.poop() : motorActions.spin.slowpoop()
                ));
            } else {
                run(new ParallelAction(
                        motorActions.spin.stop(),
                        motorActions.intakeTransfer()
                ));
            }

            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        if (gamepad1.a && !gamepad1.start) {
            run(new SequentialAction(motorActions.intakeSpecimen(), new SleepAction(0.1), motorActions.outtakeTurret.down()));
        } else if (gamepad1.b) {
            run(motorActions.outtakeSpecimen());
        } else if (gamepad1.y) {
            run(motorActions.depositSpecimen());
        }

        super.loop();
        motorControl.update();

        double rotationFactor = 1.0;

        if (motorActions.intakePosition != Enums.Intake.Extended){
            rotationFactor = 0.5;
        }

        if (gamepad1.dpad_left && !dpadLeftPressed) {
            run(new SequentialAction(
                    motorActions.hang.up(),
                    new SleepAction(3.0),
                    motorActions.hang.stop()
            ));
            dpadLeftPressed = true;
        } else if (!gamepad1.dpad_left) {
            dpadLeftPressed = false;
        }
        if (gamepad2.right_bumper) {
            run(motorActions.hang.down());
        } else if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
            run(motorActions.hang.stop());
        }

        if (gamepad1.dpad_right && !dpadrightPressed) {
            run(motorActions.edgegrabuntil(allianceColor));
            dpadLeftPressed = true;
        } else if (!gamepad1.dpad_left) {
            dpadLeftPressed = false;
        }



        if (getRuntime() >= 10 && !hangAutoDropStarted) {
            run(new SequentialAction(
                    t -> {
                        motorControl.hangr.setPower(1);
                        motorControl.hangl.setPower(1);
                        new SleepAction(3.0);
                        return false;
                    },
                    new SleepAction(3.0),
                    motorActions.hang.stop()
            ));
            hangAutoDropStarted = true;
        }

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * rotationFactor, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Alliance Color:", allianceColor);
        telemetry.addData("Auto Transfer:", autoOuttake);
        telemetry.addData("Second Bucket:", secondBucket);
        telemetry.addData("Intake Color:", motorControl.getDetectedColor());
        telemetry.addData("Intake Position State:", motorActions.intakePosition);
        telemetry.addData("Extendo Position:", motorControl.extendo.motor.getCurrentPosition());
        telemetry.addData("Lift Position:", motorControl.lift.motor.getCurrentPosition());
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
