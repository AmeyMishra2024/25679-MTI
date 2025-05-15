package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoTesting extends OpMode {

    // Declare servo variables
    Servo outtakePivot, turret, armLeft, armRight, intakePivot, outtakeRotation, outtakeLinkage, outtakeClaw;
    AnalogInput intakeSensor;

    // New: Declare motor
    DcMotor spin;

    // Target positions for each servo
    public static double intakePivotTarget = 1;
    public static double outtakePivotTarget = 0.68;
    public static double turretTarget = 0.1;
    public static double armLeftTarget = 0.2;
    public static double armRightTarget = 0.2;
    public static double outtakeRotationTarget = 0.7;
    public static double outtakeLinkageTarget = 0.6;
    public static double outtakeClawTarget = 0.05;

    // New: Power value for spin motor
    public static double spinPower = 0.0;

    // Analog input range for mapping
    public static double MIN_ANALOG = 0.115;
    public static double MAX_ANALOG = 0.905;

    @Override
    public void init() {
        // Initialize servos
        outtakePivot = hardwareMap.get(Servo.class, "outtakepivot");
        turret = hardwareMap.get(Servo.class, "outtaketurret");
        outtakeRotation = hardwareMap.get(Servo.class, "outtakerotation");
        armLeft = hardwareMap.get(Servo.class, "armleft");
        armRight = hardwareMap.get(Servo.class, "armright");
        intakePivot = hardwareMap.get(Servo.class, "intakepivot");
        outtakeLinkage = hardwareMap.get(Servo.class, "outtakelinkage");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeclaw");

        // Initialize analog input
        intakeSensor = hardwareMap.get(AnalogInput.class, "intakesensor");

        // New: Initialize spin motor
        spin = hardwareMap.get(DcMotor.class, "spin");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Map voltage to position
        double normalizedVoltage = mapVoltageToPosition(intakeSensor.getVoltage() / 3.3);

        // Set servo positions
        intakePivot.setPosition(intakePivotTarget);
        outtakePivot.setPosition(outtakePivotTarget);
        turret.setPosition(turretTarget);
        armLeft.setPosition(armLeftTarget);
        armRight.setPosition(armRightTarget);
        outtakeRotation.setPosition(outtakeRotationTarget);
        outtakeLinkage.setPosition(outtakeLinkageTarget);
        outtakeClaw.setPosition(outtakeClawTarget);

        // New: Set motor power
        spin.setPower(spinPower);

        // Telemetry
        telemetry.addData("Intake Sensor Position", intakeSensor.getVoltage() / 3.3);
        telemetry.addData("Normalized Position", normalizedVoltage);
        telemetry.addData("Intake Pivot Target", intakePivotTarget);
        telemetry.addData("Outtake Pivot Target", outtakePivotTarget);
        telemetry.addData("Turret Target", turretTarget);
        telemetry.addData("Arm Left Target", armLeftTarget);
        telemetry.addData("Arm Right Target", armRightTarget);
        telemetry.addData("Outtake Rotation Target", outtakeRotationTarget);
        telemetry.addData("Outtake Linkage Target", outtakeLinkageTarget);
        telemetry.addData("Outtake Claw Target", outtakeClawTarget);
        telemetry.addData("Spin Power", spinPower); // New telemetry
        telemetry.update();
    }

    private double mapVoltageToPosition(double voltage) {
        return Math.abs((voltage - MIN_ANALOG) / (MAX_ANALOG - MIN_ANALOG));
    }
}
