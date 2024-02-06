package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class Intake extends SubsystemBase {
  final double FACTOR = 100.0; // TODO set this correctly for intake speed - note vel [cm/s]

  /** Creates a new Intake. */
  public double intake_speed = 0.0;
  public double r_speed = 0.0;
  public double l_speed = 0.0;
  
  // Intake Angle, a servo 
  final NeoServo angle_servo;
  final PIDController positionPID = new PIDController(0.0, 0.0, 0.0); // outer (pos)

  // Intake roller motor
  final CANSparkMax intake_mtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
  final PIDFController hwAngleVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0); // inner (hw/vel)
  final PIDFController hwMotorVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0); // velocity mode
  final SparkPIDController intakeMtrPid;
  final RelativeEncoder intakeMtrEncoder;

  // lightgate tell us when we have a game piece (aka a Note)
  final DigitalInput lightgate = new DigitalInput(DigitalIO.IntakeLightGate);

  public Intake() {
    // servo controls angle of intake arm
    angle_servo = new NeoServo(CAN.ANGLE_MTR, positionPID, hwAngleVelPID, false); // should be invert false

    // use velocity control on intake motor
    intake_mtr.clearFaults();
    intake_mtr.restoreFactoryDefaults();
    intakeMtrPid = intake_mtr.getPIDController();
    intakeMtrEncoder = intake_mtr.getEncoder();
    intakeMtrEncoder.setPositionConversionFactor(FACTOR);
    intakeMtrEncoder.setVelocityConversionFactor(FACTOR / 60.0);
    // configure hardware pid with our values
    hwMotorVelPID.copyTo(intake_mtr.getPIDController(), 0);
    intake_mtr.burnFlash();

    /// Servo setup for angle_servo
    angle_servo.setConversionFactor(0.0) // TODO: add below later when we actually need values - NR 1/31/24
        // .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        // .setVelocityHW_PID(maxVel, maxAccel)
        // .setTolerance(posTol, velTol)
        // .setMaxVelocity(maxVel)
        .burnFlash();
  }

  public void setMotorSpeed(double speed) {
    intakeMtrPid.setReference(speed, ControlType.kVelocity, 0);
  }

  public boolean hasNote() {
    return lightgate.get();
  }

  public double getMotorSpeed() {
    return intakeMtrEncoder.getVelocity();
  }

  public void setAngleSetpoint(double position) {
    angle_servo.setSetpoint(position);
  }

  public double getAngleSetpoint() {
    return angle_servo.getSetpoint();
  }

  public double getAnglePosition() {
    return angle_servo.getPosition();
  }

  public double getAngleSpeed() {
    return angle_servo.getVelocity();
  }

  public void setAngleClamp(double min_ext, double max_ext) { // limit switch values?
    angle_servo.setClamp(min_ext, max_ext);
  }

  public boolean angleAtSetpoint() {
    return angle_servo.atSetpoint();  //are we there yet?
  }
}
