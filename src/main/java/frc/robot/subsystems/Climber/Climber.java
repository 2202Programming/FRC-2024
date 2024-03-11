// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NeoServo;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.PIDFController;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  final double GearRatio = 1.0; // have 2 eventually probably lol
  final double maxVel = 100.0; // placeholder. cm/s?
  final double maxAccel = 10.0; // placevholder cm/s^2
  double posTol = 2.0; // placeholder (maybe final posTol??) cm
  double velTol = 1.0; // cm/s
  final int STALL_CURRENT = 5; // placeholder // units?
  final int FREE_CURRENT = 15; // placeholder // units?
  double desiredPos; // cm, 0 is full retract

  PIDController posPID = new PIDController(0, 0, 0);
  PIDFController hwVelPID = new PIDFController(0, 0, 0, 0);
  final NeoServo climber = new NeoServo(Constants.CAN.RIGHT_CLIMBER, posPID, hwVelPID, false); //check invert

  public Climber() {
    hwVelPID.copyTo(climber.getController().getPIDController(), 0);
    climber.setConversionFactor(1.0 / GearRatio) // find this
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
  }

  /**
   * Extend arm to position given.
   *
   * @param pos Desired position in cm from fully retracted position
   */
  public void setArmHeight(double pos) {
    desiredPos = pos;
    climber.setSetpoint(pos);
  }

  // lines 46-56 are for testing only
  /**
   * Testing command, sets the arms to a comanded velocity
   *
   * @param vel Sets arms to a specific velocity (in cm/sec)
   */
  public void setArmVelocity(double vel) {
    climber.setVelocityCmd(vel);
  }

  public double getClimberHeight() {
    return climber.getPosition();
  }

  public double getClimberVelocity() {
    return climber.getVelocity();
  }

  public boolean atSetpoint() {
    return climber.atSetpoint();
  }

  public void ClampVel(double vel) {
    MathUtil.clamp(vel, maxVel, -maxVel);
  }

  public void ClampAccel(double accel) {
    MathUtil.clamp(accel, maxAccel, -maxAccel);
  }

  // TODO: Calibration helpers
  // void SetZero()
  // double getCurrent()?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climber.periodic();

  }
}
