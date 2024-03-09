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
  double desiredRightArmPos; // cm, 0 is full retract
  double desiredLeftArmPos; // cm, 0 is full retract
  boolean sync = false; // if we want to use sync or not
  double syncComp = 0.0; // what is

  PIDController rightPID = new PIDController(0, 0, 0); // sw outer pos pid
  PIDController leftPID = new PIDController(0, 0, 0);
  PIDFController rightHwVelPID = new PIDFController(0, 0, 0, 0); // hw vel pid
  PIDFController leftHwVelPID = new PIDFController(0, 0, 0, 0);
  PIDController syncPID = new PIDController(0.1, 0.0, 0.0); // sync pid left -> right
  double followComp = 0.0; // if for some reason one arm faster than other
  final NeoServo rightArm = new NeoServo(Constants.CAN.LEFT_CLIMBER, leftPID, leftHwVelPID, false); // need to find
                                                                                                    // invert
  final NeoServo leftArm = new NeoServo(Constants.CAN.RIGHT_CLIMBER, rightPID, rightHwVelPID, false);

  public Climber() {
    leftHwVelPID.copyTo(leftArm.getController().getPIDController(), 0);
    leftArm.setConversionFactor(1.0 / GearRatio) // find this
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
    rightHwVelPID.copyTo(rightArm.getController().getPIDController(), 0);
    rightArm.setConversionFactor(1.0 / GearRatio) // find this
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
    desiredLeftArmPos = pos;
    desiredRightArmPos = pos;
    leftArm.setSetpoint(pos);
    rightArm.setSetpoint(pos);
    sync = true;
  }

  public void setLeftArmHeight(double pos) {
    desiredLeftArmPos = pos;
    leftArm.setSetpoint(pos);
    sync = false;
  }

  public void setRightArmHeight(double pos) {
    desiredRightArmPos = pos;
    rightArm.setSetpoint(pos);
    sync = false;
  }

  // lines 46-56 are for testing only
  public void setArmVelocity(double vel) {
    leftArm.setVelocityCmd(vel);
    rightArm.setVelocityCmd(vel);
    sync = false;
  }

  /**
   * Run left arm at given velocity
   *
   * @param vel Velocity to run arm at in cm/s. Pos extends arm
   */
  public void setLeftArmVelocity(double vel) {
    leftArm.setVelocityCmd(vel);
    sync = false;
  }

  /**
   * Run right arm at given velocity
   *
   * @param vel Velocity to run arm at in cm/s. Pos extends arm
   */
  public void setRightArmVelocity(double vel) {
    rightArm.setVelocityCmd(vel);
    sync = false;
  }

  public double getLeftArmHeight() {
    return leftArm.getPosition();
  }

  public double getRightArmHeight() {
    return rightArm.getPosition();
  }

  public double getLeftArmVelocity() {
    return leftArm.getVelocity();
  }

  public double getRightArmVelocity() {
    return rightArm.getVelocity();
  }

  public boolean leftArmAtSetpoint() {
    return leftArm.atSetpoint();
  }

  public boolean rightArmAtSetpoint() {
    return rightArm.atSetpoint();
  }

  public void ClampVel(double vel) {
    MathUtil.clamp(vel, maxVel, -maxVel);
  }

  public void ClampAccel(double accel) {
    MathUtil.clamp(accel, maxAccel, -maxAccel);
  }

  public void setArmSync(boolean sync) {
    this.sync = sync;
  }

  // TODO: Calibration helpers
  // void SetZero()
  // double getCurrent()?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    syncComp = sync ? syncPID.calculate(leftArm.getPosition(), rightArm.getPosition()) : 0.0;
    leftArm.periodic(syncComp);
    rightArm.periodic(0.0);

  }
}
