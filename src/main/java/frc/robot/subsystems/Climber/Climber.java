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
  final double GearRatio = 0; //have 2 eventually probably lol
  final double maxVel = 100.0; //placeholder
    final double maxAccel = 10.0; //placevholder
    double posTol = 2.0; //placeholder (maybe final posTol??)
    double velTol = 1.0;
    final int STALL_CURRENT = 5; //placeholder
    final int FREE_CURRENT = 15; //placeholder
  double desiredRightArmPos;
  double desiredLeftArmPos;
  boolean syncArmsEnabled = true; //should be true most of the time

  PIDController rightPID = new PIDController(0, 0, 0); //sw outer pos pid
  PIDController leftPID = new PIDController(0, 0, 0);
  PIDFController rightHwVelPID = new PIDFController(0, 0, 0, 0); //hw vel pid
  PIDFController leftHwVelPID = new PIDFController(0, 0, 0, 0);
  double follow_comp = 0.0; //if for some reason one arm faster than other
  final NeoServo rightArm = new NeoServo(Constants.CAN.LEFT_CLIMBER, leftPID, leftHwVelPID, false); // need to find invert
  final NeoServo leftArm = new NeoServo(Constants.CAN.RIGHT_CLIMBER, rightPID, rightHwVelPID, false);

  public Climber() {
    leftHwVelPID.copyTo(leftArm.getController().getPIDController(), 0);
    leftArm.setConversionFactor(360.0 / GearRatio) // we probs want diff conversions, currently in deg
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
    rightHwVelPID.copyTo(rightArm.getController().getPIDController(), 0);
    rightArm.setConversionFactor(360.0 / GearRatio) // we probs want diff conversions, currently in deg
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
  }

  public void setArmHeight(double pos){
        desiredLeftArmPos = pos;
    desiredRightArmPos = pos;
    leftArm.setSetpoint(pos);
    rightArm.setSetpoint(pos);
  }
  public void setLeftArmHeight(double pos){
    desiredLeftArmPos = pos;
    leftArm.setSetpoint(pos);
  }
  public void setRightArmHeight(double pos){
    desiredRightArmPos = pos;
    rightArm.setSetpoint(pos);
  }
// lines 46-56 are for testing only
  public void setArmVelocity(double vel){
    leftArm.setVelocityCmd(vel);
    rightArm.setVelocityCmd(vel);
  }

  public void setLeftArmVelocity(double vel){
  leftArm.setVelocityCmd(vel);
}
public void setRightArmVelocity(double vel){
  rightArm.setVelocityCmd(vel);
}

  public double getLeftArmHeight(){
    return leftArm.getPosition();
  }
  public double getRightArmHeight(){
    return rightArm.getPosition();
  }
  public double getLeftArmVelocity(){
    return leftArm.getVelocity();
  }
  public double getRightArmVelocity(){
    return rightArm.getVelocity();
  }

  public boolean leftArmAtSetpoint(){
    return leftArm.atSetpoint();
  }
  public boolean rightArmAtSetpoint(){
    return rightArm.atSetpoint();
  }
  public void ClampVel(double vel){
    MathUtil.clamp(vel, maxVel, -maxVel);
  }
  public void ClampAccel(double accel){
    MathUtil.clamp(accel, maxAccel, -maxAccel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
