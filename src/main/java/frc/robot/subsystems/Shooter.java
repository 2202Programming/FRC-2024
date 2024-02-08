// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.util.PIDFController;

public class Shooter extends SubsystemBase {

  static final double FACTOR = 1.0; // TODO put real value here

  public enum ShooterMode {
    Trigger, RPM
  };

  // Instantiations
  final CANSparkMax leftMtr = new CANSparkMax(CAN.SHOOTER_L, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax rightMtr = new CANSparkMax(CAN.SHOOTER_R, CANSparkMax.MotorType.kBrushless);

  final SparkPIDController hw_leftPid;
  final SparkPIDController hw_rightPid;

  // says unused for some reason? used on lines 61 & 68
  final RelativeEncoder leftEncoder;
  final RelativeEncoder rightEncoder;

  private ShooterMode currentShootingMode;

  private double currentLeftMotorOutput;
  private double currentRightMotorOutput;

  private double currentLeftMotorRPM;
  // private double currentRightMotorRPM; <-- not used currently

  PIDFController pidConsts = new PIDFController(0.001, 0.0, 0.0, 0.0);

  public Shooter() {
    hw_leftPid = motor_config(leftMtr, pidConsts, true);
    hw_rightPid = motor_config(rightMtr, pidConsts, false);

    leftEncoder = config_enc(leftMtr);
    rightEncoder = config_enc(rightMtr);
    currentShootingMode = ShooterMode.RPM;
  }

  @Override
  public void periodic() {
    // read encoders each frame, save to var if needed
    currentLeftMotorOutput = leftEncoder.getVelocity();
    currentRightMotorOutput = rightEncoder.getVelocity();
    return;
  }

  public void setShootingMode(ShooterMode mode) {
    currentShootingMode = mode;
  }

  public void cycleShootingMode() {
    if (currentShootingMode == ShooterMode.Trigger) {
      currentShootingMode = ShooterMode.RPM;
      return;
    }
    if (currentShootingMode == ShooterMode.RPM)
      currentShootingMode = ShooterMode.Trigger;
    return;
  }

  public ShooterMode getShooterMode() {
    return currentShootingMode;
  }

  @Deprecated
  public void setSpeed(double foo) {}

  public void setRPM(double leftRPM, double rightRPM) {
    hw_leftPid.setReference(leftRPM, ControlType.kVelocity);
    hw_rightPid.setReference(rightRPM, ControlType.kVelocity);
    // more test code
    // System.out.println("Motor goals changed,
    // left="+leftRPM*gearboxRatio+", right="+rightRPM*gearboxRatio);
    // //just debug code, not needed
  }

  public double getLeftMotorOutput() {
    return currentLeftMotorOutput;
  }

  public double getRightMotorOutput() {
    return currentRightMotorOutput;
  }


  public double getMotorRPM() {
    return currentLeftMotorRPM;
    // here for MotorTriggerOrDash, don't know why we aren't just using
    // getLeftMotorOutput
  }

  // PID getters/setters
  public double getRPM() {
    return currentLeftMotorRPM; // should be 1.0
  }

  public void setP(double newP) {
    hw_leftPid.setP(newP);
    hw_rightPid.setP(newP);
  }

  public void setI(double newI) {
    hw_leftPid.setI(newI);
    hw_rightPid.setI(newI);
  }

  public void setD(double newD) {
    hw_leftPid.setD(newD);
    hw_rightPid.setD(newD);
  }

  public double getP() {
    return hw_leftPid.getP();
  }

  public double getI() {
    return hw_leftPid.getI();
  }

  public double getD() {
    return hw_leftPid.getD();
  }

  SparkPIDController motor_config(CANSparkMax mtr, PIDFController pid, boolean inverted) {
    mtr.clearFaults();
    mtr.restoreFactoryDefaults();
    var mtrpid = mtr.getPIDController();
    pid.copyChangesTo(mtrpid, 0, pid);
    mtr.setInverted(inverted);
    return mtrpid;
  }

  RelativeEncoder config_enc(CANSparkMax mtr) {
    var enc = rightMtr.getEncoder();
    enc.setPositionConversionFactor(FACTOR);
    enc.setVelocityConversionFactor(FACTOR / 60.0);
    return enc;
  }
}
