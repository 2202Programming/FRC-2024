// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM1;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.PIDFController;

public class Shooter extends SubsystemBase {

  final CANSparkMax leftMtr = new CANSparkMax(CAN.SHOOTER_L, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax rightMtr = new CANSparkMax(CAN.SHOOTER_R, CANSparkMax.MotorType.kBrushless);
  final SparkPIDController hw_leftPid;
  final SparkPIDController hw_rightPid;
  final RelativeEncoder leftEncoder;
  final RelativeEncoder rightEncoder;
  final double FACTOR = 1.0;
  final double kF = 1.0 / 5650.0;

  private DoubleSolenoid shooterAngle; // can be replaced w/ servo in derived class

  private double desiredLeftRPM;
  private double desiredRightRPM;
  private double currentLeftRPM;
  private double currentRightRPM;

  PIDFController pidConsts = new PIDFController(0.00005, 0.00000009, 0.0, kF);  //slot 0  - normal
  PIDFController pidConsts_freeSpin = new PIDFController(0.0, 0.0, 0.0, 0.0);  //slot 1 - free spin

  public Shooter() {
    this(true);
  }

  public Shooter(boolean HasSolenoid) {
    hw_leftPid = motor_config(leftMtr, true);
    hw_rightPid = motor_config(rightMtr, false);
    leftEncoder = config_encoder(leftMtr);
    rightEncoder = config_encoder(rightMtr);
    if (HasSolenoid) {
      shooterAngle = new DoubleSolenoid(CAN.PCM1, PneumaticsModuleType.REVPH, PCM1.Forward, PCM1.Reverse);
      retract();
    }
  }

  @Override
  public void periodic() {
    currentLeftRPM = leftEncoder.getVelocity();
    currentRightRPM = rightEncoder.getVelocity();
  }

  public boolean isAtRPM(int tolerance) {
    return Math.abs(desiredLeftRPM - currentLeftRPM) < tolerance
        && Math.abs(desiredRightRPM - currentRightRPM) < tolerance;
  }


  public void setRPM(double leftRPM, double rightRPM) {
    // slot 0 --> normal op, slot 1 --> free spin
    int slot = (leftRPM == 0.0 && rightRPM == 0.0) ? 1 : 0;
    hw_leftPid.setReference(leftRPM, ControlType.kVelocity, slot);
    hw_rightPid.setReference(rightRPM, ControlType.kVelocity, slot);
    desiredLeftRPM = leftRPM;
    desiredRightRPM = rightRPM;
  }

  public void deploy() {
    shooterAngle.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    shooterAngle.set(DoubleSolenoid.Value.kReverse);
  }

  public WatcherCmd getWatcher() {
    return new ShooterWatcherCmd();
  }

  SparkPIDController motor_config(CANSparkMax mtr, boolean inverted) {
    mtr.clearFaults();
    mtr.restoreFactoryDefaults();
    var mtrpid = mtr.getPIDController();
    pidConsts.copyTo(mtrpid, 0);
    pidConsts_freeSpin.copyTo(mtrpid, 1); 
    mtr.setInverted(inverted);
    return mtrpid;
  }

  RelativeEncoder config_encoder(CANSparkMax mtr) {
    RelativeEncoder enc = mtr.getEncoder();
    enc.setPositionConversionFactor(FACTOR);
    enc.setVelocityConversionFactor(FACTOR /* / 60.0 */);
    return enc;
  }

  // Network tables
  class ShooterWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_desiredLeftMotorRPM;
    NetworkTableEntry nt_currentLeftMotorRPM;
    NetworkTableEntry nt_desiredRightMotorRPM;
    NetworkTableEntry nt_currentRightMotorRPM;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kF;

    // add nt for pos when we add it
    @Override
    public String getTableName() {
      return Shooter.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_desiredLeftMotorRPM = table.getEntry("desiredLeftMotorRPM");
      nt_currentLeftMotorRPM = table.getEntry("currentLeftMotorRPM");
      nt_desiredRightMotorRPM = table.getEntry("desiredRightMotorRPM");
      nt_currentRightMotorRPM = table.getEntry("currentRightMotorRPM");
      nt_kP = table.getEntry("kP");
      nt_kF = table.getEntry("kF");
    }

    public void ntupdate() {
      nt_desiredLeftMotorRPM.setDouble(desiredLeftRPM);
      nt_currentLeftMotorRPM.setDouble(currentLeftRPM);
      nt_desiredRightMotorRPM.setDouble(desiredRightRPM);
      nt_currentRightMotorRPM.setDouble(currentRightRPM);
      nt_kP.setDouble(hw_leftPid.getP());
      nt_kF.setDouble(hw_leftPid.getFF());
    }
  }
}
