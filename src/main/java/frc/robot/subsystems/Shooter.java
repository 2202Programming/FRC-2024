// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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

  private double cmdLeftRPM;
  private double cmdRightRPM;
  private double measLeftRPM;
  private double measRightRPM;

  PIDFController pidConsts = new PIDFController(0.00005, 0.000000085, 0.006, kF);  //slot 0  - normal
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
    measLeftRPM = leftEncoder.getVelocity();
    measRightRPM = rightEncoder.getVelocity();
  }

  public boolean isAtRPM(double tolerance) {
    return Math.abs(cmdLeftRPM - measLeftRPM) < tolerance
        && Math.abs(cmdRightRPM - measRightRPM) < tolerance;
  }


  public void setRPM(double leftRPM, double rightRPM) {
    // slot 0 --> normal op, slot 1 --> free spin
    int slot =0; // (leftRPM == 0.0 && rightRPM == 0.0) ? 1 : 0;
    if (leftRPM == 0.0 && rightRPM == 0.0) {
      hw_leftPid.setIAccum(0.0);
      hw_rightPid.setIAccum(0.0);
      slot = 1;
    } 
    
    hw_leftPid.setReference(leftRPM, ControlType.kVelocity, slot);
    hw_rightPid.setReference(rightRPM, ControlType.kVelocity, slot);
    cmdLeftRPM = leftRPM;
    cmdRightRPM = rightRPM;
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
    mtrpid.setIMaxAccum(0.0, 1);
    mtr.setInverted(inverted);
    mtr.setIdleMode(IdleMode.kBrake);
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
    NetworkTableEntry nt_cmdLeftMotorRPM;
    NetworkTableEntry nt_measLeftMotorRPM;
    NetworkTableEntry nt_cmdRightMotorRPM;
    NetworkTableEntry nt_measRightMotorRPM;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kF;

    // add nt for pos when we add it
    @Override
    public String getTableName() {
      return Shooter.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdLeftMotorRPM = table.getEntry("cmdLeftMotorRPM");
      nt_measLeftMotorRPM = table.getEntry("measLeftMotorRPM");
      nt_cmdRightMotorRPM = table.getEntry("cmdRightMotorRPM");
      nt_measRightMotorRPM = table.getEntry("measRightMotorRPM");
      nt_kP = table.getEntry("kP");
      nt_kF = table.getEntry("kF");
    }

    public void ntupdate() {
      nt_cmdLeftMotorRPM.setDouble(cmdLeftRPM);
      nt_measLeftMotorRPM.setDouble(measLeftRPM);
      nt_cmdRightMotorRPM.setDouble(cmdRightRPM);
      nt_measRightMotorRPM.setDouble(measRightRPM);
      nt_kP.setDouble(hw_leftPid.getP());
      nt_kF.setDouble(hw_leftPid.getFF());
    }
  }
}
