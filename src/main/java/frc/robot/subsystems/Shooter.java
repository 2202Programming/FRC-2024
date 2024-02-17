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

  private final DoubleSolenoid shooterAngle;

  private double desiredLeftRPM;
  private double desiredRightRPM;
  private double currentLeftRPM;
  private double currentRightRPM;

  PIDFController pidConsts = new PIDFController(0.001, 0.0, 0.0, 0.0);

  public Shooter() {
    hw_leftPid = motor_config(leftMtr, pidConsts, true);
    hw_rightPid = motor_config(rightMtr, pidConsts, false);

    leftEncoder = leftMtr.getEncoder();
    rightEncoder = rightMtr.getEncoder();
    shooterAngle = new DoubleSolenoid(PneumaticsModuleType.REVPH, PCM1.Forward, PCM1.Reverse);
  }

  @Override
  public void periodic() {
    currentLeftRPM = leftEncoder.getVelocity();
    currentRightRPM = rightEncoder.getVelocity();
  }

  public boolean isAtRPM(int tolerance){
    return Math.abs(desiredLeftRPM - currentLeftRPM) < tolerance && Math.abs(desiredRightRPM - currentRightRPM) < tolerance;
  }

  @Deprecated
  public void setSpeed(double foo) {}

  public void setRPM(double leftRPM, double rightRPM) {
    hw_leftPid.setReference(leftRPM, ControlType.kVelocity);
    hw_rightPid.setReference(rightRPM, ControlType.kVelocity);
    desiredLeftRPM = leftRPM;
    desiredRightRPM = rightRPM;
  }

  public double getLeftMotorRPM() {
    return currentLeftRPM;
  }

  public double getRightMotorRPM() {
    return currentRightRPM;
  }
  public double getDesiredLeftRPM(){
    return desiredLeftRPM;
  }
  public double getDesiredRightRPM(){
    return desiredRightRPM;
  }


  public void deployPneumatics(){
    shooterAngle.set(DoubleSolenoid.Value.kForward);
  }
  
  public void retractPneumatics(){
    shooterAngle.set(DoubleSolenoid.Value.kReverse);
  }

  SparkPIDController motor_config(CANSparkMax mtr, PIDFController pid, boolean inverted) {
    mtr.clearFaults();
    mtr.restoreFactoryDefaults();
    var mtrpid = mtr.getPIDController();
    pid.copyChangesTo(mtrpid, 0, pid);
    mtr.setInverted(inverted);
    return mtrpid;
  }
  

  // Network tables
  class ShooterWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_desiredLeftMotorRPM;
    NetworkTableEntry nt_currentLeftMotorRPM;
    // add nt for pos when we add it
    @Override
    public String getTableName(){
      return Shooter.this.getName();
    }
    public void ntcreate(){
      NetworkTable table = getTable();
      nt_desiredLeftMotorRPM = table.getEntry("desiredLeftMotorRPM");
      nt_currentLeftMotorRPM = table.getEntry("currentLeftMotorRPM");
    }
    public void ntupdate(){
      nt_desiredLeftMotorRPM.setDouble(getDesiredLeftRPM());
      nt_currentLeftMotorRPM.setDouble(getLeftMotorRPM());
    }
  }


  /*TODO: FOR SHOOTER TUNING 
  AFTER FINISHING PID TUNING DELETE FOLLOWING*/
  public double getP(){
    return hw_leftPid.getP();
  }
  public double getI(){
    return hw_leftPid.getI();
  }
  public double getD(){
    return hw_leftPid.getD();
  }
  public void setP(double p){
    hw_leftPid.setP(p);
    hw_rightPid.setP(p);
  }
  public void setI(double i){
    hw_leftPid.setI(i);
    hw_rightPid.setI(i);
  }
  public void setD(double d){
    hw_leftPid.setD(d);
    hw_rightPid.setD(d);
  }
}
