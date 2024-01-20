// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public enum ShooterMode {
    Trigger("Trigger"), Percent("Percent"), RPM("RPM");

    public final String name;

    ShooterMode(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
   }
  }

    //Instantiations 

  final TalonFX shooterMotorLeft = new TalonFX(10);
  final TalonFX shooterMotorRight = new TalonFX(11);

  private ShooterMode currentShootingMode;
  private double currentLeftMotorOutput;
  private double currentRightMotorOutput;


  public ShooterSubsystem() {

    shooterMotorLeft.setInverted(false);
    shooterMotorRight.setInverted(false);
    
    currentShootingMode = ShooterMode.Trigger;
    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Current Shooter RPM",0.0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Left Motor Percent", getLeftMotorOutput());
    SmartDashboard.putNumber("Right Motor Percent", getRightMotorOutput());
  }

  public void setShootingMode(ShooterMode mode){
    currentShootingMode = mode;
  }

  public void cycleShootingMode() {
    if(currentShootingMode == ShooterMode.Trigger) {
      currentShootingMode = ShooterMode.Percent;
      return;
    }
    if(currentShootingMode == ShooterMode.Percent) { 
      currentShootingMode = ShooterMode.RPM;
      return;
    }
    if(currentShootingMode == ShooterMode.RPM) 
      currentShootingMode = ShooterMode.Trigger;
    return;
  }

  public ShooterMode getShooterMode() {
    return currentShootingMode;
  }

  public void setMotorSpeed(double motorSpeed){
    shooterMotorLeft.set(motorSpeed);
    shooterMotorRight.set(motorSpeed);
    currentLeftMotorOutput = motorSpeed;
  }

  public double getLeftMotorOutput(){
    currentLeftMotorOutput = shooterMotorLeft.get();
    return currentLeftMotorOutput;
  }

  public double getRightMotorOutput(){
    currentRightMotorOutput = shooterMotorRight.get();
    return currentRightMotorOutput;
  }



}
