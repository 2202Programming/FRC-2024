// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.spi.CalendarNameProvider;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
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
  final CANSparkMax shooterMotorLeft = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax shooterMotorRight = new CANSparkMax(21, CANSparkMax.MotorType.kBrushless);
  private SparkPIDController shooterLeftPidController;
  private SparkPIDController shooterRightPidController;
  private RelativeEncoder shooterLeftEncoder;
  private RelativeEncoder shooterRightEncoder;
  private ShooterMode currentShootingMode;
  private double currentLeftMotorOutput;
  private double currentRightMotorOutput;
  private double currentLeftMotorRPM;
  private double currentRightMotorRPM;
  private double gearboxRatio = 1.0;
  private double kP = 0.001;
  private double kI = 0.0;
  private double kD = 0.0;

  public ShooterSubsystem() {
    motor_config(shooterMotorLeft, true);
    motor_config(shooterMotorRight, true);
    currentLeftMotorOutput = shooterMotorLeft.get();
    shooterLeftPidController = shooterMotorLeft.getPIDController();
    shooterLeftEncoder = shooterMotorLeft.getEncoder();

    shooterRightPidController = shooterMotorRight.getPIDController();
    shooterRightEncoder = shooterMotorRight.getEncoder();

    shooterLeftPidController.setP(kP);
    shooterLeftPidController.setI(kI);
    shooterLeftPidController.setD(kD);

    shooterRightPidController.setP(kP);
    shooterRightPidController.setI(kI);
    shooterRightPidController.setD(kD);

    currentShootingMode = ShooterMode.RPM;
    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Current Shooter RPM",0.0);
    SmartDashboard.putNumber("Current Left Motor RPM",0.0);
    SmartDashboard.putNumber("Current Right Motor RPM",0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentLeftMotorRPM = shooterLeftEncoder.getVelocity();
    currentRightMotorRPM = shooterRightEncoder.getVelocity();
    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Current Shooter RPM", currentLeftMotorRPM / gearboxRatio);
    SmartDashboard.putNumber("Current Left Motor RPM", currentLeftMotorRPM);
    SmartDashboard.putNumber("Current Right Motor RPM", currentRightMotorRPM);
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

  public void setShooterRPM(double shooterLeftRPM, double shooterRightRPM){
    shooterLeftPidController.setReference(shooterLeftRPM*gearboxRatio, ControlType.kVelocity);
    shooterRightPidController.setReference(shooterLeftRPM*gearboxRatio, ControlType.kVelocity);
  }

  public double getLeftMotorSpeed(){
    return currentLeftMotorOutput;
  }

  public double getRightMotorSpeed(){
    return currentRightMotorOutput;
  }

  public double getMotorRPM(){
    return currentLeftMotorRPM;
  }

  public double getShooterRPM(){
    return currentLeftMotorRPM / gearboxRatio;
  }

  public void setP(double newP){
    shooterLeftPidController.setP(newP);
    shooterRightPidController.setP(newP);
  }

  public void setI(double newI){
    shooterLeftPidController.setP(newI);
    shooterRightPidController.setP(newI);
  }
  public void setD(double newD){
    shooterLeftPidController.setP(newD);
    shooterRightPidController.setP(newD);
  }

  public double getP() {
    return shooterLeftPidController.getP();
  }

  public double getI() {
    return shooterLeftPidController.getI();
  }

  public double getD() {
    return shooterLeftPidController.getD();
  }

  void motor_config(CANSparkMax shooterMotorLeft, boolean inverted) {
    shooterMotorLeft.clearFaults();
    shooterMotorLeft.restoreFactoryDefaults();
    shooterMotorLeft.setInverted(inverted);
 }

}
