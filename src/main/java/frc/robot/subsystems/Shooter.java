// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class Shooter extends SubsystemBase {

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
  // Instantiations 
  final CANSparkMax shooterMotorLeft = new CANSparkMax(CAN.SHOOTER_L, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax shooterMotorRight = new CANSparkMax(CAN.SHOOTER_R, CANSparkMax.MotorType.kBrushless);

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

  public Shooter() {
    motor_config(shooterMotorLeft, true);
    motor_config(shooterMotorRight, false);

    currentLeftMotorOutput = shooterMotorLeft.get();
    shooterLeftPidController = shooterMotorLeft.getPIDController();
    shooterLeftEncoder = shooterMotorLeft.getEncoder();
    System.out.println("Left Encoder Scaling Factor ="+shooterLeftEncoder.getVelocityConversionFactor());
    System.out.println("Left Encoder CPR ="+shooterLeftEncoder.getCountsPerRevolution());

    currentRightMotorOutput = shooterMotorRight.get();
    shooterRightPidController = shooterMotorRight.getPIDController();
    shooterRightEncoder = shooterMotorRight.getEncoder();
    System.out.println("Right Encoder Scaling Factor ="+shooterRightEncoder.getVelocityConversionFactor());
    System.out.println("Right Encoder CPR ="+shooterRightEncoder.getCountsPerRevolution());
    
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
    //This method will be called once per scheduler run
    currentLeftMotorRPM = shooterLeftEncoder.getVelocity();
    currentRightMotorRPM = shooterRightEncoder.getVelocity();

    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Current Shooter RPM", currentLeftMotorRPM / gearboxRatio); //should be 1.0
    SmartDashboard.putNumber("Current Left Motor RPM", currentLeftMotorRPM);
    SmartDashboard.putNumber("Current Right Motor RPM", currentRightMotorRPM);
    //these currently are not working
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

  public void setShooterRPM(double shooterLeftRPM, double shooterRightRPM){
    shooterLeftPidController.setReference(shooterLeftRPM*gearboxRatio, ControlType.kVelocity);
    shooterRightPidController.setReference(shooterRightRPM*gearboxRatio, ControlType.kVelocity);
    System.out.println("Motor goals changed, left="+shooterLeftRPM*gearboxRatio+", right="+shooterRightRPM*gearboxRatio); //just debug code, not needed
  }

  public double getLeftMotorOutput(){
    return currentLeftMotorOutput;
  }

  public double getRightMotorOutput(){
    return currentRightMotorOutput;
  }

  public double getMotorRPM(){
    return currentLeftMotorRPM;
    //here for MotorTriggerOrDash, don't know why we aren't just using getLeftMotorOutput
  }
  //PID getters/setters
  public double getShooterRPM(){
    return currentLeftMotorRPM / gearboxRatio; //should be 1.0
  }

  public void setP(double newP){
    shooterLeftPidController.setP(newP);
    shooterRightPidController.setP(newP);
  }

  public void setI(double newI){
    shooterLeftPidController.setI(newI);
    shooterRightPidController.setI(newI);
  }
  public void setD(double newD){
    shooterLeftPidController.setD(newD);
    shooterRightPidController.setD(newD);
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

  void motor_config(CANSparkMax mtr, boolean inverted) {
    mtr.clearFaults();
    mtr.restoreFactoryDefaults();
    mtr.setInverted(inverted);
 }

}
