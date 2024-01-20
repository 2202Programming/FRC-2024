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
  final CANSparkMax mtr = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);
  private SparkPIDController shooterPidController;
  private RelativeEncoder shooterEncoder;
  private ShooterMode currentShootingMode;
  private double currentMotorSpeed;
  private double currentMotorRPM;
  private double gearboxRatio = 1.0;
  private double kP = 0.001;
  private double kI = 0.0;
  private double kD = 0.0;

  public ShooterSubsystem() {
    motor_config(mtr, true);
    currentMotorSpeed = mtr.get();
    shooterPidController = mtr.getPIDController();
    shooterEncoder = mtr.getEncoder();
    shooterPidController.setP(kP);
    shooterPidController.setI(kI);
    shooterPidController.setD(kD);
    currentShootingMode = ShooterMode.RPM;
    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Current Shooter RPM",0.0);
    SmartDashboard.putNumber("Current Motor RPM",0.0);
    SmartDashboard.putNumber("Current Motor %",0.0);
    SmartDashboard.putNumber("Current Motor RPM Goal",0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentMotorRPM = shooterEncoder.getVelocity();
    SmartDashboard.putString("Current Shoooting Mode",currentShootingMode.toString());
    SmartDashboard.putNumber("Current Shooter RPM", currentMotorRPM / gearboxRatio);
    SmartDashboard.putNumber("Current Motor RPM", currentMotorRPM);
    SmartDashboard.putNumber("Current Motor %", mtr.get());
    //SmartDashboard.putNumber("Current Motor RPM Goal",shooterPidController.);

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
    mtr.set(motorSpeed);
    currentMotorSpeed = motorSpeed;
  }

  public void setShooterRPM(double shooterRPM){
    shooterPidController.setReference(shooterRPM*gearboxRatio, ControlType.kVelocity);
    SmartDashboard.putNumber("Current Motor RPM Goal",shooterRPM*gearboxRatio);
  }

  public double getMotorSpeed(){
    return currentMotorSpeed;
  }

  public double getMotorRPM(){
    return currentMotorRPM;
  }

  public double getShooterRPM(){
    return currentMotorRPM / gearboxRatio;
  }

  public void setP(double newP){
    shooterPidController.setP(newP);
  }

  public void setI(double newI){
    shooterPidController.setP(newI);
  }
  public void setD(double newD){
    shooterPidController.setP(newD);
  }

  public double getP() {
    return shooterPidController.getP();
  }

  public double getI() {
    return shooterPidController.getI();
  }

  public double getD() {
    return shooterPidController.getD();
  }

  void motor_config(CANSparkMax mtr, boolean inverted) {
    mtr.clearFaults();
    mtr.restoreFactoryDefaults();
    mtr.setInverted(inverted);
 }

}
