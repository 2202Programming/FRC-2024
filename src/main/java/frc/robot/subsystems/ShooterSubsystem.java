// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.spi.CalendarNameProvider;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    //Instantiations 
  final CANSparkMax mtr = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);
  private SparkPIDController shooterPidController;
  private RelativeEncoder shooterEncoder;
  private double currentMotorSpeed;
  private double gearboxRatio = 3.0;
  private double kP = 0.001;
  private double kI = 0.0;
  private double kD = 0.0;

  public ShooterSubsystem() {
    motor_config(mtr, false);
    currentMotorSpeed = mtr.get();
    shooterPidController = mtr.getPIDController();
    shooterEncoder = mtr.getEncoder();
    shooterPidController.setP(kP);
    shooterPidController.setI(kI);
    shooterPidController.setD(kD);
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setMotorSpeed(double motorSpeed){
    mtr.set(motorSpeed);
    currentMotorSpeed = motorSpeed;
  }

  public void setShooterRPM(double shooterRPM){
    shooterPidController.setReference(shooterRPM*gearboxRatio, ControlType.kVelocity);
  }

  public double getMotorSpeed(){
    return currentMotorSpeed;
  }

  public double getMotorRPM(){
    return shooterEncoder.getVelocity();
  }

  public double getShooterRPM(){
    return shooterEncoder.getVelocity()*(1/gearboxRatio);
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
