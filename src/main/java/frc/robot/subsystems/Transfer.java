// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
/*
 * NOTE:
 * All commented out shuffleboard pieces will be fixed when a motor is added to the transfer piece.
 */
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {

  public enum TransferShootingMode {
    Trigger("Trigger"), Percent("Percent"), RPM("RPM");

    public final String name;

    TransferShootingMode(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }
  }

  final CANSparkMax transferMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);

  private SparkPIDController transferPidController;
  private RelativeEncoder transferEncoder;
  private TransferShootingMode currentTransferShootingMode;
  private double transferMotorOutput;
  private double transferMotorRPM;
  private double gearboxRatio = 1.0;
  private double kP = 0.001;
  private double kI = 0.0;
  private double kD = 0.0;

  /* Creates a new Transfer. */
  public Transfer() {
  
    motor_config(transferMotor, false);
    transferMotorOutput = transferMotor.get();
    transferPidController = transferMotor.getPIDController();
    transferEncoder = transferMotor.getEncoder();

    transferPidController.setP(kP);
    transferPidController.setI(kI);
    transferPidController.setD(kD);

    currentTransferShootingMode = TransferShootingMode.RPM;
    //SmartDashboard.putString("Current Control Mode",currentTransferShootingMode.toString());
    //SmartDashboard.putNumber("Current Transfer Motor RPM",0.0);

    
  }

  //TODO: find out methods/behaviors, pneumatics, etc. 

  public boolean isNoteReady(){

    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    transferMotorRPM = transferEncoder.getVelocity();
    //SmartDashboard.putString("Transfer Control Mode",currentTransferShootingMode.toString());
    //SmartDashboard.putNumber("Current Transfer RPM", transferMotorRPM / gearboxRatio);
    //SmartDashboard.putNumber("Current Transfer Motor RPM", transferMotorRPM);
    //SmartDashboard.putNumber("Transfer Motor Percent", transferMotorOutput);
  }

  public void setShootingMode(TransferShootingMode mode) {
    currentTransferShootingMode = mode;
  }

  public void cycleShootingMode() {
    if(currentTransferShootingMode == TransferShootingMode.Trigger) {
      currentTransferShootingMode = TransferShootingMode.Percent;
      return;
    }
    if(currentTransferShootingMode == TransferShootingMode.Percent) { 
      currentTransferShootingMode = TransferShootingMode.RPM;
      return;
    }
    if(currentTransferShootingMode == TransferShootingMode.RPM) 
      currentTransferShootingMode = TransferShootingMode.Trigger;
    return;
  }

  public TransferShootingMode getShooterMode() {
    return currentTransferShootingMode;
  }

  public double transferMotorOutput() {
    return transferMotorOutput;
  }

  public double transferMotorRPM() {
    return transferMotorRPM;
  }

  public double transferRPM() {
    return transferMotorRPM / gearboxRatio;
  }

  public void setP(double newP){
    transferPidController.setP(newP);
  }

  public void setI(double newI){
    transferPidController.setP(newI);
  }
  public void setD(double newD){
    transferPidController.setP(newD);
  }

  public double getP() {
    return transferPidController.getP();
  }

  public double getI() {
    return transferPidController.getI();
  }

  public double getD() {
    return transferPidController.getD();
  }

  void motor_config(CANSparkMax transferMotor, boolean inverted) {
    transferMotor.clearFaults();
    transferMotor.restoreFactoryDefaults();
    transferMotor.setInverted(inverted);
  }
}
