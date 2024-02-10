// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import com.revrobotics.CANSparkMax;

/*
 * NOTE:
 * All commented out shuffleboard pieces will be fixed when a motor is added to the transfer piece.
 */
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Transfer extends SubsystemBase {
  
  DigitalInput lightGate = new DigitalInput(DigitalIO.TRANSFER_LIGHT_GATE);
  CANSparkMax transferMotor;

  /** Creates a new Transfer. */
  public Transfer() {
      transferMotor = new CANSparkMax(CAN.TRANSFER_MOTOR, CANSparkMax.MotorType.kBrushless);
  }

  // TODO: find out methods/behaviors, pneumatics, etc.

  public boolean isLightGateBlocked(){
    return lightGate.get(); 
  }
  public void transferMotorOn() {
    transferMotor.set(1);
  }
  // Motor speed will likely need to be changed
  public void transferMotorOff() {
    transferMotor.set(0);
  }
  public void transferMotorReverse() {
    transferMotor.set(-1);
  }
  // This motor speed will also probably need to be changed too, but make sure it is still a negative number
  // This method would be used to spit the note back out if it gets jammed, but might not be necessary
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    transferMotorRPM = transferEncoder.getVelocity();
    // SmartDashboard.putString("Transfer Control
    // Mode",currentTransferShootingMode.toString());
    // SmartDashboard.putNumber("Current Transfer RPM", transferMotorRPM /
    // gearboxRatio);
    // SmartDashboard.putNumber("Current Transfer Motor RPM", transferMotorRPM);
    // SmartDashboard.putNumber("Transfer Motor Percent", transferMotorOutput);
  }

  public void setShootingMode(TransferShootingMode mode) {
    currentTransferShootingMode = mode;
  }

  public void cycleShootingMode() {
    if (currentTransferShootingMode == TransferShootingMode.Trigger) {
      currentTransferShootingMode = TransferShootingMode.Percent;
      return;
    }
    if (currentTransferShootingMode == TransferShootingMode.Percent) {
      currentTransferShootingMode = TransferShootingMode.RPM;
      return;
    }
    if (currentTransferShootingMode == TransferShootingMode.RPM)
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

  public void setP(double newP) {
    transferPidController.setP(newP);
  }

  public void setI(double newI) {
    transferPidController.setP(newI);
  }

  public void setD(double newD) {
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
