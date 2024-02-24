// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.Transfer_Constants;

import com.revrobotics.CANSparkMax;

public class Transfer extends SubsystemBase {
  
  DigitalInput lightGate = new DigitalInput(DigitalIO.TRANSFER_LIGHT_GATE);
  CANSparkMax transferMotor;

  /** Creates a new Transfer. */
  public Transfer() {
      transferMotor = new CANSparkMax(CAN.TRANSFER_MOTOR, CANSparkMax.MotorType.kBrushless);
  }

  //TODO: find out methods/behaviors, pneumatics, etc. 

  public boolean isLightGateBlocked(){
    return lightGate.get(); 
  }
  public void transferMotorOn() {
    transferMotor.set(Transfer_Constants.TRANSFER_MOTOR_ON);
  }
  // Motor speed will likely need to be changed
  public void transferMotorOff() {
    transferMotor.set(Transfer_Constants.TRANSFER_MOTOR_OFF);
  }
  public void transferMotorReverse() {
    transferMotor.set(Transfer_Constants.TRANSFER_MOTOR_REVERSE);
  }
  public double getTransferVelocity(){
    return transferMotor.get();
  }
  // This motor speed will also probably need to be changed too, but make sure it is still a negative number
  // This method would be used to spit the note back out if it gets jammed, but might not be necessary
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
