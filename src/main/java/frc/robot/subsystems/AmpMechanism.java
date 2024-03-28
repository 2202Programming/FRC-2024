// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpMechanism extends SubsystemBase {
   Servo mServo = new Servo(0);
  /** Creates a new AmpMechanism. */
  public AmpMechanism() {
  }
  public void setServo(double cmdPos){
    mServo.set(cmdPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
