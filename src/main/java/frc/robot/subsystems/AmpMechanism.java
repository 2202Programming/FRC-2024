// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;

public class AmpMechanism extends SubsystemBase {
   Servo left_servo = new Servo(PWM.LEFT_AMP_MECHANISM);
   Servo right_servo = new Servo(PWM.RIGHT_AMP_MECHANISM);
  /** Creates a new AmpMechanism. */
  public AmpMechanism() {
  }
  public void setServo(double cmdPos){
    left_servo.set(cmdPos);
    right_servo.set(cmdPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
