// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    /* 1/20/24
     * CAN IDs:
     * Corner 3 Drive Motor BL: 22
     * Corner 3 Direction Motor BL: 23
     * BL Encoder: 28
     * 
     * Corner 2 Drive Motor FL: 20
     * Corner 2 Direction Motor FL: 21
     * FL Encoder: 29
     * 
     * Corner 4 Drive Motor BR: 24
     * Corner 4 Direction Motor BR: 25
     * BR Encoder
     * 
     * Corner 1 Direction Motor FR: 27
     * Corner 1 Drive Motor FR: 26
     * FR Encoder: 30
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
