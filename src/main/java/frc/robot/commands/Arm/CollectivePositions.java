// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.Constants.PowerOnPos;
import frc.robot.commands.Arm.MoveCollectiveArm.Positions;

public enum CollectivePositions {
    // Arm  [0, 37] cm
    // Elbow [-20, 110] deg
    // Wrist [-130, 110] deg

    power_on(PowerOnPos.arm, 60.0), //placeholder all below
    travelNoPieceBS(0.0, 120.0),  
    // 4/11/23 + 2 deg instead of 5 so we dont trip stall detect
    travelLockNoPieceBS(0.0, 120.0); 


    // posistions and modes for target positions
    public Positions pos_info;

    CollectivePositions(double arm, double armMaxVel) {
      pos_info = new Positions(arm, armMaxVel);
    }

    // CollectivePositions(double arm) {
    //   pos_info = new Positions(arm);
    // }
  };