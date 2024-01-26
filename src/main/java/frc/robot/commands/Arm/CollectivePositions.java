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

    power_on(PowerOnPos.arm, PowerOnPos.elbow, 18.0, 60.0),
    travelNoPieceBS(PowerOnPos.arm, 10.0, 18.0, 120.0),  
    // 4/11/23 + 2 deg instead of 5 so we dont trip stall detect
    travelLockNoPieceBS(0.0, PowerOnPos.elbow,-1.0, 120.0), 
    safeToFlip(0.0, 70.0, -1.0, 120.0),
    
    //Travel with claw on "front"
    travelFS(0.0, -20.0,  -1.0, 120.0), 
    travelLockFS(0.0, -20.0, -1.0, 120.0), 

    // upright cone pickup position
    uprightConePickup(0.0, 15.0),
    uprightConeTravelHalfway(0.0, 85.0),

    // cube car wash to claw
    cubeToClaw(0.0, 10.0),  

    //dpl pulled values out of const.conePickup 
    pickupShelfFS(9.0, 110.0),
    haveConeAtShelf(9.0, 110.0),   //assumes wrist near zero
    
    //cone placeFrontside tilts up more than cube using frontSide tracking
    placeConeMidFS(12.0, 110.0),
    placeCubeMidFS(12.0, 110.0),

    // not in trackmode so to get 100 deg relative, take off the elbow angle
    placeConeHighFS(37.0, 110.0),  
    placeCubeHighFS(37.0, 105.0);
   

    //pickupTransitionFS(15.0, 105.0),
    //placeMidFS(20.0, 90.0);
    //testShelfTopFS(38.0, 165.0, -1.0, 70.0),
    //reversePickupShelfFS(15.0, -90.0),
    //midFS(20.0, 0.0),
    //midBS(20.0, 0.0),
    //placeHighFS(38.0, 105.0),
    //travelMidFS(20.0, -10.0),
    //travelMidBS(20.0, -10.0);

    // posistions and modes for target positions
    public Positions pos_info;

    CollectivePositions(double arm, double elbow, double armMaxVel, double elbowMaxVel) {
      pos_info = new Positions(arm, elbow, armMaxVel, elbowMaxVel);
    }

    CollectivePositions(double arm, double elbow) {
      pos_info = new Positions(arm, elbow);
    }
  };