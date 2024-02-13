// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.ShooterToggle;
import frc.robot.commands.Swerve.FaceToTag;

public class AutoShooting extends SequentialCommandGroup {
  int tagID;
  /**
   * Creates a new AutoShooting. 
   */
  public AutoShooting() {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      tagID = 7;
    }
    else{
      tagID = 4;
    }
    addCommands(new FaceToTag(tagID));
    //get Position here and feed it to the ShooterToggle to adjust the RPM
    addCommands(new ShooterToggle());
    }
}
