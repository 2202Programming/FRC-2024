// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.commands.Swerve.FaceToTag;
import frc.robot.commands.Swerve.RotateTo;
import frc.robot.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;


public class TurnFaceShootAuto extends Command {
  double targetTagID;
  Limelight_Subsystem limelight;

  public TurnFaceShootAuto(int tagID) {
    targetTagID = tagID;
    limelight = RobotContainer.getSubsystem(Limelight_Subsystem.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if(checkForTarget()) { //target tag is visible
        new SequentialCommandGroup(
          new FaceToTag(targetTagID),
          new ShooterSequence(1000)
        ).schedule();
      } 
      else{ //target tag is not *yet* visible
        new SequentialCommandGroup(
          new RotateTo(),
          new FaceToTag(targetTagID),
          new ShooterSequence(1000)
        ).schedule();
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  private boolean checkForTarget() {
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == targetTagID) {
        return true;
      }
    }
    return false;
  }

}
