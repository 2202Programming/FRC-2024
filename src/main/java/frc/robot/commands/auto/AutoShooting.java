// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.commands.Swerve.FaceToTag;
import frc.robot.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;

public class AutoShooting extends SequentialCommandGroup {

  private Limelight_Subsystem limelight;

  /**
   * Alliance aware sequence command to face to the target and shoot the notes.
   * If not seeing tags on target, command will be canceled.
   * 
   * @param target Enum target to shoot at.
   */
  public AutoShooting(ShootingTarget target) {
    limelight = RobotContainer.getSubsystem(Limelight_Subsystem.class);

    // TODO: We may need to check tags later, init(), of FSM isn't ready
    double tagID = determineTag(target);
    if (checkForTarget(tagID)) {
      addCommands(new FaceToTag(tagID));
      // get Position here and feed it to the ShooterToggle to adjust the RPM
      // I have this in AutoShooting branch now -KO
      // Shoot low for auto now
      addCommands(new ShooterSequence(false,3500));
    }
  }

  /**
   * Determine the tag to face based on the alliance and the target.
   * 
   * @param target operator pick from three
   * @return tagID to face
   */
  private double determineTag(ShootingTarget target) {
    //handle Optional<> from getAlliance()
    var allianceOpt = DriverStation.getAlliance();
    var alliance = DriverStation.Alliance.Blue;  //default
    if (allianceOpt.isEmpty()) {
        System.out.println("Warning: FSM report no alliance, using Blue.");        
    }
    else {
      //use what we are given
      alliance = allianceOpt.get();
    }

    if (alliance == DriverStation.Alliance.Blue) {
      // on Blue Alliance
      if (target == ShootingTarget.Speaker) {
        return 7;
      } else if (target == ShootingTarget.Amp) {
        return 6;
      } else {

        LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
        for (LimelightTarget_Fiducial tag : tags) {
          if (tag.fiducialID == 14) {
            return 14;
          } else if (tag.fiducialID == 15) {
            return 15;
          } else if (tag.fiducialID == 16) {
            return 16;
          }
        }
      }
    } else {
      // on Red Alliance
      if (target == ShootingTarget.Speaker) {
        return 4;
      } else if (target == ShootingTarget.Amp) {
        return 5;
      } else {
        LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
        for (LimelightTarget_Fiducial tag : tags) {
          if (tag.fiducialID == 11) {
            return 11;
          } else if (tag.fiducialID == 12) {
            return 12;
          } else if (tag.fiducialID == 13) {
            return 13;
          }
        }
      }
    }
    return 17;// DNE
  }

  /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  private boolean checkForTarget(double tagID) {
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == tagID) {
        return true;
      }
    }
    return false;
  }

  public enum ShootingTarget {
    Speaker,
    Amp,
    Trap
  }
}