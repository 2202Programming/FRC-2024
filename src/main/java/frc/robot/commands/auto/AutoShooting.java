// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Shooter.ShooterServoSequence;
import frc.robot.commands.Shooter.SpeakerShooter;
import frc.robot.commands.Swerve.FaceToTag;
import frc.robot.commands.Swerve.RotateUntilSeeTags;
import frc.robot.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.Constants.Tag_Pose;

public class AutoShooting extends SequentialCommandGroup {

  private Limelight_Subsystem limelight;
  SwerveDrivetrain drivetrain;

  /**
   * Alliance aware sequence command to face to the target and shoot the notes.
   * If not seeing tags on target, command will be canceled.
   * 
   * @param target Enum target to shoot at.
   */
  public AutoShooting(ShootingTarget target) {
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    limelight = RobotContainer.getSubsystem(Limelight_Subsystem.class);

    double tagID = determineTag(target);

    addCommands(new RotateUntilSeeTags((int)tagID));
    addCommands(new FaceToTag(tagID));
    if (target == ShootingTarget.Speaker) {
      addCommands(new SpeakerShooter());
    } else if (target == ShootingTarget.Amp) {
      addCommands(new ShooterServoSequence(45.0, 800.0));
    } else {
      // Trap
      addCommands(new ShooterServoSequence(45.0, 1000.0));
    }
  }

  /**
   * 
   * @return Phase(Holder of RPM and angle) of shooting position to the Speaker
   *         (Alliance Aware)
   */
  private SpeakerShootingPhase getSpeakerPhase() {
    // Assuming that limelight has updated
    double difference;
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // Blue Alliance
      difference = Math.sqrt(
          Math.pow(drivetrain.getPose().getTranslation().getX() - Tag_Pose.ID7.getX(), 2)
              + Math.pow(drivetrain.getPose().getTranslation().getY() - Tag_Pose.ID7.getY(), 2));
    } else {
      // Red Alliance
      difference = Math.sqrt(
          Math.pow(drivetrain.getPose().getTranslation().getX() - Tag_Pose.ID4.getX(), 2)
              + Math.pow(drivetrain.getPose().getTranslation().getY() - Tag_Pose.ID4.getY(), 2));
    }

    if (difference < 2.0) {
      return SpeakerShootingPhase.Phase1;
    } else if (difference < 4.0) {
      return SpeakerShootingPhase.Phase2;
    } else {
      return SpeakerShootingPhase.Phase3;
    }
  }

  /**
   * Determine the tag to face based on the alliance and the target.
   * 
   * @param target operator pick from three
   * @return tagID to face
   */
  private double determineTag(ShootingTarget target) {
    // handle Optional<> from getAlliance()
    var allianceOpt = DriverStation.getAlliance();
    var alliance = DriverStation.Alliance.Blue; // default
    if (allianceOpt.isEmpty()) {
      System.out.println("Warning: FSM report no alliance, using Blue.");
    } else {
      // use what we are given
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
    System.out.println("Invalid in AutoShooting");
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

  /**
   * Phase of shooting position to the Speaker This hold the RPM and the angle
   */
  public enum SpeakerShootingPhase {
    Phase1(true, 2000.0), // Close
    Phase2(false, 2000.0), // Mid
    Phase3(false, 3500.0);// Far

    boolean high;
    double RPM;

    SpeakerShootingPhase(boolean high, double RPM) {
      this.high = high;
      this.RPM = RPM;
    }

    public boolean isHigh() {
      return high;
    }

    public double getRPM() {
      return RPM;
    }
  }
}