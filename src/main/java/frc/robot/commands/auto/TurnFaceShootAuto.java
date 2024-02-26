// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Tag_Pose;
import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class TurnFaceShootAuto extends Command {

  public enum Phase {
    Init("Init"),
    TurningBlind("TurningBlind"),
    CenteringTag("CenteringTag"),
    Shooting("Shooting");

    String name;

    private Phase(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }

  }

  
  private Limelight_Subsystem limelight;
  private SwerveDrivetrain drivetrain;
  private Pose2d currentPose;

  Phase currentPhase;
  Translation2d targetTranslation;
  int targetTagID;
  double angle_tolerance = 1.0; //degrees
  double currentTagOffset;
  double currentRotationError;
  double currentTargetRotation;

  private PIDController pid;
  private final double kp = 0.04;
  private final double ki = 0.0;
  private final double kd = 0.0;
  private SwerveDriveKinematics kinematics;
  private final double vel_tol = 1.0;
  private SwerveModuleState[] outputModuleState;
  private ShooterSequence shootCommand;
  private boolean finished;

  public TurnFaceShootAuto(int tagID) {
    targetTagID = tagID;
    targetTranslation = Tag_Pose.tagLocations[tagID];
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    limelight = RobotContainer.getSubsystem(Limelight_Subsystem.class);
    addRequirements(drivetrain);
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(angle_tolerance, vel_tol);
    kinematics = drivetrain.getKinematics();
    shootCommand = new ShooterSequence(1000);
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPhase = Phase.Init;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drivetrain.getPose();

    if (currentPhase != Phase.Shooting) { //once we start shooting, just keep shooting until done

      currentRotationError = rotationErrorFromPose();

      if(checkForTarget(targetTagID)) { //target tag is visible

        currentTagOffset = tagOffset(targetTagID);

        if(currentTagOffset > angle_tolerance){ //tag is not centered enough
          currentPhase = Phase.CenteringTag;
        }
        else { //tag is visible and centered enough
          currentPhase = Phase.Shooting;
        }
      }
      else { //tag is not visible
        if(currentRotationError < angle_tolerance){ //can't see tag, but internal concept of rotation is within tolerance
          currentPhase = Phase.Shooting;
        }
        else{ //can't see tag, and internal concept of rotation is above tolerance
          currentPhase = Phase.TurningBlind;
        }
      }
    }

    switch (currentPhase) {
      case CenteringTag:
        calculate(pid.calculate(currentTagOffset, 0.0)); //feed pid the x offset of visible tag, goal is centered (0.0)
        drivetrain.drive(outputModuleState);
      break;
      case TurningBlind:
        calculate(pid.calculate(currentPose.getRotation().getDegrees(), currentTargetRotation)); //feed pid our rotation with goal of target rotation.
        drivetrain.drive(outputModuleState);  
      break;
      case Shooting:
        if(!shootCommand.isScheduled()){ //presumable first time through for case shooting, schedule the shoot command but only once
          shootCommand.schedule();
        }
        else if(shootCommand.isFinished()){ //shoot command is done, so we can finish this whole command
          finished = true;
        }
      break;
    }
  }

  private void calculate(double pidOutput) {
    outputModuleState = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        pidOutput,
        currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  private boolean checkForTarget(int tagID) {
    double targetTag = tagID;
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == targetTag) {
        return true;
      }
    }
    return false;
  }

  private double tagOffset(int tagID){
    // getting tag offset from limelight
    double targetTag = tagID;
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    double tagXfromCenter = 0;

    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == targetTag) {
        tagXfromCenter = tag.tx;
        break;
      }
    }
    return tagXfromCenter;
  }

  private double rotationErrorFromPose(){
    Pose2d currentPose = drivetrain.getPose();

    //goal rotation in degrees to face target
    currentTargetRotation = Math.atan2(
        currentPose.getTranslation().getY() - targetTranslation.getY(),
        currentPose.getTranslation().getX() - targetTranslation.getX()) // [-pi, pi]
        * 180 / Math.PI - 180;

    //return the difference between current rotation and goal rotation
    return (currentPose.getRotation().getDegrees() - currentTargetRotation);
  }

}
