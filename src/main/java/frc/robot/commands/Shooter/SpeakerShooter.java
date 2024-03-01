// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tag_Pose;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class SpeakerShooter extends Command {
  private SwerveDrivetrain drivetrain;
  private ShooterServo shooter;
  private boolean finished = false;

  // TODO: Move this to Constnats
  private final double SHOOTER_Y_OFFSET = 0.0; // [m] pivotal point of shooter from the center(direction of shooter is +)
  private final double SHOOTER_Z_OFFSET = 0.0; // [m] shooter z position from floor
  private final double SPEAKER_HEIGHT = 0.0; // [m] speaker height from the floor

  private final double angle_adjustment = 0.0; // [deg] angle gain/lose for tuning
  private final double[][] reg_ind_val = {{0,0},{0,0}}; // [radius, rot] use value from collectData mode
  private final double[] reg_dep_val = {0,0}; // [rpm] shooter speed

  // RobotPose with polar coordinate(origin is tag)
  private double radius; // [m] distance from the speaker to the robot
  // [deg] 0 is the condition where the robot is in front of the speaker
  // +- 90 degree from that zero. but uses absolute value becuase of simiticity of
  // shooting condition.
  // So range of this is 0-90 which simlply means it is just how far off from the
  // front of the speaker
  private double rot;

  //shooter angle [deg]
  private double shooter_angle;

  // Switch to collect data for regression
  private boolean collectData = true;
  /**
   * Command shooting speaker
   * <p>Requires: ShooterServo and limelight detecting specified tag.
   * Calculate the angle and RPM to shoot the speaker</p>
   * <p>Shooter angle- will be calculated by the distance and geometry.</p>
   * RPM- will be calculated using formula based of polar coordinate determined
   * the target is the vertex of the polar coordinate
   */
  public SpeakerShooter() {
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // Blue Alliance
      radius = Math.sqrt(
          Math.pow(drivetrain.getPose().getTranslation().getX() - Tag_Pose.ID7.getX(), 2)
              + Math.pow(drivetrain.getPose().getTranslation().getY() - Tag_Pose.ID7.getY(), 2));
      rot = Math.abs(Math.atan(
          Math.abs(drivetrain.getPose().getTranslation().getY() - Tag_Pose.ID7.getY()) /
              Math.abs(drivetrain.getPose().getTranslation().getX() - Tag_Pose.ID7.getX())))
          * 180 / Math.PI;
    } else {
      // Red Alliance
      radius = Math.sqrt(
          Math.pow(drivetrain.getPose().getTranslation().getX() - Tag_Pose.ID4.getX(), 2)
              + Math.pow(drivetrain.getPose().getTranslation().getY() - Tag_Pose.ID4.getY(), 2));
      rot = Math.abs(Math.atan(
          Math.abs(drivetrain.getPose().getTranslation().getY() - Tag_Pose.ID4.getY()) /
              Math.abs(drivetrain.getPose().getTranslation().getX() - Tag_Pose.ID4.getX())))
          * 180 / Math.PI;
    }
    shooter_angle = Math.atan((SPEAKER_HEIGHT - SHOOTER_Z_OFFSET) /(radius - SHOOTER_Y_OFFSET));
    //TODO: set shooter angle to shooter_angle
    // shooter.setShooterAngle(shooter_angle); maybe like this?
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(collectData){
      //for data collection
      //run RPMShooter to test the shooting RPM and put value in the array
      System.out.println("[" + radius + "," + rot + "]");
      finished = true;
    }
    else{
      if(shooter.atShooterAngleSetpoint()){
        //TODO: implement shooter.setRPM() to set RPM
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

}