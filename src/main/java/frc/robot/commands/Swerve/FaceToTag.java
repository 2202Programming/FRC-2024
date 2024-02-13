// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class FaceToTag extends Command {
  private final SwerveDrivetrain drivetrain;
  private final Limelight_Subsystem limelight;

  double xSpeed, ySpeed, rot;
  SwerveModuleState[] vision_out;
  ChassisSpeeds zero_cs = new ChassisSpeeds(0.0, 0.0, 0.0);

  // PID for retroreflective-based heading to a target
  PIDController centeringPid;
  double centering_kP = 2.0;
  double centering_kI = 0.0;
  double centering_kD = 0.0;
  double centeringPidOutput = 0.0;

  Rotation2d currentAngle;
  double min_rot_rate = 6.0;

  final double vel_tol = 2.0;
  final double pos_tol = 2.0;
  final double max_rot_rate = 60.0; // [deg/s]

  final double high_tape_Y = 21.0;
  final double mid_tape_Y = 0.0;
  final double high_tape_goal = -16.0;
  final double mid_tape_goal = -24.6;
  final double max_yaw_error = 15.0; // max number of degrees the target can be off and we still think it's legit
  boolean lastValidity = false;
  boolean currentValidity = false;
  private Timer timer;
  private int validityCount = 0;
  private boolean valid_tag = false;
  private double TagID;
  private final SwerveDriveKinematics kinematics;
  private SwerveModuleState[] no_turn_states;

  /**
   * Creates a new FaceToTag.
   * 
   * @param TagID AprilTag ID to face
   */
  public FaceToTag(double TagID) {

    limelight = RobotContainer.getSubsystem(Limelight_Subsystem.class);
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

    addRequirements(drivetrain);

    // end if not seeing desired tag
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == TagID) {
        end(true);
        break;
      }
    }

    centeringPid = new PIDController(centering_kP, centering_kI, centering_kD);
    centeringPid.setTolerance(pos_tol, vel_tol);
    timer = new Timer();
    this.TagID = TagID;
    kinematics = drivetrain.getKinematics();
    no_turn_states = kinematics.toSwerveModuleStates(zero_cs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    System.out.println("FaceToTag: initialize, initial heading: " + drivetrain.getPose().getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] output;
    lastValidity = currentValidity;
    currentValidity = limelight.valid();
    validityCount = (currentValidity) ? validityCount + 1 : 0;
    if (validityCount > 5) {// calculate every 5 frames
      calculate();
      valid_tag = true;
    }

    output = (valid_tag) ? vision_out : no_turn_states;
    drivetrain.drive(output);
  }

  private void calculate() {
    // getting value from limelight
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    double tagXfromCenter = 0;
    boolean hasTarget = false;
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == TagID) {
        tagXfromCenter = tag.tx;
        hasTarget = true;
        break;
      }
    }
    if (hasTarget) {// this should be true all the time unless the tag is lost
      // TODO: CHECK right not it is assuming that camera is in center
      // need a position of robot to get distance from the tag to calculate the
      // desired angle to face the tag with offset
      // I dont know if we want to use the robot Pose but that is the only way if
      // limelight is not in the center of the robot
      centeringPidOutput = centeringPid.calculate(tagXfromCenter, 0.0);
      double min_rot = Math.signum(centeringPidOutput) * min_rot_rate;
      rot = MathUtil.clamp(centeringPidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3; // convert to radians
      vision_out = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          rot,
          drivetrain.getPose().getRotation()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return centeringPid.atSetpoint() || timer.hasElapsed(3.0);// end if it takes more than 3 sec
  }
}