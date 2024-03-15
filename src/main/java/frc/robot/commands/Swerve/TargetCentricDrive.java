package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.Tag_Pose;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors.Limelight_Subsystem;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

/*
  Driver controls the robot using field coordinates.
    X,Y, Rotation
*/
public class TargetCentricDrive extends Command {

  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final HID_Xbox_Subsystem dc;
  private final Limelight_Subsystem limelight;
  private final Intake intake;

  private PIDController blindPid;
  private final double blindPid_kp = 0.05;
  private final double blindPid_ki = 0.0;
  private final double blindPid_kd = 0.0;
  private double targetRot;
  private Pose2d currentPose;
  private Translation2d targetPose; // Position want to face to

  PIDController centeringPid;
  double centering_kP = 3.5;
  double centering_kI = 0;
  double centering_kD = 0;
  double centeringPidOutput = 2.0;
  double vel_tol = 1.0;
  double pos_tol = 1.0;
  double max_rot_rate = 45.0; // [deg/s]
  double min_rot_rate = 6.0;
  private double TagID;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public TargetCentricDrive(double TagID) {
    this.dc = RobotContainer.getSubsystem("DC"); // driverControls
    this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
    this.TagID = TagID;
    limelight = RobotContainer.getSubsystem(Limelight_Subsystem.class);

    targetPose = Tag_Pose.tagLocations[(int) TagID];

    // PID for when tag is in view
    centeringPid = new PIDController(centering_kP, centering_kI, centering_kD);
    centeringPid.setTolerance(pos_tol, vel_tol);

    // PID for when tag is not visable
    blindPid = new PIDController(blindPid_kp, blindPid_ki, blindPid_kd);
    blindPid.enableContinuousInput(-180.0, 180.0);
    blindPid.setTolerance(pos_tol, vel_tol);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    if (intake.hasNote()) {
      if (checkForTarget(TagID)) { // has note, can see target tag, close loop via limelight
        calculateRotFromTarget();
      } else { // has note, but can't see target, use odometery
        calculateRotFromOdometery();
      }
    } else {
      calculateRotFromJoystick(); // otherwise human controls rotation
    }
    calculate();  // X and Y from joysticks, and rotation from one of methods above
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  private void calculateRotFromOdometery() {
        currentPose = drivetrain.getPose();
        targetRot = (Math.atan2(currentPose.getTranslation().getY() - targetPose.getY(),
            currentPose.getTranslation().getX() - targetPose.getX())) // [-pi, pi]
            * 180 / Math.PI - 180;
        rot = blindPid.calculate(currentPose.getRotation().getDegrees(), targetRot);
  }

  private void calculateRotFromTarget() {
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

      centeringPidOutput = centeringPid.calculate(tagXfromCenter, 0.0);
      double min_rot = Math.signum(centeringPidOutput) * min_rot_rate;
      rot = MathUtil.clamp(centeringPidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3; // convert to radians

    }
  }

  void calculateRotFromJoystick() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    rot = MathUtil.clamp(rot, -DriveTrain.kMaxAngularSpeed, DriveTrain.kMaxAngularSpeed);

  }

  void calculate() {
    // X and Y from joysticks; rot from previous calculations
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;

    // Clamp speeds from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -DriveTrain.kMaxSpeed, DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -DriveTrain.kMaxSpeed, DriveTrain.kMaxSpeed);

    currrentHeading = drivetrain.getPose().getRotation();
    // convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = (DriverStation.getAlliance().get().equals(Alliance.Blue))
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading)
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, currrentHeading); // if on red alliance you're
                                                                                         // looking at robot from
                                                                                         // opposite. Pose is in blue
                                                                                         // coordinates so flip if red

    output_states = kinematics.toSwerveModuleStates(tempChassisSpeed);
  }

  private boolean checkForTarget(double tagID) {
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == tagID) {
        return true;
      }
    }
    return false;
  }

}