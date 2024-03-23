package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/


public class RobotCentricDrive extends Command {

  final SwerveDrivetrain drivetrain;
  final HID_Xbox_Subsystem dc;
  final SwerveDriveKinematics kinematics;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  double log_counter = 0;

  public RobotCentricDrive(SwerveDrivetrain drivetrain, HID_Xbox_Subsystem dc) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    this.kinematics = drivetrain.getKinematics();
  }


  @Override
  public void initialize() {
    SmartDashboard.putBoolean("RobotCentricDrive Enabled", true);
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -DriveTrain.kMaxSpeed, DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -DriveTrain.kMaxSpeed, DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -DriveTrain.kMaxAngularSpeed, DriveTrain.kMaxAngularSpeed);

    output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("RobotCentricDrive Enabled", false);
    drivetrain.stop();
  }

}
