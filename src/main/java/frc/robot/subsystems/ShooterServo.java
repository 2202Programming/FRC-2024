package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.CAN;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class ShooterServo extends Shooter {

  final static double ShooterAngleGearRatio = 350.0;
  final static double ShooterAngleRadius = 12.0; // [cm]
  final static int STALL_CURRENT = 0; // [amps]
  final static int FREE_CURRENT = 25;
  final static double SERVO_MIN = 0.0; // [cm]
  final static double SERVO_MAX = 12.0; // [cm]
  final static double maxVel = 20.0; // [cm/sec]
  final static double maxAccel = 10.0; // [cm/sec^2]
  final static double posTol = 1.0; // [cm]
  final static double velTol = 1.0; // [cm/s]
  final static double MIN_POSITION = 20.2184; // [cm]
  final static double MAX_POSITION = 32.4866; // [cm]
  final static double Hypotenuse = 42.53992; // [cm]
  public final static double MIN_DEGREES = 28.52; // [deg]
  public final static double MAX_DEGREES = 48.00;
  double cmd_deg; // [deg] angle we want the shooter (calculate extension from this)

  final static double DeployAngle = MAX_DEGREES;  // compatibility with pnumantic shooter
  final static double RetractAngle = MIN_DEGREES; // compatibility with pnumantic shooter
  
  private final NeoServo extension;
  PIDController shooterPos = new PIDController(6.0, 0.0, 0.0);
  PIDFController hwShooterVelPID = new PIDFController(0.2, 0.0, 0.0, 0.14);

  public ShooterServo() {
    super(false);
    extension = new NeoServo(CAN.SHOOTER_ANGLE, shooterPos, hwShooterVelPID, false);

    // Servo setup for angle_servo
    hwShooterVelPID.copyTo(extension.getController().getPIDController(), 0);
    extension.setConversionFactor(ShooterAngleRadius / ShooterAngleGearRatio) // [cm/rot]
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
    extension.setClamp(SERVO_MIN, SERVO_MAX); // local [cm]
    extension.setPosition(SERVO_MIN); //power on pos
  }

  @Override
  public void periodic() {
    super.periodic();
    extension.periodic();
  }

  @Override
  public WatcherCmd getWatcher() {
    return new ShooterServoWatcherCmd();
  }

  @Override
  public void deploy() {
    setAngleSetpoint(DeployAngle);
  }

  @Override
  public void retract() {
    setAngleSetpoint(RetractAngle);
  }

  public void setAngleSetpoint(double angle) {
    cmd_deg = angle;
    double rad_angle = angle * Math.PI / 180.0;
    double convertedPos = Math.sin(rad_angle) * Hypotenuse - MIN_POSITION; // [cm]
    System.out.println(convertedPos);
    extension.setSetpoint(convertedPos);
  }

  /* 
   * Extension velocity [cm/s]
  */
  public double getExtensionVelocity() {
    return extension.getVelocity();
  }

  /*
   * Sets extension positon [cm], does not move.
   * Used for calibration.
   */
  public void setExtensionPosition(double pos) {
    extension.setPosition(pos);
  }

  public double getAngle() { // [deg]
    double convertedPos = (extension.getPosition() + MIN_POSITION) / Hypotenuse;
    return Math.asin(convertedPos) * 180.0 / Math.PI;
  }

  public void setShooterAngleMaxVel(double velLimit) {
    extension.setMaxVelocity(velLimit);
  }

  public double getShooterAngleSpeed() {
    return extension.getVelocity();
  }

  public boolean atSetpoint() {
    return extension.atSetpoint();
  }

  /*
   * set extension velocity in [cm/s]
   */
  public void setExtensionVelocity(double vel) {
    extension.setVelocityCmd(vel);
  }

  public double getCurrent() {
    return extension.getController().getOutputCurrent();
  }

  public class ShooterServoWatcherCmd extends ShooterWatcherCmd {
    NetworkTableEntry nt_cmd_extension;
    NetworkTableEntry nt_extensionVel;
    NetworkTableEntry nt_cmd_extensionVel;

    NetworkTableEntry nt_angle_cmd;
    NetworkTableEntry nt_angle;
  
    NetworkTableEntry nt_atSetpoint;
  
    NetworkTableEntry nt_current;
    NetworkTableEntry nt_current_percent;
    NetworkTableEntry nt_RPM;
    NetworkTableEntry nt_at_RPM;

    // Commanded RPM for testing
    NetworkTableEntry nt_test_cmd_rpm;
    int test_cmd_rpm = 0;

    public String getTableName() {
      return super.getTableName();
    }

    @Override
    public void ntcreate() {
      NetworkTable table = getTable();
      super.ntcreate();
      nt_cmd_extension = table.getEntry("Extension_cmd");
      nt_extensionVel = table.getEntry("ExtensionVel");
      nt_cmd_extensionVel = table.getEntry("ExtensionVel_cmd");

      nt_angle_cmd = table.getEntry("Angle_cmd");
      nt_angle = table.getEntry("Angel");

      nt_atSetpoint = table.getEntry("atSetpoint");
      
      nt_current = table.getEntry("nt_output_I");
      nt_current_percent = table.getEntry("nt_output_percent");
      nt_RPM = table.getEntry("RPM");

      nt_at_RPM = table.getEntry("at_RPM");
    }

    @Override
    public void ntupdate() {
      super.ntupdate();
      nt_cmd_extension.setDouble( extension.getSetpoint());
      nt_extensionVel.setDouble(getExtensionVelocity());
      nt_cmd_extensionVel.setDouble(extension.getVelocityCmd());

      nt_angle_cmd.setDouble(cmd_deg);
      nt_angle.setDouble(getAngle());

      nt_atSetpoint.setBoolean(atSetpoint());
      nt_at_RPM.setBoolean(isAtRPM(100));
      nt_current.setDouble(extension.getController().getOutputCurrent());
      nt_current_percent.setDouble(extension.getController().getAppliedOutput());
      nt_RPM.setDouble(extension.getVelocity() / (ShooterAngleRadius / ShooterAngleGearRatio));
    }
  }
}
