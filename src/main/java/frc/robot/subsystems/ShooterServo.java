package frc.robot.subsystems;

import frc.robot.subsystems.Swerve.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Tag_Pose;
import frc.robot.RobotContainer;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.DistanceInterpretor;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class ShooterServo extends Shooter {
  // tbd for all units
  final Transfer transfer;
  final static double ShooterAngleGearRatio = 350.0;
  final static double ShooterAngleRadius = 12.0; // [cm]
  final static int STALL_CURRENT = 5; // [amps]
  final static int FREE_CURRENT = 25;
  final static double SERVO_MIN = 0.0;
  final static double SERVO_MAX = 12.0;
  final static double maxVel = 20.0; // [cm/sec]
  final static double maxAccel = 10.0; // [cm/sec^2]
  final static double posTol = 1.0; // [cm]
  final static double velTol = 1.0; // [cem/s]
  final double MIN_POSITION = 20.2184; // [cm]
  final double MAX_POSITION = 32.4866; // [cm]
  final double MIN_DEGREES = 28.52; // [deg]
  final double MAX_DEGREES = 48.00;
  double cmd_deg;

  private double distanceToTarget;
  private double targetAngle;
  private DistanceInterpretor distanceInterpretor;

  private Translation2d targetTranslation2d;
  boolean atLimit;
  int limitCount;
  boolean prev_moving;
  // enable for actual testing
  boolean auto_move_test = false;
  // double conversionFactor = Math.sin(360.0 / ShooterAngleGearRatio) *
  // Hypotenuse - MIN_POSITION;

  final static double DeployAngle = 100.0;// tbd both
  final static double RetractAngle = 50.0;
  final static double Hypotenuse = 42.53992; // [cm]
  private final NeoServo shooterAngle;

  PIDController shooterPos = new PIDController(6.0, 0.0, 0.0);
  PIDFController hwShooterVelPID = new PIDFController(0.2, 0.0, 0.0, 0.14);

  public ShooterServo() {
    super(false);
    shooterAngle = new NeoServo(CAN.SHOOTER_ANGLE, shooterPos, hwShooterVelPID, false);
    transfer = RobotContainer.getSubsystem(Transfer.class);
    // Servo setup for angle_servo
    hwShooterVelPID.copyTo(shooterAngle.getController().getPIDController(), 0);
    shooterAngle.setConversionFactor(ShooterAngleRadius / ShooterAngleGearRatio) // [cm/rot]
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
    shooterAngle.setClamp(SERVO_MIN, SERVO_MAX); // local [cm]
    shooterAngle.setPosition(SERVO_MIN); //power on pos
    distanceInterpretor = new DistanceInterpretor();
    // set default target
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // Blue Alliance
      targetTranslation2d = Tag_Pose.ID7;
    } else {
      // Red Alliance
      targetTranslation2d = Tag_Pose.ID4;
    }

  }

  @Override
  public void periodic() {
    super.periodic();
    shooterAngle.periodic();
    calculateTargetAngle();

    if (transfer.hasNote() && auto_move_test) {
      setShooterAngleSetpoint(targetAngle);
    } else if (!transfer.hasNote() && auto_move_test) {
      // if no note, shooter needs to be low to allow transfer of loading note
      setShooterAngleSetpoint(0.0); // placeholder (ideal transfer location between shooter and intake)
    }
  }

  @Override
  public WatcherCmd getWatcher() {
    return new ShooterServoWatcherCmd();
  }

  private void calculateTargetAngle() {
    distanceToTarget = RobotContainer.getSubsystem(SwerveDrivetrain.class)
        .getDistanceToTranslation(targetTranslation2d);
    targetAngle = distanceInterpretor.getAngleFromDistance(distanceToTarget);

    SmartDashboard.putNumber("Distance to Target", distanceToTarget);
    SmartDashboard.putNumber("Goal Angle for target", targetAngle);

  }

  public void setTargetTranslation(Translation2d targetTranslation2d) {
    this.targetTranslation2d = targetTranslation2d;
  }

  @Override
  public void deploy() {
    setShooterAngleSetpoint(DeployAngle);
  }

  @Override
  public void retract() {
    setShooterAngleSetpoint(RetractAngle);
  }

  public void setShooterAngleSetpoint(double angle) {
    cmd_deg = angle;
    double rad_angle = angle * Math.PI / 180.0;
    double convertedPos = Math.sin(rad_angle) * Hypotenuse - MIN_POSITION; // [cm]
    System.out.println(convertedPos);
    shooterAngle.setSetpoint(convertedPos);
  }
  public double getShooterAngleDegSetpoint(){
    return cmd_deg;
  }

  public double getShooterAngleSetpoint() {
    return shooterAngle.getSetpoint();
  }

  public double getShooterAngleVelocity() {
    return shooterAngle.getVelocity();
  }

  public void setShooterAnglePosition(double pos) {
    shooterAngle.setPosition(pos);
  }

  public double getShooterAnglePosition() { // [deg]
    double convertedPos = (shooterAngle.getPosition() + MIN_POSITION) / Hypotenuse;
    return Math.asin(convertedPos) * 180.0 / Math.PI;
  }

  public void setShooterAngleMaxVel(double velLimit) {
    shooterAngle.setMaxVelocity(velLimit);
  }

  public double getShooterAngleSpeed() {
    return shooterAngle.getVelocity();
  }

  public boolean atShooterAngleSetpoint() {
    return shooterAngle.atSetpoint();
  }

  public void setShooterAngleVelocity(double vel) {
    shooterAngle.setVelocityCmd(vel);
  }

  public boolean atLimit() {
    limitCount = 0;
    return atLimit;
  }

  public void zeroEncoder() {
    shooterAngle.setPosition(0.0);
  }

  public double getCurrent() {
    return shooterAngle.getController().getOutputCurrent();
  }

  class ShooterServoWatcherCmd extends ShooterWatcherCmd {
    NetworkTableEntry nt_desiredPosition;
    NetworkTableEntry nt_currentVel;
    NetworkTableEntry nt_currentPos;
    NetworkTableEntry nt_atSetpoint;
    NetworkTableEntry nt_desiredVel;
    NetworkTableEntry nt_CmdPosDegrees;
    NetworkTableEntry nt_desiredDegPosition;
    NetworkTableEntry nt_current;
    NetworkTableEntry nt_current_percent;
    NetworkTableEntry nt_RPM;

    public String getTableName() {
      return super.getTableName();
    }

    @Override
    public void ntcreate() {
      NetworkTable table = getTable();
      super.ntcreate();
      nt_desiredPosition = table.getEntry("CMDPositionCM");
      nt_desiredDegPosition = table.getEntry("CMDPosDegrees");
      nt_currentVel = table.getEntry("MEAVelCMS");
      nt_currentPos = table.getEntry("MEAPosDEG");
      nt_atSetpoint = table.getEntry("atSetpoint");
      nt_desiredVel = table.getEntry("CMDVelCMS");
      nt_CmdPosDegrees = table.getEntry("CmdPosDegrees");
      nt_current = table.getEntry("nt_current");
      nt_current_percent = table.getEntry("nt_current_percent");
      nt_RPM = table.getEntry("RPM");

    }

    @Override
    public void ntupdate() {
      super.ntupdate();
      nt_desiredPosition.setDouble(getShooterAngleSetpoint());
      nt_desiredDegPosition.setDouble(getShooterAngleDegSetpoint());
      nt_currentVel.setDouble(getShooterAngleVelocity());
      nt_currentPos.setDouble(getShooterAnglePosition());
      nt_atSetpoint.setBoolean(atShooterAngleSetpoint());
      nt_desiredVel.setDouble(shooterAngle.getVelocityCmd());
      nt_current.setDouble(shooterAngle.getController().getOutputCurrent());
      nt_current_percent.setDouble(shooterAngle.getController().getAppliedOutput());
      nt_RPM.setDouble(shooterAngle.getVelocity() / (ShooterAngleRadius / ShooterAngleGearRatio));
    }
  }
}
