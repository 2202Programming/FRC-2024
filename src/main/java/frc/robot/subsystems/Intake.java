package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Either we have the same setup everytime
 * Otherwise, call IntakeDefaultPos to go to limit switch pos
 * this then sets the limit switch pos to a number (probably 0)
 * every time
 */

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
// import frc.robot.Constants.DigitalIO;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class Intake extends SubsystemBase {
  final double wheelGearRatio = 10.0; // TODO set this correctly for intake speed - note vel [cm/s] - does this mean
                                      // anything or just gear raito works? (the comment before)
  final double AngleGearRatio = 100.0; // Gear ratio
  final double lower_clamp = 1.0; // TODO find both ofc
  final double upper_clamp = 15.0;
  /** Creates a new Intake. */
  public double intake_speed = 0.0;
  public double r_speed = 0.0;
  public double l_speed = 0.0;

  // Intake Angle, a servo
  final NeoServo angle_servo;
  final PIDFController hwAngleVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0); // inner (hw/vel) go up and divide by 2
  final PIDController anglePositionPID = new PIDController(1.0, 0.0, 0.0); // outer (pos)
  final PIDFController intakeVelPID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  // Intake roller motor
  final CANSparkMax intakeMtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
  final SparkPIDController intakeMtrPid;
  final RelativeEncoder intakeMtrEncoder;

  // lightgate tell us when we have a game piece (aka a Note)
  // final DigitalInput lightgate = new DigitalInput(DigitalIO.Intake_Note);

  // limit switch
  SparkLimitSwitch m_forwardLimit;
  SparkLimitSwitch m_reverseLimit;
  // Digital IO limit switches if we use
  // DigitalInput limitSwitchUp = new DigitalInput(DigitalIO.Intake_Up);
  // DigitalInput limitSwitchDown = new DigitalInput(DigitalIO.Intake_Down);

  public Intake() { // TODO: Get values
    final int STALL_CURRENT = 15; // [amp]
    final int FREE_CURRENT = 5; // [amp]
    final double maxVel = 5.0; // [deg/s]
    final double maxAccel = 5.0; // [deg/s^2]
    final double posTol = 3.0; // [deg]
    final double velTol = 1.0; // [deg/s]
    // servo controls angle of intake arm
    angle_servo = new NeoServo(CAN.ANGLE_MTR, anglePositionPID, hwAngleVelPID, false); // TODO: find invert
    // use velocity control on intake motor
    intakeMtr.clearFaults();
    intakeMtr.restoreFactoryDefaults();
    intakeMtrPid = intakeMtr.getPIDController();
    intakeMtrEncoder = intakeMtr.getEncoder();
    intakeMtrEncoder.setPositionConversionFactor(wheelGearRatio);
    intakeMtrEncoder.setVelocityConversionFactor(wheelGearRatio / 60.0); // min to sec
    intakeVelPID.copyTo(intakeMtr.getPIDController(), 0);

    // configure hardware pid with our values
    intakeMtr.burnFlash();

    /// Servo setup for angle_servo
    hwAngleVelPID.copyTo(angle_servo.getController().getPIDController(), 0);
    angle_servo.setConversionFactor((180.0 / Math.PI) / AngleGearRatio) // [deg]
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();

    this.setAngleClamp(lower_clamp, upper_clamp);

    // limit switch
    m_forwardLimit = angle_servo.getController().getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed); // TODO: Find out if it's normally closed or open
    m_reverseLimit = angle_servo.getController().getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed); // TODO: Verify that this get is a set
    // limit switches should be enabled on default, bottom two lines are just in
    // case right now
    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);
  }

  public void setIntakeSpeed(double speed) {
    // intakeMtr.set(speed);
    intakeMtrPid.setReference(speed, ControlType.kVelocity, 0);
  }

  // public boolean hasNote() {
  // return lightgate.get();
  // }

  public double getIntakeRollerSpeed() {
    return intakeMtrEncoder.getVelocity();
  }

  /* [deg] */
  public void setAngleSetpoint(double position) {
    angle_servo.setSetpoint(position);
  }

  /* [deg] */
  public double getAngleSetpoint() {
    return angle_servo.getSetpoint();
  }

  /* [deg] */
  public double getAnglePosition() {
    return angle_servo.getPosition();
  }

  public void setAnglePosition(double pos) {
    angle_servo.setPosition(pos);
  }

  /*
   * [deg/s]
   * Switches angle servo to velcoity mode
   * TESTING ONLY
   */
  public void setAngleVelocity(double speed) {
    angle_servo.setVelocityCmd(speed);
  }

  public double getAngleSpeed() {
    return angle_servo.getVelocity();
  }

  public void setAngleClamp(double min_ext, double max_ext) {
    angle_servo.setClamp(min_ext, max_ext);
  }

  public boolean angleAtSetpoint() {
    return angle_servo.atSetpoint(); // are we there yet?
  }

  public boolean atForwardLimitSwitch() {
    return m_forwardLimit.isPressed(); // do we need to check the other???
  }

  public boolean atReverseLimitSwitch() {
    return m_reverseLimit.isPressed();
  }

  public Command getWatcher() {
    return new IntakeWatcherCmd();
  }

  public void periodic() {
    this.angle_servo.periodic();

  }

  class IntakeWatcherCmd extends WatcherCmd {
    // NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_angleVel;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kI;
    NetworkTableEntry nt_kD;
    NetworkTableEntry nt_wheelVel;
    NetworkTableEntry nt_anglePos;
    NetworkTableEntry nt_forwardLimit;
    NetworkTableEntry nt_reverseLimit;

    @Override
    public String getTableName() {
      return Intake.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      // nt_lightgate = table.getEntry("lightgate");
      nt_angleVel = table.getEntry("angleVel");
      nt_kP = table.getEntry("kP");
      nt_kI = table.getEntry("kI");
      nt_kD = table.getEntry("kD");
      nt_wheelVel = table.getEntry("wheelVel");
      nt_anglePos = table.getEntry("anglePos");
      nt_forwardLimit = table.getEntry("forwardLimit");
      nt_reverseLimit = table.getEntry("reverseLimit");

      // default value for mutables
      // example nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      // nt_lightgate.setBoolean();
      nt_angleVel.setDouble(getAngleSpeed());
      nt_kP.setDouble(hwAngleVelPID.getP());
      nt_kI.setDouble(hwAngleVelPID.getI());
      nt_kD.setDouble(hwAngleVelPID.getD());
      nt_wheelVel.setDouble(getIntakeRollerSpeed());
      nt_anglePos.setDouble(getAnglePosition());
      nt_forwardLimit.setBoolean(m_forwardLimit.isPressed());
      nt_reverseLimit.setBoolean(m_reverseLimit.isPressed());

      // get mutable values
      // example maxArbFF = nt_maxArbFF.getDouble(maxArbFF);

    }
  } // watcher command

}
