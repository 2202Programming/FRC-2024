package frc.robot.subsystems;

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
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

public class Intake extends SubsystemBase {
  public static final double UpPos = 0.0; // [deg]
  public static final double ShootingPos = 20.0; // [deg]
  public static final double DownPos = 96.0; // [deg]
  public static final double TravelUp = 120.0; // [deg/s]
  public static final double TravelDown = 60.0; // [deg/s]
  public static final double EncoderOffset = 10.0; // todo Offset and the default pos

  /*
   * S - shooter
   * I - intake
   * T - transfer
   * O - outside
   * 2 - to
   * F - from
   */
  enum intakeNoteState {
    hasNote, hasNoNote, O2I, T2I, I2T, I2O, O2T
  }

  enum noteLocation {
    outside, intake, transfer, shooter, none
  }

  // External encoder used
  // https://www.revrobotics.com/rev-11-1271/
  // https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders/alternate-encoder-mode
  static final int Angle_kCPR = 8192; // alt encoder angle [counts per rotation]

  final double wheelGearRatio = 1.0; // TODO set this correctly for intake speed - note vel [cm/s] - does this mean

  // anything or just gear raito works? (the comment before)
  final double AngleGearRatio = 500.0; // Gear ratio

  /** Creates a new Intake. */
  public double intake_speed = 0.0;
  double desired_intake_speed = 0.0;
  // Intake Angle, a servo
  final NeoServo angle_servo;
  
  //PIDFController hwAngleVelPID = new PIDFController(/* 0.002141 */0.010, 0.00003, 0.0, /* 0.00503 */0.0045); //internal vel
  PIDFController hwAngleVelPID = new PIDFController(0.0050, 0.0000, 0.0, 0.0075); //3/2/24 improved vel loop match (new encoder)
  
  /* inner (hw/vel) go up and divide by 2*/
  //final PIDController anglePositionPID = new PIDController(4.0, 0.0, 0.0); // outer (pos) for internal enc
  final PIDController anglePositionPID = new PIDController(1.0, 0.0, 0.0); // outer (pos) ext enc


  // Intake roller motor
  final CANSparkMax intakeMtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
  final PIDFController intakeVelPID = new PIDFController(0.0, 0.0, 0.0, 0.0); // wip - use pwr for sussex
  final SparkPIDController intakeMtrPid;
  final RelativeEncoder intakeMtrEncoder;

  // lightgate tell us when we have a game piece (aka a Note)
  final DigitalInput lightgate = new DigitalInput(DigitalIO.Intake_LightGate);

  // limit switch
  SparkLimitSwitch m_forwardLimit;
  SparkLimitSwitch m_reverseLimit;
  // Digital IO limit switches if we use
  DigitalInput limitSwitchUp = new DigitalInput(DigitalIO.IntakeIsUp);
  DigitalInput limitSwitchDown = new DigitalInput(DigitalIO.IntakeIsDown);

  //Note State variables
  boolean holdNote;  // true, well watch for Note edge.
  boolean senseNote_prev = false; // for edge detection 
  boolean hasNote = false; // true when Intake has Note

  public Intake() { // TODO: Get values
    final int STALL_CURRENT = 15; // [amp]
    final int FREE_CURRENT = 5; // [amp]
    double maxVel = 120.0; // [deg/s]
    final double maxAccel = 20.0; // [deg/s^2]
    final double posTol = 2.0; // [deg]
    final double velTol = 1.0; // [deg/s]

    // servo controls angle of intake arm, setup for velocity mode on brushless motor
    angle_servo = new NeoServo(CAN.ANGLE_MTR, 
      //MotorType.kBrushless,// uncomment for alt enc  
      anglePositionPID, hwAngleVelPID,
      // Type.kQuadrature, Angle_kCPR, // uncomment for alt enc   
      true, 0);

    // use velocity control on intake motor
    intakeMtr.clearFaults();
    intakeMtr.restoreFactoryDefaults();
    intakeMtr.setInverted(true);
    intakeMtrPid = intakeMtr.getPIDController();
    intakeMtrEncoder = intakeMtr.getEncoder();
    intakeMtrEncoder.setPositionConversionFactor(wheelGearRatio);
    intakeMtrEncoder.setVelocityConversionFactor(wheelGearRatio / 60.0); // min to sec
    intakeVelPID.copyTo(intakeMtr.getPIDController(), 0);

    // configure hardware pid with our values
    intakeMtr.burnFlash();

    /// Servo setup for angle_servo
    hwAngleVelPID.copyTo(angle_servo.getController().getPIDController(), 0);
    angle_servo
        .setConversionFactor(360.0 / AngleGearRatio) // [deg] for internal encoder behind gears
        //.setConversionFactor(360.0)   // [deg] external encoder on arm shaft
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();

    // Add external Encoder for position, but use Vel mode on inner loop
    angle_servo.addAltPositionEncoder( Type.kQuadrature, Angle_kCPR, 360.0);
    
    // power on
    setAnglePosition(UpPos);
    angle_servo.setClamp(UpPos, DownPos + 5.0);

    // limit switch config
    // Cannot have alternate encoder and limit switches- error from lib
    // m_forwardLimit =
    // angle_servo.getController().getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // m_reverseLimit =
    // angle_servo.getController().getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // m_forwardLimit.enableLimitSwitch(false);
    // m_reverseLimit.enableLimitSwitch(false);
    // m_forwardLimit.enableLimitSwitch(true);
    // m_reverseLimit.enableLimitSwitch(true);
  }

  // TODO: change is we start with the note in the intake; assumes that we do not
  // have the note in the intake at the start of the game
  // ?? TODO/Question why are these states not set in the constructor or placed
  // with the other globals??
  intakeNoteState state = intakeNoteState.hasNoNote;
  noteLocation location = noteLocation.none;
  intakeNoteState requestedState;
  // this is the 'state' that we want when the motors turn off
  IntakeMotorHelper myIntakeMotorHelper = new IntakeMotorHelper();
  LightgateHelper myLightgateHelper = new LightgateHelper();

  public void setIntakeSpeed(double speed) {
    intakeMtr.set(speed); //[%pwr] TODO change to velocity mode & tune hwpid for intakeMtr
    // intakeMtrPid.setReference(speed, ControlType.kVelocity, 0);
  }

  boolean senseNote() {
    return !lightgate.get();
  }

  public boolean hasNote() {
    if(state == intakeNoteState.hasNote) {
      return true;
    }
    return false;
  }

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

  /*
   * sets anglePosision to given value, doesn't move the intake.
   * Useful for calibration with limits or power up.
   */
  public void setAnglePosition(double pos) {
    angle_servo.setPosition(pos);
  }

  public void setMaxVelocity(double velLimit) {
    angle_servo.setMaxVelocity(velLimit);
  }

  /*
   * [deg/s]
   * Switches angle servo to velcoity mode
   * Used for test and calibration.
   */
  public void setAngleVelocity(double speed) {
    desired_intake_speed = speed;
    angle_servo.setVelocityCmd(speed);
  }

  public double getDesiredVelocity() {
    return desired_intake_speed;
  }

  public double getAngleSpeed() {
    return angle_servo.getVelocity();
  }

  public boolean angleAtSetpoint() {
    return angle_servo.atSetpoint(); // are we there yet?
  }

  public boolean atForwardLimitSwitch() {
    return !limitSwitchDown.get(); 
  }

  public boolean atReverseLimitSwitch() {
    return !limitSwitchUp.get();
  }

  public boolean limitSwitchEnabled() {
    return m_reverseLimit.isLimitSwitchEnabled();
  }

  public boolean forwardSwitchEnabled() {
    return m_forwardLimit.isLimitSwitchEnabled();
  }

  public Command getWatcher() {
    return new IntakeWatcherCmd();
  }

  public void setHasNote(boolean state) {
    // if we ever lose a note, call this
    hasNote = state;
    senseNote_prev = false;
  }

  /*
   * sets holdNote, when true the intake will stop moving when note is in 
   * its holding position.
   * 
   * If set to true, assumes we don't have the note and clears
   * any note state so we can watch for it.
   */
  public void setHoldNote(boolean holdNote)  {
    this.holdNote = holdNote;
    senseNote_prev = false;
    if (holdNote) setHasNote(false);
  }

  public void periodic() {
    this.angle_servo.periodic();  // do child objects first

    //@Ben uncomment for testing
    // processNoteDetection();
  

    // don't bother tracking note edge if we aren't going to hold it
    if (!holdNote) return;

    // if we get here, we need to hold on to the note
    // moniter lightgate signal we have the note
    if (!hasNote) {
      if (senseNote()) {
        senseNote_prev = true;
      } else if (senseNote_prev) {
        // high to low edge seen, we have it. Cmd will use timer to position correctly
        // and stop the intakeMtr based on direction and timing.
        hasNote = true;
      }
    }
  }

  /*
   * Ben et.al. I moved your processing here so it is self contained
   */
  @SuppressWarnings("unused")
  private void processNoteDetection() {
    /*
     * S - shooter
     * I - intake
     * T - transfer
     * O - outside
     * 2 - to
     * F - from
     */
    myIntakeMotorHelper.setMotorState(getIntakeRollerSpeed());

     //nothing to do if intake motor is off
    if (!myIntakeMotorHelper.isMotorOn()) return;

    if (myIntakeMotorHelper.motorHasToggledOn()) {
      // checks if motor is toggled on, not actually on
      myLightgateHelper.resetThrows();
    }

    myLightgateHelper.setLightgateHelperState(senseNote());

    switch (state) {
      case hasNoNote:
        if (myIntakeMotorHelper.isMotorPositiveDirection()) { // RPM (positive direction; intaking a note; going towards
                                                              // shooter)
          state = intakeNoteState.O2I;
          System.out.println("We have no note and the motors are in a positive direction.");

          break;

        } else if (myIntakeMotorHelper.isMotorNegativeDirection()) { // RPM (negative direction, getting a note from
                                                                     // transfer)
          state = intakeNoteState.T2I;
          System.out.println("We have no note and the motors are in a negative direction.");
          break;    

        } else {
          System.out.println("*********Error: we are not in the correct place.");
          break;
        }
      case hasNote:
        if (myIntakeMotorHelper.isMotorPositiveDirection()) { // RPM
          state = intakeNoteState.I2T;
          System.out.println("We have a note and the motors are in a positive direction.");
          break;

        } else if (myIntakeMotorHelper.isMotorNegativeDirection()) {
          state = intakeNoteState.I2O;
          System.out.println("We have anote and the motors are in a negative direction.");
          break;

        } else {
          System.out.println("ERROR: we are not in the right place.");
          break;
        }
      case O2I:
        // outside to intake
        if (myLightgateHelper.isSingleThrow()) {
          state = intakeNoteState.hasNote;
          System.out.println("We are in the outside to intake case.");
        }
        break;

      case T2I:
        // transfer to intake
        if (myLightgateHelper.isDoubleThrow()) {
          state = intakeNoteState.hasNote;
          System.out.println("We are in the transfer nto intake case.");
        }
        break;

      case I2T:
        // intake to transfer
        if (myLightgateHelper.isSingleThrow()) {
          state = intakeNoteState.hasNoNote;
          System.out.println("We are in the intake to transfer case.");
        }
        break;

      case O2T:
      //outside to transfer
      if(myLightgateHelper.isSingleThrow()) {
        state = intakeNoteState.hasNote;
        System.out.println("We are in the outside to transfer case with a single throw.");
      }

      if(myLightgateHelper.isDoubleThrow()) {
        state = intakeNoteState.hasNoNote;
        System.out.println("We are in the outside to transfer case with a double throw.");
      }

      case I2O:
        state = intakeNoteState.hasNoNote;
        System.out.println("We are in the intake to outside code.");
        break;

      default:
        System.out.println(
            "*********ERROR: We are in the default case. Something has gone wrong.");
        break;
    }
  }

  class IntakeMotorHelper {
    enum motorState {
      positive, negative, off;
    }

    motorState currentMotorState = motorState.off;
    motorState previousMotorState = motorState.off;

    boolean isMotorOn() {
      return motorState.off != currentMotorState;
    }

    boolean isMotorPositiveDirection() {
      return motorState.positive == currentMotorState;
    }

    boolean isMotorNegativeDirection() {
      return motorState.negative == currentMotorState;
    }

    void setMotorState(double motorSpeed) {
      previousMotorState = currentMotorState;
      if (motorSpeed > 10) {
        currentMotorState = motorState.positive;
        return;
      }
      if (motorSpeed < -10) {
        currentMotorState = motorState.negative;
        return;
      }
      currentMotorState = motorState.off;

      return;
    }

    boolean motorHasToggledOff() {
      return (currentMotorState == motorState.off) && (previousMotorState != motorState.off);
      // This compares currentMotorState and previous motor state
    }

    boolean motorHasToggledOn() {
      return (currentMotorState != motorState.off) && (previousMotorState == motorState.off);
    }
  }

  class LightgateHelper {
    boolean previousLightgateState = false;
    // A 'throw' is a rising edge in the lightgate - when the lightgate is broken,
    // not when it's unbroken
    int numberOfThrows = 0;

    boolean isSingleThrow() {
      return numberOfThrows == 1;
    }

    boolean isDoubleThrow() {
      return numberOfThrows == 2;
    }

    void resetThrows() {
      // use to reset single/double throw
      numberOfThrows = 0;
    }

    void setLightgateHelperState(boolean lightgateState) {
      if (lightgateState == true && previousLightgateState == false) {
        numberOfThrows++;
      }
      previousLightgateState = lightgateState;
    }
  }

  public void resetNoteLightgate() {
    myLightgateHelper.resetThrows();
  }

  /*
   * Watcher commmand puts network table data for intake.
   * 
   */
  class IntakeWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_hasNote;
    NetworkTableEntry nt_senseNote_prev;
    NetworkTableEntry nt_angleVel;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kI;
    NetworkTableEntry nt_kD;
    NetworkTableEntry nt_wheelVel;
    NetworkTableEntry nt_anglePos;
    NetworkTableEntry nt_angleCmd;
    NetworkTableEntry nt_forwardLimit;
    NetworkTableEntry nt_reverseLimit;
    NetworkTableEntry nt_reverseLimitSwitchEnabled;
    NetworkTableEntry nt_forwardLimitSwitchEnabled;
    NetworkTableEntry nt_desiredSpeed;
    NetworkTableEntry nt_lightgate;

    @Override
    public String getTableName() {
      return Intake.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_lightgate = table.getEntry("dio_Note");
      nt_angleVel = table.getEntry("angleVel");
      nt_kP = table.getEntry("kP");
      nt_kI = table.getEntry("kI");
      nt_kD = table.getEntry("kD");
      nt_wheelVel = table.getEntry("wheelVel");
      nt_anglePos = table.getEntry("anglePos");
      nt_angleCmd = table.getEntry("anglePosCmd");
      nt_forwardLimit = table.getEntry("dio_LimitFwd");
      nt_reverseLimit = table.getEntry("dio_LimitRev");
      //nt_reverseLimitSwitchEnabled = table.getEntry("reverseLimitEnabled");
      //nt_forwardLimitSwitchEnabled = table.getEntry("forwardLimitSwitch");
      nt_desiredSpeed = table.getEntry("desiredSpeed");
      nt_hasNote = table.getEntry("_hasNote");
      nt_senseNote_prev = table.getEntry("_senseNote_prev");

    }

    public void ntupdate() {
      nt_lightgate.setBoolean(senseNote());
      nt_angleVel.setDouble(getAngleSpeed());
      nt_kP.setDouble(hwAngleVelPID.getP());
      nt_kI.setDouble(hwAngleVelPID.getI());
      nt_kD.setDouble(hwAngleVelPID.getD());
      nt_wheelVel.setDouble(getIntakeRollerSpeed());
      nt_anglePos.setDouble(getAnglePosition());
      nt_angleCmd.setDouble(getAngleSetpoint());
      nt_forwardLimit.setBoolean(atForwardLimitSwitch());
      nt_reverseLimit.setBoolean(atReverseLimitSwitch());
      //nt_reverseLimitSwitchEnabled.setBoolean(limitSwitchEnabled());
      //nt_forwardLimitSwitchEnabled.setBoolean(forwardSwitchEnabled());
      nt_desiredSpeed.setDouble(getDesiredVelocity());
      nt_hasNote.setBoolean(hasNote());
      nt_senseNote_prev.setBoolean(senseNote_prev);
    }
  } // watcher command

}
