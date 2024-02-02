package frc.robot.subsystems.Swerve.Config;

 // CAN IDs for swerve drivetrain

  public class CANConfig {
    // drive train CANCoders
    public final int DT_BL_CANCODER;
    public final int DT_BR_CANCODER;
    public final int DT_FR_CANCODER;
    public final int DT_FL_CANCODER;

    // drive train drive / angle motors - sparkmax neo
    public final int DT_FL_DRIVE;
    public final int DT_FL_ANGLE;
    public final int DT_BL_DRIVE;
    public final int DT_BL_ANGLE;
    public final int DT_BR_DRIVE;
    public final int DT_BR_ANGLE;
    public final int DT_FR_DRIVE;
    public final int DT_FR_ANGLE;

   /**
   * CANCONFIG - BLC = back left cancoder.  FLD = Front Left drive motor.  FLA = Front Left angle motor.
   * 
   */
    public CANConfig(int BLC, int BRC, int FRC, int FLC, int FLD, int FLA, int BLD, int BLA, int BRD, int BRA, int FRD, int FRA) {
      this.DT_BL_CANCODER = BLC;
      this.DT_BR_CANCODER = BRC;
      this.DT_FL_CANCODER = FLC;
      this.DT_FR_CANCODER = FRC;

      this.DT_FL_DRIVE = FLD;
      this.DT_FR_DRIVE = FRD;
      this.DT_BL_DRIVE = BLD;
      this.DT_BR_DRIVE = BRD;

      this.DT_FL_ANGLE = FLA;
      this.DT_BL_ANGLE = BLA;
      this.DT_FR_ANGLE = FRA;
      this.DT_BR_ANGLE = BRA;
    }
  }