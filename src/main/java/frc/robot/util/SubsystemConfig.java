package frc.robot.util;

public class SubsystemConfig {

    // Support for multiple robots on same code base
    public final boolean HAS_INTAKE;
    public final boolean HAS_SHOOTER;
    public final boolean HAS_ANALOG_PNEUMATICS;
    public final boolean IS_COMPETITION_BOT;
    public final boolean HAS_MAGAZINE;
    public final boolean HAS_CLIMBER;
    public final boolean HAS_POSITIONER;
    public final boolean HAS_DRIVETRAIN;
    public final boolean HAS_LIMELIGHT;

    public SubsystemConfig(boolean HAS_INTAKE, boolean HAS_SHOOTER, boolean HAS_ANALOG_PNEUMATICS,
            boolean IS_COMPETITION_BOT, boolean HAS_MAGAZINE,
            boolean HAS_CLIMBER,
            boolean HAS_POSITIONER, boolean HAS_DRIVETRAIN, boolean HAS_LIMELIGHT) {
        this.HAS_INTAKE = HAS_INTAKE;
        this.HAS_SHOOTER = HAS_SHOOTER;
        this.HAS_ANALOG_PNEUMATICS = HAS_ANALOG_PNEUMATICS;
        this.IS_COMPETITION_BOT = IS_COMPETITION_BOT;
        this.HAS_MAGAZINE = HAS_MAGAZINE;
        this.HAS_CLIMBER = HAS_CLIMBER;
        this.HAS_POSITIONER = HAS_POSITIONER;
        this.HAS_DRIVETRAIN = HAS_DRIVETRAIN;
        this.HAS_LIMELIGHT = HAS_LIMELIGHT;
    }
}
