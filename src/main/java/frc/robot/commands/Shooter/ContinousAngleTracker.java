package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class ContinousAngleTracker extends Command {
    final Transfer transfer;
    final ShooterServo shooter;
    final SwerveDrivetrain drivetrain;

    // Auto angle move based on distance to speaker Tag
    double targetDistance;
    double targetAngle;
    double targetRPM;

    boolean dont_move;

    DistanceInterpretor distanceInterpretor;


    public ContinousAngleTracker(boolean dont_move) {
        this.dont_move = dont_move;
        shooter = RobotContainer.getSubsystem(ShooterServo.class);
        transfer = RobotContainer.getSubsystem(Transfer.class);
        drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        distanceInterpretor = DistanceInterpretor.getSingleton();
        // don't add requirements, only tracking angle.
    }

    @Override
    public void initialize() {
       distanceInterpretor.setTarget();
    }

    @Override
    public void execute() {
        super.execute();  // calls calculate, does network table work.

        // for testing, this command can run, NT will update, but won't move shooter
        if (dont_move)
            return;

        if (transfer.hasNote()) {
            shooter.setAngleSetpoint(targetAngle);
        } else if (!transfer.hasNote()) {
            // if no note, shooter needs to be low to allow transfer of loading note
            // placeholder (ideal transfer location between shooter and intake)
            shooter.setAngleSetpoint(ShooterServo.MIN_DEGREES);
        }
    }

}
