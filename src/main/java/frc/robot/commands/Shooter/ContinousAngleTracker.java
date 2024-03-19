package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Tag_Pose;
import frc.robot.RobotContainer;
import frc.robot.commands.utility.TargetWatcherCmd;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.DistanceInterpretor;

public class ContinousAngleTracker extends TargetWatcherCmd {
    final Transfer transfer;
    final ShooterServo shooter;
    final SwerveDrivetrain drivetrain;

    // Auto angle move based on distance to speaker Tag
    double targetDistance;
    double targetAngle;
    double targetRPM;

    boolean dont_move;

    private DistanceInterpretor distanceInterpretor;
    private Translation2d targetTranslation2d;

    public ContinousAngleTracker(boolean dont_move) {
        this.dont_move = dont_move;
        shooter = RobotContainer.getSubsystem(ShooterServo.class);
        transfer = RobotContainer.getSubsystem(Transfer.class);
        drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        distanceInterpretor = new DistanceInterpretor();
        // don't add requirements, only tracking angle.
    }

    @Override
    public void initialize() {
        // deal with Optional<> , can be null when simulating without driverstation. 
        var optAlliance = DriverStation.getAlliance();
        var alliance = optAlliance.isPresent() ? optAlliance.get() : DriverStation.Alliance.Blue;
        targetTranslation2d = (alliance == DriverStation.Alliance.Blue) ? Tag_Pose.ID7 : // Blue Alliance
                Tag_Pose.ID4; // Red Alliance

        if (!optAlliance.isPresent())
            System.out.println("Warning: Defaulting to Blue Alliance in ContinousAngleTracker cmd.");
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

    // Implementations for TargetWatcherCmd
    public double getTargetAngle(){
        return targetAngle;
    }

    public double getTargetRPM(){
        return targetRPM;
    }
    public double getTargetDistance(){
        return targetDistance;
    }

    @Override
    public void calculate() {
        targetDistance = drivetrain.getDistanceToTranslation(targetTranslation2d);
        targetAngle = distanceInterpretor.getAngleFromDistance(targetDistance);
        targetRPM = 3000.0;  //TODO calc rpm
    }
}
