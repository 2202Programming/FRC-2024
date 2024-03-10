package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tag_Pose;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.DistanceInterpretor;

public class ContinousAngleTracker extends Command {
    final Transfer transfer;
    final ShooterServo shooter;
    final SwerveDrivetrain drivetrain;

    // Auto angle move based on distance to speaker Tag
    private double distanceToTarget;
    private double targetAngle;
    private DistanceInterpretor distanceInterpretor;
    private Translation2d targetTranslation2d;

    public ContinousAngleTracker() {
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
        calculateTargetAngle();

        if (transfer.hasNote()) {
            shooter.setAngleSetpoint(targetAngle);
        } else if (!transfer.hasNote()) {
            // if no note, shooter needs to be low to allow transfer of loading note
            // placeholder (ideal transfer location between shooter and intake)
            shooter.setAngleSetpoint(ShooterServo.MIN_DEGREES);
        }
    }

    private void calculateTargetAngle() {
        distanceToTarget = drivetrain.getDistanceToTranslation(targetTranslation2d);
        targetAngle = distanceInterpretor.getAngleFromDistance(distanceToTarget);

        SmartDashboard.putNumber("Distance to Target", distanceToTarget);
        SmartDashboard.putNumber("Goal Angle for target", targetAngle);
    }
}