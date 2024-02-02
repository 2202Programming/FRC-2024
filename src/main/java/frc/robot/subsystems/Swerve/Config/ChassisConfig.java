package frc.robot.subsystems.Swerve.Config;

public class ChassisConfig {

    // Kinematics model - wheel offsets from center of robot (0, 0)
    // Left Front given below, symmetry used for others
    public final double XwheelOffset; // meters, half of X wheelbase
    public final double YwheelOffset; // meters, half of Y wheelbase

    public final double wheelCorrectionFactor; // percent
    public final double wheelDiameter; // meters
    public final double kSteeringGR; // [mo-turns to 1 angle wheel turn]
    public final double kDriveGR; // [mo-turn to 1 drive wheel turn]

    public ChassisConfig(double XwheelOffset, double YwheelOffset, double wheelCorrectionFactor, double wheelDiameter,
        double kSteeringGR,
        double kDriveGR) {
      this.XwheelOffset = XwheelOffset;
      this.YwheelOffset = YwheelOffset;
      this.wheelCorrectionFactor = wheelCorrectionFactor;
      this.wheelDiameter = wheelDiameter * wheelCorrectionFactor;
      this.kSteeringGR = kSteeringGR;
      this.kDriveGR = kDriveGR;
    }
  }
