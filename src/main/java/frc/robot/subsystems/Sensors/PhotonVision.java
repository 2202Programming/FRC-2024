// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Sensors;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class PhotonVision extends SubsystemBase {

  // Cameras we have
  String camera1 = "Microsoft_LifeCam_HD-3000";
  String camera2 = "Arducam_OV9281_USB_Camera";

  // private PhotonCamera camera_global;
  private PhotonCamera camera_microsoft;
  private PhotonCamera camera_arducam;
  private boolean hasAprilTargets;
  private boolean hasTapeTargets;
  private List<PhotonTrackedTarget> AprilTargets;
  private List<PhotonTrackedTarget> TapeTargets;
  private PhotonTrackedTarget bestTarget; // was hiding access bug = new PhotonTrackedTarget();
  private double yaw;
  private double pitch;
  private double area;
  private double skew;
  private Transform3d targetPose;
  private List<TargetCorner> corners;
  private Transform3d robotToCam;
  private int targetID;
  // private double poseAmbiguity;
  // private Transform3d bestCameraToTarget;
  // private Transform3d alternateCameraToTarget;
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator robotPoseEstimator;
  private Pair<Pose2d, Double> currentPoseEstimate;
  private SwerveDrivetrain sdt = null;
  private Pair<Pose2d, Double> previousPoseEstimate;

  private NetworkTable table;
  private NetworkTableEntry nt_tapeTargets;
  private NetworkTableEntry nt_aprilTargets;
  private NetworkTableEntry nt_PVPoseX;
  private NetworkTableEntry nt_PVPoseY;
  private NetworkTableEntry nt_PVTapeYaw;
  private NetworkTableEntry nt_PVTapeYaw2;

  public final String NT_Name = "Vision"; // expose data under Vision table
  
  public PhotonVision() {
    // build path to apriltag json file in deploy directory
    File deploy = Filesystem.getDeployDirectory();
    String path = deploy.getPath() + "/aprilTags.json";

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nt_tapeTargets = table.getEntry("/PV Num Tape Targets");
    nt_aprilTargets = table.getEntry("/PV Num April Targets");
    nt_PVPoseX = table.getEntry("/PV Pose X");
    nt_PVPoseY = table.getEntry("/PV Pose Y");
    nt_PVTapeYaw = table.getEntry("/PV Tape Yaw");
    nt_PVTapeYaw2 = table.getEntry("/PV Tape Yaw Two");
    // load apriltag field layout
    try {
      fieldLayout = new AprilTagFieldLayout(path);
    } catch (Exception e) {
      System.out.println("***FAILED TO LOAD APRILTAG LIST***");
    }

    // Assemble the list of cameras & mount locations
    camera_microsoft = new PhotonCamera(camera1);
    camera_arducam = new PhotonCamera(camera2);
    robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.75), new Rotation3d(0, 0, 0)); // Cam mounted facing
                                                                                              // forward
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera_arducam, robotToCam));

    // setup PhotonVision's pose estimator,
    robotPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, camera_arducam, robotToCam);
    previousPoseEstimate = new Pair<>(new Pose2d(), 0.0);
    currentPoseEstimate = new Pair<>(new Pose2d(), 0.0);

  }

  public void setDrivetrain(SwerveDrivetrain sdt)
  {
    this.sdt = sdt;
  }

  /* Used to set the starting postions either at Auto, or anytime the pose estimator needs a kick. */
  public void setInitialPose(Pair<Pose2d, Double> pose) {
    currentPoseEstimate = pose;
  }

  @Override
  public void periodic() {
    // Query the latest Apriltag result from PhotonVision
    var result_global = camera_arducam.getLatestResult();

    // Check if the latest result has any targets.
    hasAprilTargets = result_global.hasTargets();
    nt_aprilTargets.setInteger(result_global.getTargets().size());

    if (hasAprilTargets) {
      // Get a list of currently tracked targets.
      AprilTargets = result_global.getTargets();
      bestTarget = result_global.getBestTarget();

      // Get information from target.
      yaw = bestTarget.getYaw();
      pitch = bestTarget.getPitch();
      area = bestTarget.getArea();
      skew = bestTarget.getSkew();
      targetPose = bestTarget.getBestCameraToTarget();
      corners = bestTarget.getDetectedCorners();

      // Get information from target.
      targetID = bestTarget.getFiducialId();


      // only targets we care about, sometime we see number outside expected range
      if (targetID < 9) {
        Pair<Pose2d, Double> previousPoseEstimateHolder = previousPoseEstimate;
        previousPoseEstimate = currentPoseEstimate;

        // guard, we can't estimate without a sdt
        if (sdt == null) return;

        //currentPoseEstimate = getEstimatedGlobalPose(sdt.getPose());
        currentPoseEstimate = getEstimatedGlobalPose(previousPoseEstimate.getFirst());

        //don't update dash if there isn't a pose, photonVision return null if it doesn't have an estimate
        if (currentPoseEstimate.getFirst() != null) {
          nt_PVPoseX.setDouble(currentPoseEstimate.getFirst().getX());
          nt_PVPoseY.setDouble(currentPoseEstimate.getFirst().getY());
        }
        else {
          previousPoseEstimate = previousPoseEstimateHolder;
        }
          
      }
    }

    // Query the latest Retroreflective result from PhotonVision
    var result_microsoft = camera_microsoft.getLatestResult();

    // Check if the latest result has any targets.
    hasTapeTargets = result_microsoft.hasTargets();

    if (hasTapeTargets) {
      // Get a list of currently tracked targets.
      TapeTargets = result_microsoft.getTargets();
      // Get the current best target.
      bestTarget = result_microsoft.getBestTarget();

      // Get information from target.
      nt_tapeTargets.setInteger(TapeTargets.size());
      nt_PVTapeYaw.setDouble(TapeTargets.get(0).getYaw());

      SmartDashboard.putNumber("PV Area #1", TapeTargets.get(0).getArea());
      if (getNumberOfTapeTargets() > 1) {
        nt_PVTapeYaw2.setDouble(TapeTargets.get(1).getYaw());
        SmartDashboard.putNumber("PV Yaw #2", TapeTargets.get(1).getYaw());
        SmartDashboard.putNumber("PV Area #2", TapeTargets.get(1).getArea());
        SmartDashboard.putNumber("PV Largest Yaw", getLargestTapeTarget().getYaw());
        SmartDashboard.putNumber("PV 2nd Largest Yaw", getSecondLargestTapeTarget().getYaw());
      }

    }
  }

  public double getVisionTimestamp(){
    return currentPoseEstimate.getSecond();
  }

  public boolean hasAprilTarget() {
    return hasAprilTargets;
  }

  public List<PhotonTrackedTarget> getAprilTargets() {
    return AprilTargets;
  }

  public Transform3d getTarget3dPose() {
    return targetPose;
  }

  public List<TargetCorner> getCorners() {
    return corners;
  }

  // Pair<> getPoseEstimate() used by Swerve to add vision estimate
  public Pair<Pose2d, Double> getPoseEstimate() {
    Pose2d curr = currentPoseEstimate.getFirst();
    Pose2d curr_copy = new Pose2d(curr.getTranslation(), curr.getRotation());
    return new Pair<>(curr_copy, currentPoseEstimate.getSecond());
  }

  int getTargetID() {
    return targetID;
  }

  public boolean hasTapeTarget() {
    return hasTapeTargets;
  }

  public int getNumberOfTapeTargets() {
    return TapeTargets.size();
  }

  public PhotonTrackedTarget getLargestTapeTarget() {
    TapeTargets.sort(new PhotonTrackedTargetComparator());
    return TapeTargets.get(0);
  }

  public PhotonTrackedTarget getSecondLargestTapeTarget() {
    TapeTargets.sort(new PhotonTrackedTargetComparator());
    return TapeTargets.get(1);
  }

  public double getYaw() {
    return yaw;
  }

  public double getPitch() {
    return pitch;
  }

  public double getArea() {
    return area;
  }

  public double getSkew() {
    return skew;
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   *         NOTE - APRIL TAG NEEDS TO BE IN 3D mode
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<EstimatedRobotPose> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      return new Pair<Pose2d, Double>(result.get().estimatedPose.toPose2d(),
          currentTime - result.get().timestampSeconds);
    } else {
      return new Pair<Pose2d, Double>(sdt.getPose(), 0.0);
    }

  }

  // make a comparator class to allow for list sorting
  class PhotonTrackedTargetComparator implements Comparator<PhotonTrackedTarget> {
    @Override
    public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b) {
      return (a.getArea() > b.getArea()) ? 1 : -1;
    }
  }
}