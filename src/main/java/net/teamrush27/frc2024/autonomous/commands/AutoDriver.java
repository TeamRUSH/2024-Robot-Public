package net.teamrush27.frc2024.autonomous.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.teamrush27.frc2024.util.AprilTag2024Field;

public class AutoDriver {

  private final AprilTag2024Field field;

  public AutoDriver() {
    field = AprilTag2024Field.getInstance();
  }

  public static PathConstraints quickConstraints = new PathConstraints(
      2.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

  public static PathConstraints approachConstraints = new PathConstraints(
      1.0, 1.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

  public Command driveToPose(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(
        targetPose,
        quickConstraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
    // before attempting to rotate.
    );
  }

  public Command driveToPose(Pose2d targetPose, Transform2d transform2d) {
    return AutoBuilder.pathfindToPose(
        targetPose.transformBy(transform2d),
        quickConstraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
    // before attempting to rotate.
    );
  }

  public SequentialCommandGroup alignAndDriveToPose(Pose2d targetPose, Transform2d transform2d) {
    Pose2d approachPose = targetPose.transformBy(transform2d);
    return new SequentialCommandGroup(AutoBuilder.pathfindToPose(
        approachPose,
        quickConstraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
    // before attempting to rotate.
    ),
        AutoBuilder.pathfindToPose(
            targetPose,
            approachConstraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel
        // before attempting to rotate.
        ));
  }

  public Command pathfindThenFollow(PathPlannerPath plannerPath) {
    return AutoBuilder.pathfindThenFollowPath(
        plannerPath,
        quickConstraints,
        0.0);
  }

  private PathPlannerPath createPath(int tagId) {
    Pose2d element = field.getTagPose3d(tagId).toPose2d();
    List<Translation2d> approachElementPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(element.getX(), element.getY() - Units.inchesToMeters(24), Rotation2d.fromDegrees(0)),
        new Pose2d(element.getX(), element.getY() - Units.inchesToMeters(18), Rotation2d.fromDegrees(0)),
        new Pose2d(element.getX(), element.getY() - Units.inchesToMeters(12), Rotation2d.fromDegrees(0)));

    /*
     * TODO add a transform to the AT pose to line up relative to an AT, not
     * relative to field (stage, source ATs are angled)
     * List<Translation2d> approachElementPoints = PathPlannerPath.bezierFromPoses(
     * element.plus(new Transform2d(0, Units.inchesToMeters(-24), new
     * Rotation2d())),
     * element.plus(new Transform2d(0, Units.inchesToMeters(-18), new
     * Rotation2d())),
     * element.plus(new Transform2d(0, Units.inchesToMeters(-12), new Rotation2d()))
     * );
     */
    PathPlannerPath path = new PathPlannerPath(
        approachElementPoints,
        approachConstraints,
        new GoalEndState(0, Rotation2d.fromDegrees(-90)) // TODO Make rotation based on AT
    );
    path.preventFlipping = true;
    return path;
  }

  public Command driveToAmp(Optional<Alliance> alliance) {
    if(alliance.isEmpty()) {
      return Commands.print("Driver Station alliance color is empty! Failed to drive to amp!");
    }
    return switch (alliance.get()) {
      case Blue -> pathfindThenFollow(createPath(6));
      case Red -> pathfindThenFollow(createPath(5));
    };
  }

  // public Command driveToAmp() {
  // Transform2d transform2d = new Transform2d(0, Units.inchesToMeters(36),
  // Rotation2d.fromDegrees(0));
  // return switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
  // case Blue -> alignAndDriveToPose(field.getTagPose3d(6).toPose2d(),
  // transform2d);
  // case Red -> alignAndDriveToPose(field.getTagPose3d(5).toPose2d(),
  // transform2d);
  // };
  // }

}