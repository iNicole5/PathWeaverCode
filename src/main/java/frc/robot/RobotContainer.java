package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.driveTrain.Drivetrain;

public class RobotContainer {
    private Drivetrain drive = new Drivetrain();

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(
            Units.feetToMeters(2.0), Units.feetToMeters(2.0));
        config.setKinematics(drive.getKinematics());
    
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()),
                new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
            config
        );
    
        RamseteCommand command = new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(kRamseteB, kRamseteB),
            new SimpleMotorFeedforward(Drivetrain.ks, Drivetrain.kv, Drivetrain.ka),
            drive.getKinematics(),
            drive::getSpeeds,
            drive.getLeftPIDController(),
            drive.getRightPIDController(),
            drive::setOutputVolts,
                (Subsystem) drive
        );
    
        return command.andThen(() -> drive.setOutputVolts(0, 0));
      }
    
      public void reset() {
        drive.reset();
      }

}
