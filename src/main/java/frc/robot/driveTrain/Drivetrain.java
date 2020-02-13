/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.driveTrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {

    private static final double kGearRatio = 7.29;
    private static final double kWheelRadiusInches = 3.0;
    
    TalonFX leftMaster = new TalonFX(3);
    TalonFX rightMaster = new TalonFX(1);

    TalonFX leftSlave = new TalonFX(2);
    TalonFX rightSlave = new TalonFX(0);

    PigeonIMU gyro = new PigeonIMU(0);  //Change to different gyro

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

    PIDController leftPIDController = new PIDController(2.95, 0, 0);
    PIDController rightPIDController = new PIDController(2.95, 0, 0);

    Pose2d pose = new Pose2d();
    
public Drivetrain() {

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster); 

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    gyro.setYaw(0);
}

public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getFusedHeading());
}

public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getSelectedSensorVelocity()/ kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
        rightMaster.getSelectedSensorVelocity()/ kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(ControlMode.PercentOutput, leftVolts / 12);
    rightMaster.set(ControlMode.PercentOutput, rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    //pose = odometry.update(getHeading(), getSpeeds());
    //pose = odometry.update(getHeading(), getLeftEncoder(), getRightEncoder());
    pose = odometry.update(getHeading(), getSpeeds());
  }

}
