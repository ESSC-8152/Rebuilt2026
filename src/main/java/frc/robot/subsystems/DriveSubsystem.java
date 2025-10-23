// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

	// Créer les moteurs swerves
	private MAXSwerveModule avantGauche = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

	private MAXSwerveModule avantDroite = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

	private MAXSwerveModule arriereGauche = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

	private MAXSwerveModule arriereDroite = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

	// Le gyroscope
	// private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

	// Initialisation PoseEstimator
	SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			DriveConstants.kDriveKinematics,
			Rotation2d.fromDegrees(getAngle()),
			new SwerveModulePosition[] { avantGauche.getPosition(),
					avantDroite.getPosition(), arriereGauche.getPosition(),
					arriereDroite.getPosition() },
			Pose2d.kZero);

	Field2d field2d = new Field2d();


	public DriveSubsystem() {

		// // Reset initial
		// resetGyro();
		resetEncoders();
		resetOdometry(new Pose2d());
	}

	@Override
	public void periodic() {
		// Update du Pose Estimator
		poseEstimator.update(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(),
						avantDroite.getPosition(), arriereGauche.getPosition(), arriereDroite.getPosition() });

		//SmartDashboard.putBoolean("redalliance", isRedAlliance());

		SmartDashboard.putNumber("Angle Gyro", getAngle());

		SmartDashboard.putNumber("Pose Estimator X : ", getPose().getX());
		SmartDashboard.putNumber("Pose Estimator Y : ", getPose().getY());
		SmartDashboard.putNumber(
				"Pose Estimator Theta : ",
				getPose().getRotation().getDegrees());
	}

	/// ////// MÉTHODE DONNANT DES CONSIGNES À CHAQUE MODULE

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		//La librairie de REV utilise la fonction .desaturate ici.
		//Attention, ils utilisent le maxChassisSpeed au lieu du maxVitesseModule
		//SetPointGenerator ôte la nécessiter de désaturer
		avantGauche.setDesiredState(desiredStates[0]);
		avantDroite.setDesiredState(desiredStates[1]);
		arriereGauche.setDesiredState(desiredStates[2]);
		arriereDroite.setDesiredState(desiredStates[3]);
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] { avantGauche.getState(),
				avantDroite.getState(), arriereGauche.getState(),
				arriereDroite.getState() };
	}
	

	public void conduire(
			double xSpeed,
			double ySpeed,
			double rot,
			boolean fieldRelative,
			boolean squared) {

		double deadband = 0.05;
		// appliquer une deadband sur les joysticks et corriger la direction
		xSpeed = -MathUtil.applyDeadband(xSpeed, deadband);
		ySpeed = -MathUtil.applyDeadband(ySpeed, deadband);
		rot = -MathUtil.applyDeadband(rot, deadband);

		if (squared) {// Mettre les joysticks "au carré" pour adoucir les
			// déplacements
			xSpeed = xSpeed * Math.abs(xSpeed);
			ySpeed = ySpeed * Math.abs(ySpeed);
			rot = rot * Math.abs(rot);
		}

		// Convert the commanded speeds into the correct units for the
		// drivetrain
		double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

		// inversion du field oriented selon l'alliance
		double invert = -1;

		//Ajuster pour field relative
		//L'inversion selon l'alliance seulement nécessaire en x et y en field oriented
		//Façon vraiment plus clean de gérer ça qu'en 2024
		ChassisSpeeds speeds = fieldRelative ? 
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered * invert, ySpeedDelivered * invert,
				rotDelivered, getPose().getRotation())
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
	}

	public void stop() {
		conduire(0, 0, 0, false, false);

	}

	// Sets the wheels into an X formation to prevent movement.
	public void setX() {
		avantGauche.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						45)));
		avantDroite.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						-45)));
		arriereGauche.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						-45)));
		arriereDroite.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						45)));
	}

	/// ////// Pose estimator
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {// pose est à la pose où reset, c'est typiquement l'origine du terrain
		poseEstimator.resetPosition(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(),
						avantDroite.getPosition(), arriereGauche.getPosition(), arriereDroite.getPosition() },
				pose);
	}

	////////////// Encodeurs
	// Pas besoin de méthode pour obtenir la position des encodeurs, tout ça
	// passe directement par la pose2D du robot
	public void resetEncoders() {
		avantGauche.resetEncoders();
		arriereGauche.resetEncoders();
		avantDroite.resetEncoders();
		arriereDroite.resetEncoders();
	}

	/////////////// GYRO
	public double getAngle() {
		// return -m_gyro.getYaw();
        return 0;
	}

	// public double getRate() {
	// 	return m_gyro.getRate();
	// }

	// public void resetGyro() {
	// 	m_gyro.reset();
	// }

	/// ///////////// Path Planner
	public ChassisSpeeds getChassisSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(
				avantDroite.getState(),
				avantGauche.getState(),
				arriereDroite.getState(),
				arriereGauche.getState());
	}

	public void conduireChassis(ChassisSpeeds chassisSpeeds) {
		// Ramene la vitesse en intervale de 20 ms
		ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(
				chassisSpeeds,
				0.02);

		SwerveModuleState[] swerveModuleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				targetSpeed);
		setModuleStates(swerveModuleState);
	}

	//Vérifier l'alliance. Il faut le caller en tout temps car l'alliance est initialiser après le boot du robot
	public boolean isRedAlliance() {
		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent()) {
			return ally.get() == Alliance.Red;

		} else {
			return false;
		}
	}

}