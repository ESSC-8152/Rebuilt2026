// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;


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

	/** Vitesse max utilisée uniquement pour la conduite vers une pose (m/s) */
	private final double kMaxPoseSpeedMetersPerSecond = 0.05; // <- valeur réduite (ajuste si besoin)

	/** Limiteurs de pente pour lisser l'accélération en m/s² */
	private final SlewRateLimiter m_vxLimiter = new SlewRateLimiter(0.01); // 3 m/s^2
	private final SlewRateLimiter m_vyLimiter = new SlewRateLimiter(0.01);

	// Le gyroscope
	private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

	// Initialisation PoseEstimator
	SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			DriveConstants.kDriveKinematics,
			Rotation2d.fromDegrees(getAngle()),
			new SwerveModulePosition[] { avantGauche.getPosition(),
					avantDroite.getPosition(), arriereGauche.getPosition(),
					arriereDroite.getPosition() },
			Pose2d.kZero);

	Field2d field2d = new Field2d();

	// PID controllers pour conduite vers une pose
	// NOTE: Ajuster ces gains pour ton robot
	private final PIDController xController = new PIDController(1.5, 0.0, 0.0); // m/s per m
	private final PIDController yController = new PIDController(1.5, 0.0, 0.0); // m/s per m
	private final PIDController thetaController = new PIDController(3.0, 0.0, 0.0); // deg/s per deg

	// tolérances
	private final double positionToleranceMeters = 0.05; // 5 cm
	private final double angleToleranceDegrees = 3.0; // 3 degrees


	public DriveSubsystem() {
		resetEncoders();
		resetOdometry(new Pose2d());

		// theta controller travaille en degrés pour rester cohérent avec getAngle()
		thetaController.enableContinuousInput(-180.0, 180.0);
		// éventuellement setTolerance si tu veux utiliser atSetpoint()
		xController.setTolerance(positionToleranceMeters);
		yController.setTolerance(positionToleranceMeters);
		thetaController.setTolerance(angleToleranceDegrees);

		// s'assurer que field2d pointe sur l'instance
		field2d = new Field2d();
	}

	@Override
	public void periodic() {
		// Update du Pose Estimator
		poseEstimator.update(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(),
						avantDroite.getPosition(), arriereGauche.getPosition(), arriereDroite.getPosition() });

		SmartDashboard.putNumber("Angle Gyro", getAngle());
		SmartDashboard.putNumber("Pose Estimator X : ", getPose().getX());
		SmartDashboard.putNumber("Pose Estimator Y : ", getPose().getY());
		SmartDashboard.putNumber("Pose Estimator Theta : ", getPose().getRotation().getDegrees());

		setLimelightRobotOrientation();
		addVisionPosition("limelight");

		// Mettre à jour le Field2d
		field2d.setRobotPose(getPose());
		SmartDashboard.putData("Field", field2d);
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

	public Pose2d calculerPositionSouhaite() {
		return new Pose2d(new Translation2d(10.4,3.8), Rotation2d.fromDegrees(175));
	}

	public boolean isAtPose(Pose2d target) {
		Pose2d current = getPose();
		double dx = target.getX() - current.getX();
		double dy = target.getY() - current.getY();
		double distance = Math.hypot(dx, dy);
		double angleError = MathUtil.inputModulus(target.getRotation().getDegrees() - current.getRotation().getDegrees(), -180.0, 180.0);
		return distance < positionToleranceMeters && Math.abs(angleError) < angleToleranceDegrees;
	}
	
	public void conduireToPose(Pose2d poseTarget) {
		Pose2d current = getPose();
	
		// Calcul PID (calculate(measurement, setpoint))
		double rawVx = xController.calculate(current.getX(), poseTarget.getX()); // m/s
		double rawVy = yController.calculate(current.getY(), poseTarget.getY()); // m/s
	
		// Appliquer limite de vitesse spécifique pour la navigation vers une pose
		rawVx = MathUtil.clamp(rawVx, -kMaxPoseSpeedMetersPerSecond, kMaxPoseSpeedMetersPerSecond);
		rawVy = MathUtil.clamp(rawVy, -kMaxPoseSpeedMetersPerSecond, kMaxPoseSpeedMetersPerSecond);
	
		// Appliquer SlewRateLimiter pour lisser l'accélération
		double vxField = m_vxLimiter.calculate(rawVx);
		double vyField = m_vyLimiter.calculate(rawVy);
	
		// Contrôle angulaire (en degrés via thetaController), converti en rad/s
		double currentDeg = current.getRotation().getDegrees();
		double targetDeg = poseTarget.getRotation().getDegrees();
		double omegaDegPerSec = thetaController.calculate(currentDeg, targetDeg);
		double omegaRadPerSec = Units.degreesToRadians(MathUtil.clamp(omegaDegPerSec, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed));
	
		// Inversion selon l'alliance si nécessaire (comme ton code existant)
		double invert = isRedAlliance() ? -1.0 : 1.0;
		vxField *= invert;
		vyField *= invert;
	
		// Création des vitesses chassis (field-relative)
		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, omegaRadPerSec, current.getRotation());
	
		// Envoi aux modules
		conduireChassis(speeds);
	
		// Arrêt quand on est dans la tolérance
		boolean positionOK = Math.hypot(poseTarget.getX() - current.getX(), poseTarget.getY() - current.getY()) < positionToleranceMeters;
		boolean angleOK = Math.abs(MathUtil.inputModulus(targetDeg - currentDeg, -180.0, 180.0)) < angleToleranceDegrees;
		if (positionOK && angleOK) {
			stop();
			xController.reset();
			yController.reset();
			thetaController.reset();
			m_vxLimiter.reset(0.0);
			m_vyLimiter.reset(0.0);
		}
	
		// Pour debug: afficher les commandes sur SmartDashboard
		SmartDashboard.putNumber("conduireToPose_vxField", vxField);
		SmartDashboard.putNumber("conduireToPose_vyField", vyField);
		SmartDashboard.putNumber("conduireToPose_omegaRad", omegaRadPerSec);
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
        
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
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

	public void resetPose(Pose2d pose) {
		resetOdometry(pose);
	}

	public void resetOdometry(Pose2d pose) {// pose est à la pose où reset, c'est typiquement l'origine du terrain
		poseEstimator.resetPosition(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(),
						avantDroite.getPosition(), arriereGauche.getPosition(), arriereDroite.getPosition() },
				pose);
	}

	public void setLimelightRobotOrientation() {
		LimelightHelpers.SetRobotOrientation(
				"limelight",
				poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
				0,
				0,
				0,
				0,
				0);
	}

	public void addVisionPosition(String nomComplet) {

		// parametre limelight
		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
				0.7,
				0.7,
				9999999));

		LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
				nomComplet);
		boolean doRejectUpdate = false;
		if (poseEstimate == null) {
			return;
		}

		if (Math.abs(getRate()) > 720) {
			doRejectUpdate = true;
		}
		if (poseEstimate.tagCount == 0) {
			doRejectUpdate = true;
		}
		SmartDashboard.putBoolean(nomComplet, !doRejectUpdate);
		if (!doRejectUpdate) {
			poseEstimator.addVisionMeasurement(
					poseEstimate.pose,
					poseEstimate.timestampSeconds);
		}
	}

    public void setZeroPostion() {
        resetOdometry(new Pose2d());
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
		return -m_gyro.getYaw();
	}

	public double getRate() {
		return m_gyro.getRate();
	}

	public void resetGyro() {
		m_gyro.reset();
	}

	/// ///////////// Path Planner
	public ChassisSpeeds getRobotRelativeSpeeds() {
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