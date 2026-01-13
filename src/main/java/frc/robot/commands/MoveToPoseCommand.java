package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

public class MoveToPoseCommand extends Command {
    private final DriveSubsystem drive;
    private final Pose2d target;

    public MoveToPoseCommand(DriveSubsystem drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // reset PID si tu veux
        // drive.resetPID(); // si tu implémentes reset des intégrateurs
    }

    @Override
    public void execute() {
        drive.conduireToPose(target); // appelé à 50Hz
    }

    @Override
    public boolean isFinished() {
        return drive.isAtPose(target);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}