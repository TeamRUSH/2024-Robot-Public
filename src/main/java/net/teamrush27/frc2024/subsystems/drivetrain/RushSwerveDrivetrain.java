package net.teamrush27.frc2024.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class RushSwerveDrivetrain extends SwerveDrivetrain {
    public RushSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    public void setFieldRelativeOffset() {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = new Rotation2d();
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return m_kinematics;
    }
}
