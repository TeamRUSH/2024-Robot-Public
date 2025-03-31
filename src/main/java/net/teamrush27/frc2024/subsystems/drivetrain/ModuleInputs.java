package net.teamrush27.frc2024.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleInputs {

    private final String moduleName;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder cancoder;
    public ModuleInputs(String name, SwerveModule module) {
        this.moduleName = name;
        this.driveMotor = module.getDriveMotor();
        this.steerMotor = module.getSteerMotor();
        this.cancoder = module.getCANcoder();
    }

    public double driveVelocityMps;
    public double driveCurrentAmps;

    public Rotation2d steerPosition = new Rotation2d();
    public double steerClosedLoopError;
    public double steerCurrentAmps;


    public void update() {
        driveVelocityMps = driveMotor.getVelocity().getValue();
        driveCurrentAmps = driveMotor.getStatorCurrent().getValue();
        steerPosition = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue());
        steerClosedLoopError = steerMotor.getClosedLoopError().getValue();
        steerCurrentAmps = steerMotor.getStatorCurrent().getValue();
    }

    public void recordOutputs() {
        
    }
}