package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.config.SwerveConfig;

public class SwerveModule {
    private final TalonSRX driveMotor;
    private final VictorSPX steerMotor;

    private final DutyCycleEncoder steerEncoder;

    private final PIDController steerPID = new PIDController(SwerveConfig.STEER_P, SwerveConfig.STEER_I, SwerveConfig.STEER_D);

    public SwerveModule(int driveCANId, int steerCANId, int encoderDIOPort) {
        driveMotor = new TalonSRX(driveCANId);
        steerMotor = new VictorSPX(steerCANId);
        
        steerEncoder = new DutyCycleEncoder(encoderDIOPort);

        steerPID.enableContinuousInput(0, 2 * Math.PI);
    }

    public Rotation2d getAngle() {
        double radians = steerEncoder.get() * 2.0 * Math.PI;
        return new Rotation2d(radians);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle
        );

        optimizedState.optimize(getAngle());

        driveMotor.set(ControlMode.PercentOutput, optimizedState.speedMetersPerSecond / 4.5);
        
        double currentRadians = getAngle().getRadians();
        double targetRadians = optimizedState.angle.getRadians();

        double steerOutput = steerPID.calculate(currentRadians, targetRadians);

        steerMotor.set(ControlMode.PercentOutput, steerOutput);
    }
}
