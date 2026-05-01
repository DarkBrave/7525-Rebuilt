package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

public class ClimberIOReal implements ClimberIO {

	protected TalonFX leftMotor;
	protected PIDController climbPID;
	protected double motorSpeed;
	private boolean manualMode = false;

	public ClimberIOReal() {
		leftMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID);
		leftMotor.setPosition(0);
		leftMotor.setNeutralMode(NeutralModeValue.Brake);
		climbPID = ClimberConstants.CLIMB_PID.get();

		motorSpeed = ClimberConstants.IDLE_SETPOINT;
	}

	@Override
	public void logOutputs(ClimberIOOutputs outputs) {
		outputs.leftPosition = leftMotor.getPosition().getValue();
		outputs.speed = motorSpeed;
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/Current", leftMotor.getSupplyCurrent().getValue());
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/CurrentStator", leftMotor.getStatorCurrent().getValue());
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/LeftPositionRot", outputs.leftPosition.in(Rotations));
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/SetpointRot", outputs.speed);
	}

	public void setSetpoint(double setpoint) {
		motorSpeed = setpoint;
		if (OPERATOR_CONTROLLER.getPOV() == 0) {
			manualMode = true;
			leftMotor.set(0.75);
		} else if (OPERATOR_CONTROLLER.getPOV() == 180) {
			manualMode = true;
			leftMotor.set(-0.75);
		} else if (!manualMode) {
			leftMotor.setVoltage(climbPID.calculate(leftMotor.getPosition().getValue().in(Rotations), setpoint));
		} else {
			leftMotor.set(0);
		}

		if (OPERATOR_CONTROLLER.getBackButtonPressed()) {
			manualMode = false;
		}
	}
}
