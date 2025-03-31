package net.teamrush27.frc2024.subsystems.climber;
import com.revrobotics.CANSparkBase.ControlType;

public enum ClimberSystemState {
    HOMING(-2.3, ControlType.kVoltage, ClimberConfig.SERVO_UNLATCH_POSITION) {
        @Override
        public ClimberSystemState handle(ClimberInputs inputs, ClimberOutputs outputs, ClimberWantedState wantedState) {
            if (inputs.medianMotorCurrent > ClimberConfig.CLIMBER_HOME_CURRENT) {

                return HOME;
            }
            return HOMING;
        }
    },

    HOME(0, ControlType.kVoltage, ClimberConfig.SERVO_UNLATCH_POSITION) {

        @Override
        public ClimberSystemState handle(ClimberInputs inputs, ClimberOutputs outputs, ClimberWantedState wantedState) {

            return switch (wantedState) {
                case IDLE, CLIMB -> HOME;
                case DEPLOY -> DEPLOYING;
            };
        }
    },

    DEPLOYING(ClimberConfig.CLIMBER_DEPLOY_POSITION, ControlType.kPosition, ClimberConfig.SERVO_UNLATCH_POSITION) {

        @Override
        public ClimberSystemState handle(ClimberInputs inputs, ClimberOutputs outputs, ClimberWantedState wantedState) {

            return checkPositionInTolerance(inputs.motorPosition, outputs.motorSetpoint) ? DEPLOYED : DEPLOYING;
        }
    },

    DEPLOYED(ClimberConfig.SERVO_LATCH_POSITION) {
        @Override
        public ClimberSystemState handle(ClimberInputs inputs, ClimberOutputs outputs, ClimberWantedState wantedState) {

            return switch (wantedState) {
                case IDLE, DEPLOY -> DEPLOYED;
                case CLIMB -> CLIMB;
            };
        }
    },

    CLIMB(ClimberConfig.CLIMBER_RETRACT1_POSITION, ControlType.kPosition, ClimberConfig.SERVO_LATCH_POSITION) {
        @Override
        public ClimberSystemState handle(ClimberInputs inputs, ClimberOutputs outputs, ClimberWantedState wantedState) {

            return switch (wantedState) {
                case CLIMB -> inputs.bottomSensor ? DEPLOYED : CLIMB;
                case DEPLOY, IDLE -> DEPLOYED;
            };
        }
    };

    private double servoPosition;
    private double motorSetpoint;
    private ControlType controlType;

    ClimberSystemState(double motorSetpoint, ControlType controlType, double servoPosition) {
        this.servoPosition = servoPosition;
        this.motorSetpoint = motorSetpoint;
        this.controlType = controlType;
    }

    ClimberSystemState(double servoPosition) {
        this.servoPosition = servoPosition;
        this.motorSetpoint = 0;
        this.controlType = ControlType.kDutyCycle;
    }

    public double getMotorSetpoint() {
        return motorSetpoint;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public ControlType getControlType() {
        return controlType;
    }

    public abstract ClimberSystemState handle(ClimberInputs inputs, ClimberOutputs outputs,
            ClimberWantedState wantedState);

    public static boolean checkPositionInTolerance(double currentPosition, double targetPosition) {
        return Math.abs(currentPosition - targetPosition) <= ClimberConfig.CLIMB_POSITION_TOLERANCE;
    }
}
