package net.teamrush27.frc2024.subsystems.intake;

public enum IntakeSystemState {
    INIT() {
        @Override
        IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs){
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_IDLE;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_STOW;
            return switch (intakeWantedState) {
                case IDLE -> INIT;
                case INTAKE -> DEPLOY;
                case EXHAUST -> EXHAUST;
                case CLIMB -> CLIMB;
            };
        }
    }, IDLE() {

        @Override
        IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs) {
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_IDLE;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_STOW;
            return switch (intakeWantedState) {
                case IDLE -> inPosition(intakeInputs.filteredLampreyPosition, IntakeConfig.INTAKE_POS_ERROR_TOLERANCE_DEG) ? IDLE : RETRACT;
                case INTAKE -> DEPLOY;
                case EXHAUST -> inPosition(intakeInputs.filteredLampreyPosition, IntakeConfig.INTAKE_POS_ERROR_TOLERANCE_DEG) ? EXHAUST : RETRACT;
                case CLIMB -> CLIMB;
            };
        }
    }, DEPLOY() {
        @Override
        IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs) {
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_DEPLOYED;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_OUT;

            return switch (intakeWantedState) {
                case IDLE, EXHAUST -> RETRACT;
                case INTAKE -> inPosition(intakeInputs.filteredLampreyPosition,  IntakeConfig.INTAKE_POS_ERROR_TOLERANCE_DEG) ? INTAKE : DEPLOY;
                case CLIMB -> CLIMB;
            };
        }
    }, INTAKE() {
        @Override
        IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs) {
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_DEPLOYED;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_OUT;

            return switch (intakeWantedState) {
                case IDLE, EXHAUST -> RETRACT;
                case INTAKE -> INTAKE;
                case CLIMB -> CLIMB;
            };
        }
    }, RETRACT() {
        @Override
        IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs) {
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_IDLE;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_STOW;

            return switch (intakeWantedState) {
                case IDLE -> inPosition(intakeInputs.filteredLampreyPosition, IntakeConfig.INTAKE_POS_ERROR_TOLERANCE_DEG) ? IDLE : RETRACT;
                case INTAKE -> DEPLOY;
                case EXHAUST -> inPosition(intakeInputs.filteredLampreyPosition, 8) ? EXHAUST : RETRACT;
                case CLIMB -> CLIMB;
            };
        }
    }, EXHAUST() {
        @Override
        IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs) {
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_EXHAUST;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_STOW;

            return switch (intakeWantedState) {
                case IDLE -> IDLE;
                case INTAKE -> DEPLOY;
                case EXHAUST -> EXHAUST;
                case CLIMB -> CLIMB;
            };
        }
    },
    CLIMB() {
        @Override
        IntakeSystemState handle(IntakeWantedState wantedState, IntakeInputs inputs) {
            this.rollerDuty = IntakeConfig.ROLLER_DUTY_IDLE;
            this.targetAngle = IntakeConfig.DEPLOY_ANGLE_OUT;
            return CLIMB;
        }
    };

    double rollerDuty;
    double targetAngle;

    IntakeSystemState() {}

    abstract IntakeSystemState handle(IntakeWantedState intakeWantedState, IntakeInputs intakeInputs);

    boolean inPosition(double currentAngle, double tolerance) {
        return Math.abs(targetAngle - currentAngle) <= tolerance;
    }

    public double getTargetSpeed() {
        return this.rollerDuty;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public void setOutputs(IntakeOutputs outputs) {
        outputs.rollerDuty = this.rollerDuty;
        outputs.targetAngle = this.targetAngle;
    }

}
