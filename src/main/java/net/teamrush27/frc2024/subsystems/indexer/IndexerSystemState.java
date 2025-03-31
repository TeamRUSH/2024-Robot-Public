package net.teamrush27.frc2024.subsystems.indexer;

import net.teamrush27.frc2024.subsystems.indexer.IndexerIO.*;

public enum IndexerSystemState {
    IDLE(IndexerConfig.INDEXER_IDLE_RPM) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch(indexerWantedState) {
                case IDLE -> IDLE;
                case FEED -> FEED;
                case INDEX -> INTAKE;
                case EXHAUST -> EXHAUST;
            };
        }
    },
    INTAKE(IndexerConfig.INDEXER_INTAKE_RPM) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch (indexerWantedState) {
                case IDLE -> (inputs.frontSensorDebounced || inputs.backSensorDebounced) ? INDEX : IDLE; // TODO really should not look at the backSensor.
                case FEED -> FEED;
                case INDEX -> (inputs.frontSensorDebounced || inputs.backSensorDebounced) ? INDEX : INTAKE; // No reason to.
                case EXHAUST -> EXHAUST;
            };
        }
    },
    INDEX(IndexerConfig.INDEXER_INDEX_RPM) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch(indexerWantedState) {
                case IDLE, INDEX -> (!inputs.frontSensorDebounced && inputs.backSensorDebounced) ? LOADING : INDEX;
                case FEED -> FEED;
                case EXHAUST -> EXHAUST;
            };
        }
    },
    LOADING(IndexerConfig.INDEXER_LOADING_RPM) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch (indexerWantedState) {
                case INDEX, IDLE -> inputs.frontSensorDebounced ? LOADED : LOADING;
                case FEED -> FEED;
                case EXHAUST -> EXHAUST;
            };
        }
    },
    LOADED(0) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch (indexerWantedState) {
                case IDLE, INDEX -> LOADED;
                case FEED -> FEED;
                case EXHAUST -> EXHAUST;
            };
        }
    },
    FEED(IndexerConfig.INDEXER_FEED_RPM) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch (indexerWantedState) {
                case IDLE -> IDLE;
                case INDEX -> INDEX;
                case FEED -> FEED;
                case EXHAUST -> EXHAUST;
            };
        }
    },
    EXHAUST(0) {
        @Override
        public IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs) {
            return switch (indexerWantedState) {
                case IDLE, INDEX, FEED -> IDLE;
                case EXHAUST -> EXHAUST;
            };
        }
    };

    public final double indexRpm;

    IndexerSystemState(double rpm) {
        this.indexRpm = rpm;
    }

    public double getRpm() {
        return indexRpm;
    }

    public abstract IndexerSystemState handle(IndexerWantedState indexerWantedState, IndexerInputs inputs);
}
