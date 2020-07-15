package org.firstinspires.ftc.teamcode.shared;

public interface Clock {
    /**
     * Returns time in milliseconds since last call to reset
     * */
    long getCurrentTime();
    void reset();
}
