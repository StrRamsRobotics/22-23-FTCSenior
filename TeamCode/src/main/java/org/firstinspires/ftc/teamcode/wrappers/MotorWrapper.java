package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorWrapper {
    final double PI = 3.14159265;
    final int MAX_TICKS = 500;
    final int TICKS_PER_SPIN = 1440;

    private DcMotor motor;
    int passed_ticks = 0, current = 0, past = 0;

    public MotorWrapper(DcMotor motor){
        this.motor = motor;
        // set motor to use encoders
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update()
    {
        past = current;
        current = getCurrentTicks();
        passed_ticks = current - past;
    }

    public int getCurrentTicks(){
        return this.motor.getCurrentPosition();
    }

    public int getDeltaTicks(){
        return this.passed_ticks;
    }

    public double getDistanceTravelledThisUpdate(int ticksPerRotation, double wheelRadius){
        return wheelRadius * ((double)passed_ticks / (double)(ticksPerRotation));
    }

    public double getTotalDistanceTravelled(int ticksPerRotation, double wheelRadius){
        return wheelRadius * ((double)current / (double)ticksPerRotation);
    }


}
