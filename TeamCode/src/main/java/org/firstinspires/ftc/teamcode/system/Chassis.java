package org.firstinspires.ftc.teamcode.system;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis extends System{

    DcMotor motorLeft;
    DcMotor motorRight;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;
  
    public Chassis(Telemetry telemetry){
        super(telemetry);
        
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        color1  = hardwareMap.get(ColorSensor.class, "color1");
        distance1  = hardwareMap.get(DistanceSensor.class, "distance1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        motorLeft.setDirection("FORWARD");
        motorRight.setDirection("FORWARD");

    }

    @Override
    public void update() {
        
    }
    
    public void move(float left, float right){
        motorLeft.setPower(left);
        motorRight.setPower(right);
    }
    
    public float checkDistance(DcMotor motor){
        float value = motor.getCurrentPosition();
        return value;
    }

    public void goStraight(float distance){
        // convert
        float leftDistance = this.checkDistance(this.motorLeft);
        float rightDistance = this.checkDistance(this.motorRight);
        
        float averageDistance = (leftDistance + rightDistance) / 2;
        
        while(averageDistance != distance){
            float left = this.PID(leftDistance, target, 0.1, 0.1, 0.1);
            float right = this.PID(rightDistance, target, 0.1, 0.1, 0.1);
            this.move(left, right);
            
            // Update
            float leftDistance = this.checkDistance(this.motorLeft);
            float rightDistance = this.checkDistance(this.motorRight);

            float averageDistance = (leftDistance + rightDistance) / 2;
        }
    }
    
    public float PID(float input, float target, float Kp, float Ki, float Kd) {
        static int prevError = 0, integral = 0;

        double error = (target - input);
        double derivative = error - prevError;  // only an approximation
        integral = 0.5 * integral + error;  // only an approximation
        prevError = error;

        return Kp * error + Kd * derivative + Ki * integral;
    }
    
    public void turn(float degree, float radius){
        float theta = degree;
        S = theta * radius;
        
    }
}
