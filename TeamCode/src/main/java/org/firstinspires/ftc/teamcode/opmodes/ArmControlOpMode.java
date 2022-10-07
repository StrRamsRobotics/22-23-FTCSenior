package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="ArmTestOpMode")
public class ArmControlOpMode extends LinearOpMode {

    MotorWrapper arm;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup
        arm = new MotorWrapper(hardwareMap.get(DcMotor.class, "arm"));
        arm.setTargetRelative(800);
        // prerun
        waitForStart();

        // run loop
        while (opModeIsActive()){
            arm.update();
        }
    }
}
