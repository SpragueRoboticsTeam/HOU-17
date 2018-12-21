package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
@Autonomous
public class servoTest extends LinearOpMode {

    CRServo servo = null;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.crservo.get("servo");
        telemetry.addData("Mode", "waiting for start");
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while(opModeIsActive()){
            while(true){
                servo.setPower(0.5);
                sleep(3000);
                servo.setPower(-0.5);
                sleep(3000);
            }


        }
        servo.setPower(0);
    }
}
