
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="stupidDriver", group="Iterative Opmode")
public class stupidDriver extends OpMode {
    ElapsedTime runtime = new ElapsedTime();

    // DcMotor LeftFrontWheels = null;
    DcMotor LeftBackWheels = null;
    //  DcMotor RightFrontWheels = null;
    DcMotor RightBackWheels = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //  LeftFrontWheels = hardwareMap.get(DcMotor.class,"LFW");
        //  RightFrontWheels = hardwareMap.get(DcMotor.class,"RFW");
        RightBackWheels = hardwareMap.get(DcMotor.class,"RBW");
        LeftBackWheels = hardwareMap.get(DcMotor.class,"LBW");

        //LeftFrontWheels.setDirection(DcMotor.Direction.FORWARD);
        LeftBackWheels.setDirection(DcMotor.Direction.FORWARD);
        // RightFrontWheels.setDirection(DcMotor.Direction.REVERSE);
        RightBackWheels.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {

        double leftPower;
        double rightPower;
        double liftPower;

        leftPower  = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        if(leftPower != 0 || rightPower != 0) {
            //LeftFrontWheels.setPower(leftPower);
            LeftBackWheels.setPower(leftPower);
            //RightFrontWheels.setPower(rightPower);
            RightBackWheels.setPower(rightPower);
        }



        else {
            // RightFrontWheels.setPower(0);
            RightBackWheels.setPower(0);
            LeftBackWheels.setPower(0);
            // LeftFrontWheels.setPower(0);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    }

    @Override
    public void stop() {
    }

}
