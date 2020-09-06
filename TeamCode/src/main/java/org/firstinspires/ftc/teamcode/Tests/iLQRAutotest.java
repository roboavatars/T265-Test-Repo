package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.iLQR.Optimizer;

@Autonomous @Disabled
public class iLQRAutotest extends LinearOpMode {

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this,9,111, 0, true);
        waitForStart();

        double [][] x0 = {{9},{111}};
        double [][] xf = {{20},{30}};

        double timestep = 0.02;
        double seconds = 3;

        SimpleMatrix[] Klis;
        SimpleMatrix [] dlis;
        SimpleMatrix [] xlis;
        SimpleMatrix A = SimpleMatrix.identity(2);
        SimpleMatrix B = SimpleMatrix.identity(2).scale(timestep);



        Optimizer optim = new Optimizer(A,B,timestep);

        SimpleMatrix startx = new SimpleMatrix(x0);
        SimpleMatrix finalx = new SimpleMatrix(xf);

        Object[] a = optim.optimizePath(startx,finalx,seconds);
        Klis = (SimpleMatrix[])a[0];
        dlis = (SimpleMatrix[])a[1];
        xlis = (SimpleMatrix[])a[2];
        double [][] uarr = {{0},{0}};
        SimpleMatrix u = new SimpleMatrix(uarr);

        time.reset();
        int counter = 0;
        while(opModeIsActive()){
            if(counter < (int)(seconds/timestep)-1){
                double[][] xarr = {{robot.drivetrain.x}, {robot.drivetrain.y}};
                SimpleMatrix x = new SimpleMatrix(xarr);
                u = ((Klis[counter].scale(1)).mult(xlis[counter].minus(x)).negative()).plus(dlis[counter].scale(0));
//                robot.drivetrain.setGlobalControls(u.get(0,0)/64,u.get(1,0)/64,0);
                robot.drivetrain.setTargetPoint(xlis[counter].get(0,0),xlis[counter].get(1,0),0);


            }
            else{
                robot.drivetrain.setTargetPoint(xf[1][0], xf[1][0],0);

            }
            counter = (int)(time.seconds()/timestep);

            robot.update();



        }
    }
}
