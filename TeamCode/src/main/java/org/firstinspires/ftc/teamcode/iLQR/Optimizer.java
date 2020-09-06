package org.firstinspires.ftc.teamcode.iLQR;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;


public class Optimizer {
    private double timestep;
    private SimpleMatrix A;
    private SimpleMatrix B;
    private final double costConvergence = 10;
    private Field field;

    public Optimizer(SimpleMatrix A, SimpleMatrix B, double timestep){
        //create field with all obstacles
        Obstacle alliancebotredparked = new Obstacle(new Point2D(12,72),9,50,2);
        Obstacle alliancebotblueparked = new Obstacle(new Point2D(132,72),9,50,2);

        Obstacle alliancebotredstart = new Obstacle(new Point2D(12,54),9,50,2);
        Obstacle alliancebotbluestart = new Obstacle(new Point2D(132,54),9,50,2);

        Obstacle neutralbridgered = new Obstacle(new Point2D(57,72),9,50,-2);
        Obstacle neutralbridgecenter = new Obstacle(new Point2D(72,72),9,50,2);
        Obstacle neutralbridgeblue = new Obstacle(new Point2D(87,72),9,50,2);

        field = new Field(new ArrayList<>(Arrays.asList(alliancebotredparked,alliancebotblueparked,alliancebotredstart,alliancebotbluestart,neutralbridgered,neutralbridgecenter,neutralbridgeblue)));

        //get system information
        //get timestep
        this.timestep = timestep;
        this.A = A;
        this.B = B;
    }
    public Object[] solveForControls(SimpleMatrix A, SimpleMatrix B, SimpleMatrix[] Q, SimpleMatrix[] R,
                                 SimpleMatrix[] q, SimpleMatrix[] r, SimpleMatrix Qf, SimpleMatrix qf){
        int H = Q.length;
        SimpleMatrix V_x = qf;
        SimpleMatrix V_xx = Qf;
        SimpleMatrix [] K = new SimpleMatrix[H];
        SimpleMatrix [] d = new SimpleMatrix[H];
        for(int i = 0; i<H; i++){
            // Calculating linear approximations of the cost to go function
            SimpleMatrix Q_x = q[H-i-1].plus(A.transpose().mult(V_x));
            SimpleMatrix Q_u = r[H-i-1].plus(B.transpose().mult(V_x));

            //Quadratic Approximations of the cost to go function
            SimpleMatrix Q_xx = Q[H-i-1].plus(A.transpose().mult(V_xx.mult(A)));
            //l_ux is zero since there are no cross terms
            //Q_ux = l_ux[t] + np.dot(B.T, np.dot(V_xx, A))
            SimpleMatrix Q_ux = B.transpose().mult(V_xx.mult(A));
            SimpleMatrix Q_uu = R[H-i-1].plus(B.transpose().mult(V_xx.mult(B)));

            //finding inverse of the second partial with respect to control u
            SimpleMatrix Q_uu_inv = Q_uu.invert();

            //5b) k = -np.dot(Q_uu^-1, Q_u)
            d[H-i-1] = (Q_uu_inv.mult(Q_u)).negative();
            K[H-i-1] = (Q_uu_inv.mult(Q_ux)).negative();

            //updating cost to go approximations
            V_x = Q_x.minus(K[H-i-1].transpose().mult(Q_uu.mult(d[H-i-1])));
            // 6c) V_xx = Q_xx - np.dot(-K^T, np.dot(Q_uu, K))
            V_xx = Q_xx.minus(K[H-i-1].transpose().mult(Q_uu.mult(K[H-i-1])));

        }
        return new Object[]{K, d};

    }
    public Object[] optimizePath(SimpleMatrix startPoint, SimpleMatrix endxPoint, double seconds){
        SimpleMatrix Q = SimpleMatrix.identity(2).scale(30);
        SimpleMatrix R = SimpleMatrix.identity(2).scale(1);

        double[][] qarr = {{0},{0}};
        SimpleMatrix q = new SimpleMatrix(qarr);

        double[][] rarr = {{0},{0}};
        SimpleMatrix r = new SimpleMatrix(rarr);

        int horizon = (int)(seconds/timestep);

        SimpleMatrix [] Qlis = new SimpleMatrix[horizon];
        SimpleMatrix [] Rlis = new SimpleMatrix[horizon];
        SimpleMatrix [] qlis = new SimpleMatrix[horizon];
        SimpleMatrix [] rlis = new SimpleMatrix[horizon];

        for (int i = 0; i < horizon; i++) {
            Qlis[i] = new SimpleMatrix(Q);
            Rlis[i] = new SimpleMatrix(R);
            qlis[i] = new SimpleMatrix(q);
            rlis[i] = new SimpleMatrix(r);
        }


        SimpleMatrix[] Klis = new SimpleMatrix[horizon];
        SimpleMatrix[] dlis = new SimpleMatrix[horizon];

        SimpleMatrix[] xlist = new SimpleMatrix[horizon+1];
        SimpleMatrix[] lastxlist = new SimpleMatrix[horizon+1];

        SimpleMatrix[] ulis = new SimpleMatrix[horizon];
        SimpleMatrix u = r;
        SimpleMatrix x;


        double cost = 0;
        double lastcost = 1000000000;
        int iterationCount = 0;
        for(;iterationCount<100;){
            System.out.println("Iteration: " + iterationCount);
            lastcost = cost;
            cost = 0;
            x=startPoint;
            lastxlist = xlist;
            xlist[0] = startPoint;
            Object[] a = solveForControls(A,B,Qlis,Rlis,qlis,rlis,Q,q);
            Klis = (SimpleMatrix[]) a[0];
            dlis = (SimpleMatrix[]) a[1];

            for (int i = 0; i < horizon; i++) {
                if(iterationCount == 0){
                    u = ((Klis[i].mult(endxPoint.minus(x))).negative()).plus(dlis[i]);
                    ulis[i] = new SimpleMatrix(u);
                }
                else{
                    u = (Klis[i].mult(lastxlist[i].minus(x).negative())).plus(dlis[i].scale(1)).plus(ulis[i]);
                    ulis[i] = new SimpleMatrix(u);
                }
                x = A.mult(x).plus(B.mult(u));
                Point2D currentpoint = new Point2D(x.get(0,0),x.get(1,0)) ;
                cost += field.ObstacleCost(currentpoint) + (endxPoint.minus(x)).transpose().mult(Q).mult(endxPoint.minus(x)).get(0,0);
                qlis[i] = field.ObstacleCostGradient(currentpoint);
                Qlis[i] = field.ObstacleCostHessian(currentpoint);

                xlist[i+1] = x;
            }
            iterationCount++;
            System.out.println("Total Cost: " + cost);
            q = Q.mult(x.minus(endxPoint)).scale(2);

        }
        dlis = ulis;
        return new Object[]{Klis,dlis,xlist};
    }


}
