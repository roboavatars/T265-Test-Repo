package org.firstinspires.ftc.teamcode.iLQR;


import org.ejml.simple.SimpleMatrix;



public class Obstacle {
    private Point2D position;
    private double obstacleRadius;
    private double robotradius = 9*Math.sqrt(2);

    //constants in cost computation
    private double alpha;
    private double beta;
    private double range;

    public Obstacle(Point2D position, double obstacleRadius ,double alpha, double range){
        this.position = position;
        this.obstacleRadius = obstacleRadius + robotradius;
        this.alpha = alpha;
        this.range = range;
        beta = 1/(Math.pow(obstacleRadius,2));
    }
    public double computeCost(Point2D robotPos){
        double cost = alpha*Math.pow(Math.E, -beta*robotPos.distance(position));
        return cost;
    }
    public SimpleMatrix computeGradient(Point2D robotPos){
        double cost = computeCost(robotPos);
        double[][] garr = {{2*beta *(position.getX()-robotPos.getX())},{2*beta*(position.getY()-robotPos.getY())}};
        SimpleMatrix g = new SimpleMatrix(garr);
        g = g.scale(cost);
        return g;
    }
    public SimpleMatrix computeHessian(Point2D robotPos){
        double cost = computeCost(robotPos);
        double[][] Harr = {{4*(Math.pow(beta,2))*(Math.pow(robotPos.getX()-position.getX(),2))-2*beta,
                            4*(Math.pow(beta,2))*(robotPos.getX()-position.getX())*(robotPos.getY()-position.getY())},
                           {4*(Math.pow(beta,2))*(robotPos.getX()-position.getX())*(robotPos.getY()-position.getY()),
                                   4*(Math.pow(beta,2))*(Math.pow(robotPos.getY()-position.getY(),2))-2*beta}};

        SimpleMatrix H = new SimpleMatrix(Harr);
        H = H.scale(cost);
        return H;
    }
    public boolean isInRange(Point2D robotPos){
        if(robotPos.distance(position)<=(obstacleRadius+range)){
            return true;
        }
        else{
            return false;
        }
    }

}
