package org.firstinspires.ftc.teamcode.iLQR;

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;


import java.util.ArrayList;

public class Field {
    private ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();

    public Field(ArrayList<Obstacle> obstacles){
        this.obstacles = obstacles;

    }

    public void addObstacle(Obstacle obst){
        obstacles.add(obst);
    }
    public SimpleMatrix ObstacleCostHessian(Point2D robotPos){
        double[][] Qarr = {{0,0},{0,0}};
        SimpleMatrix Q = new SimpleMatrix(Qarr);
        for (int i = 0; i < obstacles.size(); i++) {
            if(obstacles.get(i).isInRange(robotPos)){
                 Q = Q.plus(obstacles.get(i).computeHessian(robotPos));
            }

        }
        return Q;

    }
    public SimpleMatrix ObstacleCostGradient(Point2D robotPos){
        double[][] qarr = {{0},{0}};
        SimpleMatrix q = new SimpleMatrix(qarr);
        for (int i = 0; i < obstacles.size(); i++) {
            if(obstacles.get(i).isInRange(robotPos)){
                q = q.plus(obstacles.get(i).computeGradient(robotPos));
            }

        }
        return q;

    }
    public double ObstacleCost(Point2D robotPos){
        double cost = 0;
        for (int i = 0; i < obstacles.size(); i++) {
            if(obstacles.get(i).isInRange(robotPos)){
                cost += obstacles.get(i).computeCost(robotPos);
            }

        }
        return cost;

    }
}
