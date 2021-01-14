package org.firstinspires.ftc.teamcode.AutonomousPathGeneration;

public class Node {

    public NodeStatus status;

    public double x;
    public double y;

    public int row;
    public int col;
    public int quadrant;

    public NodeUsage usage;
    public double gScore;   //distance from start node
    public double hScore;   //heuristic; distance from end node
    private double fScore;   //sum of f and g scores

    public Node parentNode;

    public Node(double x, double y, int row, int col, int quadrant) {
        this.x = x;
        this.y = y;
        this.row = row;
        this.col = col;
        this.quadrant = quadrant;
        gScore = -1;
        hScore = -1;
        fScore = -1;
        status = NodeStatus.NONE;
    }

    public double fScore() {
        return (gScore + hScore);
    }

}
