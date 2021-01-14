package org.firstinspires.ftc.teamcode.AutonomousPathGeneration;

import java.util.ArrayList;

public class Map {

    public Node startNode;
    public Node goalNode;
    ArrayList<ArrayList<Node>> quadrant1 = new ArrayList<>();   //domain: 0-infinity    range: 0-infinity
    ArrayList<ArrayList<Node>> quadrant2 = new ArrayList<>();   //domain: 1-infinity    range: 0-infinity
    ArrayList<ArrayList<Node>> quadrant3 = new ArrayList<>();   //domain: 1-infinity    range: 1-infinity
    ArrayList<ArrayList<Node>> quadrant4 = new ArrayList<>();   //domain: 0-infinity    range: 1-infinity

    public Map() {

    }

    public ArrayList<Node> getNeighbors(Node n) {
        /*
        Function that returns all neighbors of a node. Checks in each quadrant, and ensure the neihgbor is not closed and is traversable.
         */

        ArrayList<Node> neighbors = new ArrayList<>();
        Node check;

        if (n.row > 0) {
            if (n.col > 0) {    //row>0 and col>0
                for (int i=n.row-1; i<n.row+3; i++) {
                    for (int j=n.col-1; j<n.col+3; j++) {
                        if (i==n.row && j==n.col) { //Do nothing; this is the input node
                        }
                        else {
                            check = getNode(n.quadrant, i, j);
                            if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                                neighbors.add(n);
                            }
                        }
                    }
                }
            }
            else {  //row>0 and col<=0
                for (int i=n.row-1; i<n.row+3; i++) {
                    for (int j=n.col; j<n.col+2; j++) {
                        if (i==n.row && j==n.col) { //Do nothing; this is the input node
                        }
                        else {
                            check = getNode(n.quadrant, i, j);
                            if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                                neighbors.add(n);
                            }
                        }
                    }
                    check = getNode(n.quadrant, i, -1);
                    if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                        neighbors.add(n);
                    }
                }
            }
        }
        else {
            if (n.col > 0) {    //row<=0 and col>0
                for (int i=n.row; i<n.row+2; i++) {
                    for (int j=n.col-1; j<n.col+3; j++) {
                        if (i==n.row && j==n.col) { //Do nothing; this is the input node
                        }
                        else {
                            check = getNode(n.quadrant, i, j);
                            if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                                neighbors.add(n);
                            }
                        }
                    }
                }
                check = getNode(n.quadrant, -1, n.col-1);
                if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                    neighbors.add(n);
                }
                check = getNode(n.quadrant, -1, n.col);
                if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                    neighbors.add(n);
                }
                check = getNode(n.quadrant, -1, n.col+1);
                if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                    neighbors.add(n);
                }
            }
            else {  //row<=0 and col<=0
                for (int i=n.row; i<n.row+2; i++) {
                    for (int j=n.col; j<n.col+2; j++) {
                        if (i==n.row && j==n.col) { //Do nothing; this is the input node
                        }
                        else {
                            check = getNode(n.quadrant, i, j);
                            if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                                neighbors.add(n);
                            }
                        }
                    }
                    check = getNode(n.quadrant, i, -1);
                    if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                        neighbors.add(n);
                    }
                }
                check = getNode(n.quadrant, -1, -1);
                if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                    neighbors.add(n);
                }
                check = getNode(n.quadrant, -1, n.col-1);
                if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                    neighbors.add(n);
                }
                check = getNode(n.quadrant, -1, n.col);
                if (check != startNode && check.usage != NodeUsage.OBSTACLE && check.usage != NodeUsage.SAFETY && check.status != NodeStatus.CLOSED) {
                    neighbors.add(n);
                }
            }
        }
        return null;
    }

    public Node getNode(int quadrant, int row, int col) {
        /*
        getNode function if the quadrant is known
         */
        switch (quadrant) {
            case 1:
                return quadrant1.get(Math.abs(row)).get(Math.abs(col));
            case 2:
                return quadrant2.get(Math.abs(row)).get(Math.abs(col));
            case 3:
                return quadrant3.get(Math.abs(row)).get(Math.abs(col));
            case 4:
                return quadrant4.get(Math.abs(row)).get(Math.abs(col));
            default:
                return null;
        }
    }

    public Node getNode(int row, int col) {
        /*
        getNode function if the quadrant is known, and instead we only have cartesian row, column coords
         */
        if (row >= 0) {
            if (col >= 0) {
                return quadrant1.get(Math.abs(row)).get(Math.abs(col));
            }
            else {
                return quadrant4.get(Math.abs(row)).get(Math.abs(col));
            }
        }
        else {
            if (col >= 0) {
                return quadrant2.get(Math.abs(row)).get(Math.abs(col));
            }
            else {
                return quadrant3.get(Math.abs(row)).get(Math.abs(col));
            }
        }
    }

}
