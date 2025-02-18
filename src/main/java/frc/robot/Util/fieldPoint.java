package frc.robot.Util;

public class fieldPoint {
    private double x, y;
    /*x,y point on a field for fieldPoly class */
    public fieldPoint(double x,double y){
        this.x = x;
        this.y = y;
    }
    public double getX(){
        return this.x;
    }
    public double getY(){
        return this.y;
    }
}
