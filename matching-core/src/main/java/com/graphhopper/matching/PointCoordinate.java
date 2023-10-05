package com.graphhopper.matching;

public class PointCoordinate {
    public double obsLat;
    public double obsLon;
    public double canLat;
    public double canLon;
    public PointCoordinate(double lat1,double lon1, double lat2, double lon2){
        this.obsLat = lat1;
        this.obsLon = lon1;
        this.canLat = lat2;
        this.canLon = lon2;
    }
}
