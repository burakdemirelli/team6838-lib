package com.team6838.lib.util;

public class Gearbox {

    private double gearRatio;
    
    public Gearbox(double drivingGear, double drivenGear){
        this.gearRatio = drivenGear / drivingGear;
    }

    public double getGearboxOutput(double rot){
        return rot*gearRatio;
    }

    public double getGearRatio(){
        return gearRatio;
    }
}
