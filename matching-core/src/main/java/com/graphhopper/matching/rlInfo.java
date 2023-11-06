package com.graphhopper.matching;

import java.util.List;

public class rlInfo {
    public List<Double> state;
    public double reward;
    public
    boolean done;

    public rlInfo(List<Double> state,double reward,boolean done){
        this.state = state;
        this.reward = reward;
        this.done = done;
    }
}
