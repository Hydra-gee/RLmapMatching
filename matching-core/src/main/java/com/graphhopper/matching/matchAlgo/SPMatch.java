package com.graphhopper.matching.matchAlgo;

import com.bmw.hmm.SequenceState;
import com.graphhopper.matching.RLMM;
import com.graphhopper.matching.entities.Observation;
import com.graphhopper.matching.entities.ObservationWithCandidateStates;
import com.graphhopper.matching.entities.State;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIterator;

import java.util.*;

public class SPMatch extends BaseMatch{

    public SPMatch(RLMM rlmm) {
        super(rlmm);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        System.out.println("Using SPMatch");
        double timethresold = 10000;
        Map<State, Double> sd = new HashMap<>();
        Map<State, State> bl = new HashMap<>();
        Map<State,SequenceState<State, Observation, Path>> bd = new HashMap<>();
        ObservationWithCandidateStates lastTimeStep = null;
        for(ObservationWithCandidateStates timeStep : timeSteps){
            boolean allCandidateUnreachable = true;
            //第一个观测点的候选点，总距离为0
            for(State candidate:timeStep.candidates){
                if(lastTimeStep==null){
                    sd.put(candidate, 0.0);
                    allCandidateUnreachable = false;
                }else{
                    double minLen = Double.MAX_VALUE;
                    State bestLastCandidate = null;
                    Path bestPath = null;
                    for(State lastCandidate:lastTimeStep.candidates){
                        if(!sd.containsKey(lastCandidate)){
                            continue;
                        }
                        Path path = createRouter().calcPath(lastCandidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(), EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                        if(path.isFound()){
                            if(path.getDistance() + sd.get(lastCandidate) < minLen){
//                                if(lastCandidate.getSnap().getSnappedPoint().lat-31.34828316246198 <=0.00000000000001&&
//                                candidate.getSnap().getSnappedPoint().lat-31.34853276644348 <=0.00000000000001){
//                                    int a = 1;
//                                }
                                minLen = path.getDistance() + sd.get(lastCandidate);
                                bestLastCandidate = lastCandidate;
                                bestPath = path;
                            }
                        }
                    }
                    if(bestPath==null){
                        continue;
                    }
                    double v = bestPath.getDistance() * 1000/ (timeStep.observation.getTime().getTime() - lastTimeStep.observation.getTime().getTime());
                    if(v < 35){
                        sd.put(candidate,minLen);
                        bl.put(candidate,bestLastCandidate);
                        bd.put(candidate,new SequenceState<>(candidate,timeStep.observation,bestPath));
                        allCandidateUnreachable = false;
                    }
                }
            }
            if(!allCandidateUnreachable){
                lastTimeStep = timeStep;
            }
        }
        State minLastState = null;
        Double minTotalLen = Double.MAX_VALUE;
        //计算终点对应的状态点
        for(State candidate:lastTimeStep.candidates){
            if(sd.containsKey(candidate) && sd.get(candidate) < minTotalLen){
                minTotalLen = sd.get(candidate);
                minLastState = candidate;
            }
        }
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        State thisCandidate = minLastState;
        while(bl.containsKey(thisCandidate)){
            result.add(bd.get(thisCandidate));
            thisCandidate = bl.get(thisCandidate);
        }
        Collections.reverse(result);
        return result;
    }
}
