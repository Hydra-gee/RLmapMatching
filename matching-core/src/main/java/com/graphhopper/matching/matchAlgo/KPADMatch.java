package com.graphhopper.matching.matchAlgo;

import com.bmw.hmm.SequenceState;
import com.graphhopper.matching.RLMM;
import com.graphhopper.matching.entities.Observation;
import com.graphhopper.matching.entities.ObservationWithCandidateStates;
import com.graphhopper.matching.entities.State;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

import java.util.*;

import static java.lang.Math.min;

public class KPADMatch extends BaseMatch{
    public KPADMatch(RLMM rlmm) {
        super(rlmm);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        System.out.println("Using KPADMatch");
        int keypointNum = 0;
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        ObservationWithCandidateStates dstTimeStep = timeSteps.get(timeSteps.size()-1);
        List<ObservationWithCandidateStates> tempKeyPoints = new ArrayList<>();
        Map<State,Double> forwardLenMap = new HashMap<>();
        Map<State,SequenceState<State, Observation, Path>> forwardPathMap = new HashMap<>();
        Map<State,State> forwardLastStateMap = new HashMap<>();
        Path bestPathToPrev = null;
        Path bestPathToDst = null;
        State bestDstState = null;
        ObservationWithCandidateStates prevKeypoint = null;
        for(int i=0;i<timeSteps.size()-1;i++){
            if(keypointNum >= rlmm.maxKeypointNum+1){
                break;
            }
            ObservationWithCandidateStates timeStep = timeSteps.get(i);
            //check if this observation is far away from current path
            List<EdgeIteratorState> allEdges = new ArrayList<>();
            if(bestPathToPrev!=null){
                allEdges.addAll(bestPathToPrev.calcEdges());
            }
            if(bestPathToDst!=null){
                allEdges.addAll(bestPathToDst.calcEdges());
            }
            if(!pointNearEdges(timeStep.observation,rlmm.kpDisThreshold,allEdges)){
                tempKeyPoints.add(timeStep);
            }else{
                tempKeyPoints.clear();
            }
            //Check if consecutive observations are potential keypoints.
            if(tempKeyPoints.size()>=rlmm.kpConnumThreshold||i==0){
                keypointNum+=1;
                //choose arbitrary observation as keypoint (here I use the first)
                ObservationWithCandidateStates keyPoint = tempKeyPoints.get(0);
                double minTotalLen = Double.MAX_VALUE;
                for(State candidate:keyPoint.candidates){
                    //calculate the min distance to prev
                    if(prevKeypoint==null){
                        forwardLenMap.put(candidate, 0.0);
                        forwardPathMap.put(candidate,new SequenceState<>(candidate,keyPoint.observation,null));
                    }else{
                        double minLenToLast = Double.MAX_VALUE;
                        SequenceState<State, Observation, Path> bestSequence = null;
                        State bestLastState = null;
                        for(State lastCandidate:prevKeypoint.candidates){
                            if(!forwardLenMap.containsKey(lastCandidate)){
                                continue;
                            }
                            Path path = createRouter().calcPath(lastCandidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(), EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                            if(path.isFound() && path.getDistance() + forwardLenMap.get(lastCandidate) < minLenToLast){
                                minLenToLast = path.getDistance() + forwardLenMap.get(lastCandidate);
                                bestSequence = new SequenceState<>(candidate,keyPoint.observation,path);
                                bestLastState = lastCandidate;
                            }
                        }
                        forwardLenMap.put(candidate,minLenToLast);
                        forwardPathMap.put(candidate,bestSequence);
                        forwardLastStateMap.put(candidate,bestLastState);
                    }
                    //calculate the min distance to dst
                    double minDisToDst = Double.MAX_VALUE;
                    Path minPathToDst = null;
                    State tempDst = null;
                    for(State dstCandidate:dstTimeStep.candidates){
                        Path path = createRouter().calcPath(candidate.getSnap().getClosestNode(), dstCandidate.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                        if(path.isFound() && path.getDistance() < minDisToDst){
                            minDisToDst = path.getDistance();
                            minPathToDst = path;
                            tempDst = dstCandidate;
                        }
                    }
                    if(minDisToDst + forwardLenMap.get(candidate) < minTotalLen){
                        minTotalLen = minDisToDst + forwardLenMap.get(candidate);
                        bestPathToPrev = forwardPathMap.get(candidate).transitionDescriptor;
                        bestPathToDst = minPathToDst;
                        bestDstState = tempDst;
                        forwardLastStateMap.put(bestDstState,candidate);
                        //keyPoint = new ObservationWithCandidateStates(keyPoint.observation,keyPoint.candidates.stream().filter(s->s.getSnap().getClosestNode()==candidate.getSnap().getClosestNode()).collect(Collectors.toList()));
                    }
                }
                prevKeypoint = keyPoint;
                tempKeyPoints.clear();
            }
        }
        result.add(new SequenceState<>(bestDstState,timeSteps.get(timeSteps.size()-1).observation,bestPathToDst));
        State thisCandidate = forwardLastStateMap.get(bestDstState);
        while(forwardPathMap.containsKey(thisCandidate)){
            result.add(forwardPathMap.get(thisCandidate));
            thisCandidate = forwardLastStateMap.get(thisCandidate);
        }
        Collections.reverse(result);
        return result;
    }

    public boolean pointNearEdges(Observation obs,double threshold,List<EdgeIteratorState> edges){
        for(EdgeIteratorState e:edges){
            double n1lat = rlmm.queryGraph.getNodeAccess().getLatitude(e.getBaseNode());
            double n1lon = rlmm.queryGraph.getNodeAccess().getLongitude(e.getBaseNode());
            double n2lat = rlmm.queryGraph.getNodeAccess().getLatitude(e.getAdjNode());
            double n2lon = rlmm.queryGraph.getNodeAccess().getLongitude(e.getAdjNode());
            double dis = calcNormalizedEdgeDistance(obs.getPoint().lat, obs.getPoint().lon,n1lat,n1lon,n2lat,n2lon);
            dis = rlmm.distanceCalc.calcDenormalizedDist(dis);
            if (dis < threshold){
                return true;
            }
        }
        return false;
    }

}
