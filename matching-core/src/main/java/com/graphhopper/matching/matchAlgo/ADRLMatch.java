package com.graphhopper.matching.matchAlgo;

import com.bmw.hmm.SequenceState;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.matching.RLMM;
import com.graphhopper.matching.entities.Observation;
import com.graphhopper.matching.entities.ObservationWithCandidateStates;
import com.graphhopper.matching.entities.State;
import com.graphhopper.matching.entities.rlInfo;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.*;

import static java.lang.Integer.parseInt;
import static java.lang.Math.min;

public class ADRLMatch extends BaseMatch{

    public ADRLMatch(RLMM rlmm) {
        super(rlmm);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        long t1 = System.currentTimeMillis();
        String hostName = "localhost";
        int portNumber = 7878;
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        try (
                Socket echoSocket = new Socket(hostName, portNumber);
                PrintWriter out =
                        new PrintWriter(echoSocket.getOutputStream(), true);
                BufferedReader in =
                        new BufferedReader(
                                new InputStreamReader(echoSocket.getInputStream()))
        ) {
            int keypointNum = 0;
            ObservationWithCandidateStates dstTimeStep = timeSteps.get(timeSteps.size()-1);
            Map<State,Double> forwardLenMap = new HashMap<>(); //the min distance from origin to state
            Map<State,SequenceState<State, Observation, Path>> forwardPathMap = new HashMap<>();
            Map<State,State> forwardPrevStateMap = new HashMap<>(); //the previous state of the min path from origin to state
            Path bestPathToPrev = null;
            Path bestPathToDst = null;
            State bestDstState = null;
            ObservationWithCandidateStates prevKeypoint = null;
            int action = 0;
            boolean isKey = false;
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
                double obsToPathDis = pointToEdgesDis(timeStep.observation,allEdges);
                //origin obs is keypoint
                isKey = false;
                if(i==0){
                    isKey = true;
                }
                //if(i>0 && obsToPathDis > this.measurementErrorSigma){
                if(i>0){
                    ObjectMapper mapper = new ObjectMapper();
                    List<Double> nextState = new ArrayList<>();
                    nextState.add(obsToPathDis);
                    double d1 = rlmm.distanceCalc.calcDist(timeSteps.get(i-1).observation.getPoint().lat,
                            timeSteps.get(i-1).observation.getPoint().lon, timeSteps.get(i).observation.getPoint().lat, timeSteps.get(i).observation.getPoint().lon);
                    double d2 = rlmm.distanceCalc.calcDist( timeSteps.get(i).observation.getPoint().lat,
                            timeSteps.get(i).observation.getPoint().lon,  timeSteps.get(i+1).observation.getPoint().lat,  timeSteps.get(i+1).observation.getPoint().lon);
                    nextState.add(d1);
                    nextState.add(d2);
                    int reward = action == 0?0:-1;
                    isKey = action != 0;
                    rlInfo rlreturn = new rlInfo(nextState,reward,false);
                    out.println(mapper.writeValueAsString(rlreturn)); // write state
                    String info = in.readLine(); // get action
                    action = parseInt(info);
                }
                //update path
                if(isKey){
                    keypointNum+=1;
                    ObservationWithCandidateStates keyPoint = timeSteps.get(i);
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
                            forwardPrevStateMap.put(candidate,bestLastState);
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
                            forwardPrevStateMap.put(bestDstState,candidate);
                            //keyPoint = new ObservationWithCandidateStates(keyPoint.observation,keyPoint.candidates.stream().filter(s->s.getSnap().getClosestNode()==candidate.getSnap().getClosestNode()).collect(Collectors.toList()));
                        }
                    }
                    prevKeypoint = keyPoint;
                }
            }
            ObjectMapper mapper = new ObjectMapper();
            List<Double> nextState = new ArrayList<>();
            nextState.add(0.0);nextState.add(0.0);nextState.add(0.0);
            int reward = action == 0?0:-1;
            rlInfo rlreturn = new rlInfo(nextState,reward,true);
            out.println(mapper.writeValueAsString(rlreturn)); // 写入套接字
            echoSocket.close();
            result.add(new SequenceState<>(bestDstState,timeSteps.get(timeSteps.size()-1).observation,bestPathToDst));
            State thisCandidate = forwardPrevStateMap.get(bestDstState);
            while(forwardPathMap.containsKey(thisCandidate)){
                result.add(forwardPathMap.get(thisCandidate));
                thisCandidate = forwardPrevStateMap.get(thisCandidate);
            }
            Collections.reverse(result);
        }catch (UnknownHostException e) {
            System.err.println("Don't know about host " + hostName);
            System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to " +
                    hostName);
            System.exit(1);
        }
        long t2 = System.currentTimeMillis();
        System.out.println(t2-t1);
        return result;
    }
}
