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
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import static java.lang.Integer.parseInt;
import static java.lang.Math.min;

public class RLMatch extends BaseMatch{
    public RLMatch(RLMM rlmm) {
        super(rlmm);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        String hostName = "localhost";
        int portNumber = 7878;
        try (
                Socket echoSocket = new Socket(hostName, portNumber);
                PrintWriter out =
                        new PrintWriter(echoSocket.getOutputStream(), true);
                BufferedReader in =
                        new BufferedReader(
                                new InputStreamReader(echoSocket.getInputStream()))
        ) {
            //计算起点和终点
            int oriPos = 0,dstPos = timeSteps.size()-1;
            SequenceState<State, Observation, Path> ori = null;
            SequenceState<State, Observation, Path> dst = null;
            Path tempPath;
            Path bestTempPath = null;
            double minPathLen = Double.MAX_VALUE;
            for(int i = 0;i<timeSteps.size() && bestTempPath==null;i++){
                if (timeSteps.get(i).candidates.isEmpty()){
                    continue;
                }
                for(int j = timeSteps.size()-1;j>=0 && bestTempPath==null;j--){
                    if (timeSteps.get(j).candidates.isEmpty()){
                        continue;
                    }
                    for(State oriCandidates:timeSteps.get(i).candidates){
                        for(State dstCandidates:timeSteps.get(j).candidates){
                            Path p = createRouter().calcPath(oriCandidates.getSnap().getClosestNode(), dstCandidates.getSnap().getClosestNode(), EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
//                        Path p = createRouter().calcPath(oriCandidates.getSnap().getClosestNode(), dstCandidates.getSnap().getClosestNode(), oriCandidates.isOnDirectedEdge() ? oriCandidates.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE, dstCandidates.isOnDirectedEdge() ? dstCandidates.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
                            if(p.isFound()&&p.getDistance()<minPathLen){
                                minPathLen = p.getDistance();
                                bestTempPath = p;
                                oriPos = i;
                                dstPos = j;
                                ori = new SequenceState<>(oriCandidates, timeSteps.get(i).observation, null);
                                dst = new SequenceState<>(dstCandidates, timeSteps.get(j).observation, null);

                            }
                        }
                    }
                }
            }
            result.add(ori);
            long t2 = System.currentTimeMillis();
            tempPath = bestTempPath;
            int action = 0;
            for(int i=oriPos+1;i<dstPos;i++){
                if(result.size()>rlmm.maxKeypointNum)break;
                //计算到临时路径距离
                ObservationWithCandidateStates timeStep = timeSteps.get(i);
                Observation o = timeStep.observation;
                List<EdgeIteratorState> allEdges = result.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());
                allEdges.addAll(tempPath.calcEdges());
                boolean isKey = true;
                double minDis = Double.MAX_VALUE;
                for(EdgeIteratorState e:allEdges){
                    double n1lat = rlmm.queryGraph.getNodeAccess().getLatitude(e.getBaseNode());
                    double n1lon = rlmm.queryGraph.getNodeAccess().getLongitude(e.getBaseNode());
                    double n2lat = rlmm.queryGraph.getNodeAccess().getLatitude(e.getAdjNode());
                    double n2lon = rlmm.queryGraph.getNodeAccess().getLongitude(e.getAdjNode());
                    double dis = this.calcNormalizedEdgeDistance(o.getPoint().lat, o.getPoint().lon,n1lat,n1lon,n2lat,n2lon);
                    minDis = min(minDis,rlmm.distanceCalc.calcDenormalizedDist(dis));
                }
                ObjectMapper mapper = new ObjectMapper();
                List<Double> ns = new ArrayList<>();
                ns.add(minDis);
                rlInfo rlreturn;
                int reward = action == 0?0:-1;
                isKey = action != 0;
                if(i==dstPos-1){
                    rlreturn = new rlInfo(ns,reward,true);
                }else{
                    rlreturn = new rlInfo(ns,reward,false);
                }
                out.println(mapper.writeValueAsString(rlreturn)); // 写入套接字
                if(i==dstPos-1){
                    echoSocket.close();
                }
                if(i!=dstPos-1){
                    String info = in.readLine(); // 读取服务器返回的内容
                    action = parseInt(info);
                }
                if(isKey){
                    State last = result.get(result.size()-1).state;
                    SequenceState<State, Observation, Path> finalDst1 = dst;
                    State state = timeSteps.get(i).candidates.stream().min(Comparator.comparing((s)-> {
                        double finalPathLen;
                        Path toLast = createRouter().calcPath(last.getSnap().getClosestNode(),s.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                        Path toDst = createRouter().calcPath(s.getSnap().getClosestNode(), finalDst1.state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                        if(toLast.isFound() && toDst.isFound()){
                            finalPathLen = toLast.getDistance() + toDst.getDistance();
                        }else{
                            finalPathLen = Double.MAX_VALUE;
                        }
                        return finalPathLen;
                    })).orElse(null);
                    Path pa = createRouter().calcPath(last.getSnap().getClosestNode(),state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                    Path pb = createRouter().calcPath(state.getSnap().getClosestNode(), finalDst1.state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                    result.add(new SequenceState<>(state,timeSteps.get(i).observation,pa));
                    tempPath = pb;
                }
            }
            result.add(new SequenceState<>(dst.state,dst.observation,tempPath));
            long t3 = System.currentTimeMillis();
            System.out.println(t3-t2);
        } catch (UnknownHostException e) {
            System.err.println("Don't know about host " + hostName);
            System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to " +
                    hostName);
            System.exit(1);
        }
        return result;
    }
}
