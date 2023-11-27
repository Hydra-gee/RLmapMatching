package com.graphhopper.matching.matchAlgo;

import com.bmw.hmm.SequenceState;
import com.graphhopper.matching.RLMM;
import com.graphhopper.matching.entities.Observation;
import com.graphhopper.matching.entities.ObservationWithCandidateStates;
import com.graphhopper.matching.entities.State;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class KPMatch extends BaseMatch{
    public KPMatch(RLMM rlmm) {
        super(rlmm);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        System.out.println("Using KPMatch");
        long t1 = System.currentTimeMillis();
        int oriPos = 0,dstPos = timeSteps.size()-1;
        SequenceState<State, Observation, Path> ori = null;
        SequenceState<State, Observation, Path> dst = null;
        List<SequenceState<State, Observation, Path>> tempKeyPoints = new ArrayList<>();
        Path tempPath;
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
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
        System.out.println(t2-t1);
        tempPath = bestTempPath;
        List<Integer> tempKPIndexs = new ArrayList<>();
        for(int i=oriPos+1;i<dstPos;i++){
            if(result.size()>rlmm.maxKeypointNum)break;
            //计算到临时路径距离
            ObservationWithCandidateStates timeStep = timeSteps.get(i);
            Observation o = timeStep.observation;
            List<EdgeIteratorState> allEdges = result.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());
            allEdges.addAll(tempPath.calcEdges());
            boolean isKey = true;
            for(EdgeIteratorState e:allEdges){
                double n1lat = rlmm.queryGraph.getNodeAccess().getLatitude(e.getBaseNode());
                double n1lon = rlmm.queryGraph.getNodeAccess().getLongitude(e.getBaseNode());
                double n2lat = rlmm.queryGraph.getNodeAccess().getLatitude(e.getAdjNode());
                double n2lon = rlmm.queryGraph.getNodeAccess().getLongitude(e.getAdjNode());
                double dis = calcNormalizedEdgeDistance(o.getPoint().lat, o.getPoint().lon,n1lat,n1lon,n2lat,n2lon);
                dis = rlmm.distanceCalc.calcDenormalizedDist(dis);
                if (dis < rlmm.kpDisThreshold){
                    isKey = false;
                    break;
                }
            }

            //超过阈值，设为临时关键点
//            if(closePoints.isEmpty()){
            if(isKey){
                tempKPIndexs.add(i);
            }else{
                tempKPIndexs.clear();
            }
            if(tempKPIndexs.size()>=rlmm.kpConnumThreshold){
                State last = result.get(result.size()-1).state;
                SequenceState<State, Observation, Path> finalDst1 = dst;
                State state = timeSteps.get(tempKPIndexs.get(0)).candidates.stream().min(Comparator.comparing((s)-> {
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
                result.add(new SequenceState<>(state,timeSteps.get(tempKPIndexs.get(0)).observation,pa));
                tempPath = pb;
                tempKPIndexs.clear();
            }
        }
        result.add(new SequenceState<>(dst.state,dst.observation,tempPath));
        long t3 = System.currentTimeMillis();
        System.out.println(t3-t2);
        return result;
    }
}
