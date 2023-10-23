/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.matching;

import com.bmw.hmm.SequenceState;
import com.bmw.hmm.Transition;
import com.bmw.hmm.ViterbiAlgorithm;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.LMProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.*;
import com.graphhopper.routing.lm.LMApproximator;
import com.graphhopper.routing.lm.LandmarkStorage;
import com.graphhopper.routing.lm.PrepareLandmarks;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint3D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

import static java.lang.System.currentTimeMillis;

/**
 * This class matches real world GPX entries to the digital road network stored
 * in GraphHopper. The Viterbi algorithm is used to compute the most likely
 * sequence of map matching candidates. The Viterbi algorithm takes into account
 * the distance between GPX entries and map matching candidates as well as the
 * routing distances between consecutive map matching candidates.
 * <p>
 * <p>
 * See http://en.wikipedia.org/wiki/Map_matching and Newson, Paul, and John
 * Krumm. "Hidden Markov map matching through noise and sparseness." Proceedings
 * of the 17th ACM SIGSPATIAL International Conference on Advances in Geographic
 * Information Systems. ACM, 2009.
 *
 * @author Peter Karich
 * @author Michael Zilske
 * @author Stefan Holder
 * @author kodonnell
 */
public class MapMatching {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    private final Graph graph;
    private final PrepareLandmarks landmarks;
    private final LocationIndexTree locationIndex;
    private double measurementErrorSigma = 50.0;
    private double transitionProbabilityBeta = 2.0;
    private final int maxVisitedNodes;
    private final DistanceCalc distanceCalc = new DistancePlaneProjection();
    private final Weighting weighting;
    private QueryGraph queryGraph;

    public MapMatching(GraphHopper graphHopper, PMap hints) {
        this.locationIndex = (LocationIndexTree) graphHopper.getLocationIndex();

        if (hints.has("vehicle"))
            throw new IllegalArgumentException("MapMatching hints may no longer contain a vehicle, use the profile parameter instead, see core/#1958");
        if (hints.has("weighting"))
            throw new IllegalArgumentException("MapMatching hints may no longer contain a weighting, use the profile parameter instead, see core/#1958");

        if (graphHopper.getProfiles().isEmpty()) {
            throw new IllegalArgumentException("No profiles found, you need to configure at least one profile to use map matching");
        }
        if (!hints.has("profile")) {
            throw new IllegalArgumentException("You need to specify a profile to perform map matching");
        }
        String profileStr = hints.getString("profile", "");
        Profile profile = graphHopper.getProfile(profileStr);
        if (profile == null) {
            List<Profile> profiles = graphHopper.getProfiles();
            List<String> profileNames = new ArrayList<>(profiles.size());
            for (Profile p : profiles) {
                profileNames.add(p.getName());
            }
            throw new IllegalArgumentException("Could not find profile '" + profileStr + "', choose one of: " + profileNames);
        }

        boolean disableLM = hints.getBool(Parameters.Landmark.DISABLE, false);
        if (graphHopper.getLMPreparationHandler().isEnabled() && disableLM && !graphHopper.getRouterConfig().isLMDisablingAllowed())
            throw new IllegalArgumentException("Disabling LM is not allowed");

        boolean disableCH = hints.getBool(Parameters.CH.DISABLE, false);
        if (graphHopper.getCHPreparationHandler().isEnabled() && disableCH && !graphHopper.getRouterConfig().isCHDisablingAllowed())
            throw new IllegalArgumentException("Disabling CH is not allowed");

        // see map-matching/#177: both ch.disable and lm.disable can be used to force Dijkstra which is the better
        // (=faster) choice when the observations are close to each other
        boolean useDijkstra = disableLM || disableCH;

        if (graphHopper.getLMPreparationHandler().isEnabled() && !useDijkstra) {
            // using LM because u-turn prevention does not work properly with (node-based) CH
            List<String> lmProfileNames = new ArrayList<>();
            PrepareLandmarks lmPreparation = null;
            for (LMProfile lmProfile : graphHopper.getLMPreparationHandler().getLMProfiles()) {
                lmProfileNames.add(lmProfile.getProfile());
                if (lmProfile.getProfile().equals(profile.getName())) {
                    lmPreparation = graphHopper.getLMPreparationHandler().getPreparation(
                            lmProfile.usesOtherPreparation() ? lmProfile.getPreparationProfile() : lmProfile.getProfile()
                    );
                }
            }
            if (lmPreparation == null) {
                throw new IllegalArgumentException("Cannot find LM preparation for the requested profile: '" + profile.getName() + "'" +
                        "\nYou can try disabling LM using " + Parameters.Landmark.DISABLE + "=true" +
                        "\navailable LM profiles: " + lmProfileNames);
            }
            landmarks = lmPreparation;
        } else {
            landmarks = null;
        }
        graph = graphHopper.getGraphHopperStorage();
        weighting = graphHopper.createWeighting(profile, hints);
        this.maxVisitedNodes = hints.getInt(Parameters.Routing.MAX_VISITED_NODES, Integer.MAX_VALUE);
    }

    /**
     * Beta parameter of the exponential distribution for modeling transition
     * probabilities.
     */
    public void setTransitionProbabilityBeta(double transitionProbabilityBeta) {
        this.transitionProbabilityBeta = transitionProbabilityBeta;
    }

    /**
     * Standard deviation of the normal distribution [m] used for modeling the
     * GPS error.
     */
    public void setMeasurementErrorSigma(double measurementErrorSigma) {
        this.measurementErrorSigma = measurementErrorSigma;
    }

    public MatchResult match(List<Observation> observations) {
        List<Observation> filteredObservations = filterObservations(observations);

        // Snap observations to links. Generates multiple candidate snaps per observation.
        // In the next step, we will turn them into splits, but we already call them splits now
        // because they are modified in place.
        List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()), measurementErrorSigma))
                .collect(Collectors.toList());
        // Create the query graph, containing split edges so that all the places where an observation might have happened
        // are a node. This modifies the Snap objects and puts the new node numbers into them.
        queryGraph = QueryGraph.create(graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));

        // Due to how LocationIndex/QueryGraph is implemented, we can get duplicates when a point is snapped
        // directly to a tower node instead of creating a split / virtual node. No problem, but we still filter
        // out the duplicates for performance reasons.
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());

        // Creates candidates from the Snaps of all observations (a candidate is basically a
        // Snap + direction).
        List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, splitsPerObservation);
//        timeSteps = timeSteps.stream().filter(s->s.candidates.iterator().next().getSnap().getQueryDistance()<=measurementErrorSigma).collect(Collectors.toList());
        // Compute the most likely sequence of map matching candidates:
        List<SequenceState<State, Observation, Path>> seq = computeViterbiSequence(timeSteps);

        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());

        MatchResult result = new MatchResult(prepareEdgeMatches(seq));
        result.setMergedPath(new MapMatchedPath(queryGraph, weighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> s.transitionDescriptor.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(weighting);
        return result;
    }
    public MatchResult SPmatch(List<Observation> observations) {
        List<Observation> filteredObservations = filterObservations(observations);
//        List<Observation> filteredObservations = observations;
                List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()), measurementErrorSigma))
                .collect(Collectors.toList());
        queryGraph = QueryGraph.create(graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());
        List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, splitsPerObservation);
//        List<SequenceState<State, Observation, Path>> seq = computeSPDynamic(timeSteps);
        List<SequenceState<State, Observation, Path>> seq = computeKeypointSequence(timeSteps);

        //generate Result
        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());
        MatchResult result = new MatchResult(prepareEdgeMatches(seq));
        //设置观测点和候选点坐标
        List<PointCoordinate> pointCoordinates = new ArrayList<>();
        for(SequenceState<State, Observation, com.graphhopper.routing.Path> s:seq){
            pointCoordinates.add(new PointCoordinate(s.observation.getPoint().lat, s.observation.getPoint().lon,
                    s.state.getSnap().getSnappedPoint().getLat(),s.state.getSnap().getSnappedPoint().getLon()));
        }
        result.setPointCoordinates(pointCoordinates);
        result.setMergedPath(new MapMatchedPath(queryGraph, weighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> s.transitionDescriptor.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(weighting);
        return result;
    }

    private List<SequenceState<State, Observation, Path>> computeSPDynamic(List<ObservationWithCandidateStates> timeSteps) {
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
                        Path path = createRouter().calcPath(lastCandidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
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

    //已提前进行降采样（密集点过滤）
    private List<SequenceState<State, Observation, Path>> computeKeypointSequence(List<ObservationWithCandidateStates> timeSteps) {
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
                        Path p = createRouter().calcPath(oriCandidates.getSnap().getClosestNode(), dstCandidates.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
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
//        //获取第一个有效timeStep对应的状态点
//        for(int i = 0;i<timeSteps.size();i++){
//            ObservationWithCandidateStates timeStep = timeSteps.get(i);
//            State c = timeStep.candidates.stream().min(Comparator.comparing(s->s.getSnap().getQueryDistance())).orElse(null);
//            //若没有候选点说明该点无效
//            if(c!=null){
//                ori = new SequenceState<>(c, timeStep.observation, null);
//                oriPos = i;
//                result.add(ori);
//                break;
//            }
//        }
//
//        //获取最后一个timeStep对应的状态点
//        for(int i = timeSteps.size()-1;i>=0;i--){
//            ObservationWithCandidateStates timeStep = timeSteps.get(i);
//            State c = timeStep.candidates.stream().min(Comparator.comparing(s->s.getSnap().getQueryDistance())).orElse(null);
//            //若没有候选点说明该点无效
//            if(c!=null){
//                dst = new SequenceState<>(c, timeStep.observation, null);
//                dstPos = i;
//                break;
//            }
//        }
//        GHPoint3D oriState = ori.state.getSnap().getSnappedPoint();
//        GHPoint3D dstState = dst.state.getSnap().getSnappedPoint();
//        tempPath = createRouter().calcPath(ori.state.getSnap().getClosestNode(), dst.state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
        tempPath = bestTempPath;
        List<Integer> tempKPIndexs = new ArrayList<>();
        for(int i=oriPos+1;i<dstPos;i++){
//            if(result.size()>1)break;
            double keyThreshold = 10;
            //计算到临时路径距离
            ObservationWithCandidateStates timeStep = timeSteps.get(i);
            Observation o = timeStep.observation;
            List<EdgeIteratorState> allEdges = result.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());
            allEdges.addAll(tempPath.calcEdges());
            List<Snap> closePoints = locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, new SpecificEdgeFilter(allEdges), keyThreshold);
//            List<Snap> closePoints = locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(weighting.getFlagEncoder()), keyThreshold);
            closePoints = closePoints.stream().filter(s->s.getQueryDistance()<keyThreshold).collect(Collectors.toList());
            boolean isKey = true;
            for(EdgeIteratorState e:allEdges){
                double n1lat = queryGraph.getNodeAccess().getLatitude(e.getBaseNode());
                double n1lon = queryGraph.getNodeAccess().getLongitude(e.getBaseNode());
                double n2lat = queryGraph.getNodeAccess().getLatitude(e.getAdjNode());
                double n2lon = queryGraph.getNodeAccess().getLongitude(e.getAdjNode());
//                double dis = distanceCalc.calcNormalizedEdgeDistance(o.getPoint().lat, o.getPoint().lon,30,40,30.000000000000001,40.000000000000001);
                double dis = distanceCalc.calcNormalizedEdgeDistance(o.getPoint().lat, o.getPoint().lon,n1lat,n1lon,n2lat,n2lon);
                dis = distanceCalc.calcDenormalizedDist(dis);
                if (dis < keyThreshold){
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
            if(tempKPIndexs.size()>=3){
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



//        for(int i=oriPos+1;i<=dstPos;i++){
//            ObservationWithCandidateStates timeStep = timeSteps.get(i);
//            Observation o = timeStep.observation;
//            List<EdgeIteratorState> allEdges = result.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());
//            allEdges.addAll(tempPath.calcEdges());
//            List<Snap> closePoints = locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, new SpecificEdgeFilter(allEdges), 200);
//            closePoints = closePoints.stream().filter(s->s.getQueryDistance()<200).collect(Collectors.toList());
//            //检测到已匹配路径距离是否超过阈值，若未超过阈值或已到达终点，关键点序列中断
//            if(closePoints.isEmpty() && i!=dstPos){
//                //将该点添加到临时关键点列表
//                SequenceState<State, Observation, Path> last;
//                if(!tempKeyPoints.isEmpty()){
//                    last = tempKeyPoints.get(tempKeyPoints.size()-1);
//                }else{
//                    last = result.get(result.size()-1);
//                }
//                State state = null;
//                Path path = null;
//                SequenceState<State, Observation, Path> finalDst = dst;
//                //从关键点对应的候选点中，选出“到前一状态点距离”+“到终点距离”最小的一个
//                state = timeStep.candidates.stream().min(Comparator.comparing((s)-> {
//                    double finalPathLen;
//                    Path toLast = createRouter().calcPath(last.state.getSnap().getClosestNode(),s.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
//                    Path toDst = createRouter().calcPath(s.getSnap().getClosestNode(), finalDst.state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
//                    if(toLast.isFound() && toDst.isFound()){
//                        finalPathLen = toLast.getDistance() + toDst.getDistance();
//                    }else{
//                        finalPathLen = Double.MAX_VALUE;
//                    }
//                    return finalPathLen;
//                })).orElse(null);
//                if(state==null)continue;
//                path = createRouter().calcPath(last.state.getSnap().getClosestNode(), state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
//                tempKeyPoints.add(new SequenceState<>(state, timeStep.observation, path));
//            }else{
//                //检测临时关键点数量是否达到阈值
//                if(tempKeyPoints.size() >= 5){
//                    //达到阈值，将临时关键点全部转为正式关键点
//                    result.addAll(tempKeyPoints);
//                    State lastState = result.get(result.size()-1).state;
//                    tempPath = createRouter().calcPath(lastState.getSnap().getClosestNode(), dst.state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
//                }
//                //清空临时关键点
//                tempKeyPoints.clear();
//                if(i==dstPos){
//                    result.add(new SequenceState<>(dst.state,dst.observation,tempPath));
//                }
//            }
//            if(tempKeyPoints.size() >= 5){
//                //达到阈值，将临时关键点全部转为正式关键点
//                result.addAll(tempKeyPoints);
//                State lastState = result.get(result.size()-1).state;
//                tempPath = createRouter().calcPath(lastState.getSnap().getClosestNode(), dst.state.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
//                tempKeyPoints.clear();
//            }
//        }
        long t3 = System.currentTimeMillis();
        System.out.println(t3-t2);
        return result;
    }
    /**
     * Filters observations to only those which will be used for map matching (i.e. those which
     * are separated by at least 2 * measurementErrorSigma
     */
    private List<Observation> filterObservations(List<Observation> observations) {
        List<Observation> filtered = new ArrayList<>();
        Observation prevEntry = null;
        int last = observations.size() - 1;
        for (int i = 0; i <= last; i++) {
            Observation observation = observations.get(i);
            if (i == 0 || i == last || distanceCalc.calcDist(
                    prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                    observation.getPoint().getLat(), observation.getPoint().getLon()) > 2 * measurementErrorSigma) {
                filtered.add(observation);
                prevEntry = observation;
            } else {
                logger.debug("Filter out observation: {}", i + 1);
            }
        }
        return filtered;
    }

    private Collection<Snap> deduplicate(Collection<Snap> splits) {
        // Only keep one split per node number. Let's say the last one.
        Map<Integer, Snap> splitsByNodeNumber = splits.stream().collect(Collectors.toMap(Snap::getClosestNode, s -> s, (s1, s2) -> s2));
        return splitsByNodeNumber.values();
    }

    /**
     * Creates TimeSteps with candidates for the GPX entries but does not create emission or
     * transition probabilities. Creates directed candidates for virtual nodes and undirected
     * candidates for real nodes.
     */
    private List<ObservationWithCandidateStates> createTimeSteps(List<Observation> filteredObservations, List<Collection<Snap>> splitsPerObservation) {
        if (splitsPerObservation.size() != filteredObservations.size()) {
            throw new IllegalArgumentException(
                    "filteredGPXEntries and queriesPerEntry must have same size.");
        }

        final List<ObservationWithCandidateStates> timeSteps = new ArrayList<>();
        for (int i = 0; i < filteredObservations.size(); i++) {
            Observation observation = filteredObservations.get(i);
            Collection<Snap> splits = splitsPerObservation.get(i);
            List<State> candidates = new ArrayList<>();
            for (Snap split : splits) {
                if (queryGraph.isVirtualNode(split.getClosestNode())) {
                    List<VirtualEdgeIteratorState> virtualEdges = new ArrayList<>();
                    EdgeIterator iter = queryGraph.createEdgeExplorer().setBaseNode(split.getClosestNode());
                    while (iter.next()) {
                        if (!queryGraph.isVirtualEdge(iter.getEdge())) {
                            throw new RuntimeException("Virtual nodes must only have virtual edges "
                                    + "to adjacent nodes.");
                        }
                        virtualEdges.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
                    }
                    if (virtualEdges.size() != 2) {
                        throw new RuntimeException("Each virtual node must have exactly 2 "
                                + "virtual edges (reverse virtual edges are not returned by the "
                                + "EdgeIterator");
                    }

                    // Create a directed candidate for each of the two possible directions through
                    // the virtual node. We need to add candidates for both directions because
                    // we don't know yet which is the correct one. This will be figured
                    // out by the Viterbi algorithm.
                    candidates.add(new State(observation, split, virtualEdges.get(0), virtualEdges.get(1)));
                    candidates.add(new State(observation, split, virtualEdges.get(1), virtualEdges.get(0)));
                } else {
                    // Create an undirected candidate for the real node.
                    candidates.add(new State(observation, split));
                }
            }

            timeSteps.add(new ObservationWithCandidateStates(observation, candidates));
        }
        return timeSteps;
    }

    /**
     * Computes the most likely state sequence for the observations.
     */
    private List<SequenceState<State, Observation, Path>> computeViterbiSequence(List<ObservationWithCandidateStates> timeSteps) {
        final HmmProbabilities probabilities = new HmmProbabilities(measurementErrorSigma, transitionProbabilityBeta);
        final ViterbiAlgorithm<State, Observation, Path> viterbi = new ViterbiAlgorithm<>();

        int timeStepCounter = 0;
        ObservationWithCandidateStates prevTimeStep = null;
        for (ObservationWithCandidateStates timeStep : timeSteps) {
            final Map<State, Double> emissionLogProbabilities = new HashMap<>();
            Map<Transition<State>, Double> transitionLogProbabilities = new HashMap<>();
            Map<Transition<State>, Path> roadPaths = new HashMap<>();
            for (State candidate : timeStep.candidates) {
                // distance from observation to road in meters
                final double distance = candidate.getSnap().getQueryDistance();
                emissionLogProbabilities.put(candidate, probabilities.emissionLogProbability(distance));
            }

            if (prevTimeStep == null) {
                viterbi.startWithInitialObservation(timeStep.observation, timeStep.candidates, emissionLogProbabilities);
            } else {
                final double linearDistance = distanceCalc.calcDist(prevTimeStep.observation.getPoint().lat,
                        prevTimeStep.observation.getPoint().lon, timeStep.observation.getPoint().lat, timeStep.observation.getPoint().lon);

                for (State from : prevTimeStep.candidates) {
                    for (State to : timeStep.candidates) {
                        final Path path = createRouter().calcPath(from.getSnap().getClosestNode(), to.getSnap().getClosestNode(), from.isOnDirectedEdge() ? from.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE, to.isOnDirectedEdge() ? to.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE);
                        if (path.isFound()) {
                            double transitionLogProbability = probabilities.transitionLogProbability(path.getDistance(), linearDistance);
                            Transition<State> transition = new Transition<>(from, to);
                            roadPaths.put(transition, path);
                            transitionLogProbabilities.put(transition, transitionLogProbability);
                        }
                    }
                }
                viterbi.nextStep(timeStep.observation, timeStep.candidates,
                        emissionLogProbabilities, transitionLogProbabilities,
                        roadPaths);
            }
            if (viterbi.isBroken()) {
                fail(timeStepCounter, prevTimeStep, timeStep);
            }

            timeStepCounter++;
            prevTimeStep = timeStep;
        }

        return viterbi.computeMostLikelySequence();
    }

    private void fail(int timeStepCounter, ObservationWithCandidateStates prevTimeStep, ObservationWithCandidateStates timeStep) {
        String likelyReasonStr = "";
        if (prevTimeStep != null) {
            double dist = distanceCalc.calcDist(prevTimeStep.observation.getPoint().lat, prevTimeStep.observation.getPoint().lon, timeStep.observation.getPoint().lat, timeStep.observation.getPoint().lon);
            if (dist > 2000) {
                likelyReasonStr = "Too long distance to previous measurement? "
                        + Math.round(dist) + "m, ";
            }
        }

        throw new IllegalArgumentException("Sequence is broken for submitted track at time step "
                + timeStepCounter + ". "
                + likelyReasonStr + "observation:" + timeStep.observation + ", "
                + timeStep.candidates.size() + " candidates: "
                + getSnappedCandidates(timeStep.candidates)
                + ". If a match is expected consider increasing max_visited_nodes.");
    }

    private BidirRoutingAlgorithm createRouter() {
        BidirRoutingAlgorithm router;
        if (landmarks != null) {
            AStarBidirection algo = new AStarBidirection(queryGraph, weighting, TraversalMode.EDGE_BASED) {
                @Override
                protected void initCollections(int size) {
                    super.initCollections(50);
                }
            };
            LandmarkStorage lms = landmarks.getLandmarkStorage();
            int activeLM = Math.min(8, lms.getLandmarkCount());
            algo.setApproximation(LMApproximator.forLandmarks(queryGraph, lms, activeLM));
            algo.setMaxVisitedNodes(maxVisitedNodes);
            router = algo;
        } else {
            router = new DijkstraBidirectionRef(queryGraph, weighting, TraversalMode.EDGE_BASED) {
                @Override
                protected void initCollections(int size) {
                    super.initCollections(50);
                }
            };
//            router.setMaxVisitedNodes(maxVisitedNodes);
            router.setMaxVisitedNodes(Integer.MAX_VALUE);
        }
        return router;
    }

    private List<EdgeMatch> prepareEdgeMatches(List<SequenceState<State, Observation, Path>> seq) {
        // This creates a list of directed edges (EdgeIteratorState instances turned the right way),
        // each associated with 0 or more of the observations.
        // These directed edges are edges of the real street graph, where nodes are intersections.
        // So in _this_ representation, the path that you get when you just look at the edges goes from
        // an intersection to an intersection.

        // Implementation note: We have to look at both states _and_ transitions, since we can have e.g. just one state,
        // or two states with a transition that is an empty path (observations snapped to the same node in the query graph),
        // but these states still happen on an edge, and for this representation, we want to have that edge.
        // (Whereas in the ResponsePath representation, we would just see an empty path.)

        // Note that the result can be empty, even when the input is not. Observations can be on nodes as well as on
        // edges, and when all observations are on the same node, we get no edge at all.
        // But apart from that corner case, all observations that go in here are also in the result.

        // (Consider totally forbidding candidate states to be snapped to a point, and make them all be on directed
        // edges, then that corner case goes away.)
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        List<State> states = new ArrayList<>();
        EdgeIteratorState currentDirectedRealEdge = null;
        for (SequenceState<State, Observation, Path> transitionAndState : seq) {
            // transition (except before the first state)
            if (transitionAndState.transitionDescriptor != null) {
                for (EdgeIteratorState edge : transitionAndState.transitionDescriptor.calcEdges()) {
                    EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(edge);
                    if (currentDirectedRealEdge != null) {
                        if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                            edgeMatches.add(edgeMatch);
                            states = new ArrayList<>();
                        }
                    }
                    currentDirectedRealEdge = newDirectedRealEdge;
                }
            }
            // state
            if (transitionAndState.state.isOnDirectedEdge()) { // as opposed to on a node
                EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(transitionAndState.state.getOutgoingVirtualEdge());
                if (currentDirectedRealEdge != null) {
                    if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                        EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                        edgeMatches.add(edgeMatch);
                        states = new ArrayList<>();
                    }
                }
                currentDirectedRealEdge = newDirectedRealEdge;
            }
            states.add(transitionAndState.state);
        }
        if (currentDirectedRealEdge != null) {
            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
            edgeMatches.add(edgeMatch);
        }
        return edgeMatches;
    }

    private double gpxLength(List<Observation> gpxList) {
        if (gpxList.isEmpty()) {
            return 0;
        } else {
            double gpxLength = 0;
            Observation prevEntry = gpxList.get(0);
            for (int i = 1; i < gpxList.size(); i++) {
                Observation entry = gpxList.get(i);
                gpxLength += distanceCalc.calcDist(prevEntry.getPoint().lat, prevEntry.getPoint().lon, entry.getPoint().lat, entry.getPoint().lon);
                prevEntry = entry;
            }
            return gpxLength;
        }
    }

    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }

    private EdgeIteratorState resolveToRealEdge(EdgeIteratorState edgeIteratorState) {
        if (queryGraph.isVirtualNode(edgeIteratorState.getBaseNode()) || queryGraph.isVirtualNode(edgeIteratorState.getAdjNode())) {
            return graph.getEdgeIteratorStateForKey(((VirtualEdgeIteratorState) edgeIteratorState).getOriginalEdgeKey());
        } else {
            return edgeIteratorState;
        }
    }

    private String getSnappedCandidates(Collection<State> candidates) {
        String str = "";
        for (State gpxe : candidates) {
            if (!str.isEmpty()) {
                str += ", ";
            }
            str += "distance: " + gpxe.getSnap().getQueryDistance() + " to "
                    + gpxe.getSnap().getSnappedPoint();
        }
        return "[" + str + "]";
    }

    private static class MapMatchedPath extends Path {
        MapMatchedPath(Graph graph, Weighting weighting, List<EdgeIteratorState> edges) {
            super(graph);
            int prevEdge = EdgeIterator.NO_EDGE;
            for (EdgeIteratorState edge : edges) {
                addDistance(edge.getDistance());
                addTime(GHUtility.calcMillisWithTurnMillis(weighting, edge, false, prevEdge));
                addEdge(edge.getEdge());
                prevEdge = edge.getEdge();
            }
            if (edges.isEmpty()) {
                setFound(false);
            } else {
                setFromNode(edges.get(0).getBaseNode());
                setFound(true);
            }
        }
    }

}