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
import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.LMProfile;
import com.graphhopper.config.Profile;
import com.graphhopper.matching.entities.*;
import com.graphhopper.matching.matchAlgo.*;
import com.graphhopper.matching.preprocessAlgo.BasePreProcess;
import com.graphhopper.matching.preprocessAlgo.CutHeadPreProcess;
import com.graphhopper.matching.preprocessAlgo.NormalPreProcess;
import com.graphhopper.matching.preprocessAlgo.StrictPreProcess;
import com.graphhopper.routing.*;
import com.graphhopper.routing.lm.LMApproximator;
import com.graphhopper.routing.lm.LandmarkStorage;
import com.graphhopper.routing.lm.PrepareLandmarks;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.*;
import java.util.stream.Collectors;

import static java.lang.Integer.parseInt;
import static java.lang.Math.max;
import static java.lang.Math.min;

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
public class RLMM {

    public final Logger logger = LoggerFactory.getLogger(getClass());

    public final Graph graph;
    public final PrepareLandmarks landmarks;
    public final LocationIndexTree locationIndex;
    public double measurementErrorSigma = 50.0;
    public double transitionProbabilityBeta = 2.0;
    public final int maxVisitedNodes;
    public final DistanceCalc distanceCalc = new DistancePlaneProjection();
    public final Weighting weighting;
    public QueryGraph queryGraph;
    public int maxKeypointNum = 999;
    public double kpDisThreshold = 50.0;
    public int kpConnumThreshold = 3;
    public int matchAlgoIndex = 0;
    public RLMM(GraphHopper graphHopper, PMap hints) {
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

    public void setMaxKeypointNum(int maxKeypointNum){
        this.maxKeypointNum = maxKeypointNum;
    }
    public void setKpDisThreshold(double kpDisThreshold){
        this.kpDisThreshold = kpDisThreshold;
    }
    public void setKpConnumThreshold(int kpConnumThreshold){
        this.kpConnumThreshold = kpConnumThreshold;
    }
    public void setMatchAlgoIndex(int matchAlgoIndex){
        this.matchAlgoIndex = matchAlgoIndex;
    }
    public MatchResult match(List<Observation> observations) {
        BaseMatch[] matchAlgorithms = {
                new HMMMatch(this),
                new SPMatch(this),
                new KPMatch(this),
                new KPADMatch(this),
                new RLMatch(this),
                new ADRLMatch(this)
        };

        int index = this.matchAlgoIndex;
        if (index < 0 || index >= matchAlgorithms.length) {
            index = 0; // Default index
        }
        BaseMatch matchAlgo = matchAlgorithms[index];
//        BasePreProcess preprocessAlgo = new NormalPreProcess(this);
//        BasePreProcess preprocessAlgo = new StrictPreProcess(this);
        BasePreProcess preprocessAlgo = new CutHeadPreProcess(this);
        List<ObservationWithCandidateStates> timeSteps = preprocessAlgo.preprocess(observations);
        List<SequenceState<State, Observation, Path>> seq = matchAlgo.match(timeSteps);


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

    public String getSnappedCandidates(Collection<State> candidates) {
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