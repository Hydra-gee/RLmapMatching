package com.graphhopper.matching.preprocessAlgo;

import com.graphhopper.matching.RLMM;
import com.graphhopper.matching.entities.Observation;
import com.graphhopper.matching.entities.ObservationWithCandidateStates;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.storage.index.Snap;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class CutHeadPreProcess extends BasePreProcess{
    public CutHeadPreProcess(RLMM rlmm) {
        super(rlmm);
    }

    @Override
    public List<ObservationWithCandidateStates> preprocess(List<Observation> observations) {
        List<Observation> filteredObservations = filterObservations(observations);
        //List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> rlmm.locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(rlmm.weighting.getFlagEncoder()), rlmm.measurementErrorSigma).stream().filter(s->s.getQueryDistance()<=rlmm.measurementErrorSigma).collect(Collectors.toList()))
        //        .collect(Collectors.toList());
        List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> rlmm.locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(rlmm.weighting.getFlagEncoder()), rlmm.measurementErrorSigma).stream().filter(s-> s.getQueryDistance() <= rlmm.measurementErrorSigma).collect(Collectors.toList()))
                .collect(Collectors.toList());
        rlmm.queryGraph = QueryGraph.create(rlmm.graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());
        return createTimeSteps(filteredObservations, splitsPerObservation).stream().filter(t-> !t.candidates.isEmpty()).collect(Collectors.toList());
    }

}
