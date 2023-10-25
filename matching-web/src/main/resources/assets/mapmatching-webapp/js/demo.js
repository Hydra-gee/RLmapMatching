var iconObject = L.icon({
    iconUrl: './img/marker-icon.png',
    shadowSize: [50, 64],
    shadowAnchor: [4, 62],
    iconAnchor: [12, 40]
});

var pointMarkers = {};
$(document).ready(function (e) {
    jQuery.support.cors = true;

    var mmMap = createMap('map-matching-map');
    var mmClient = new GraphHopperMapMatching(/*{host: "https://graphhopper.com/api/1/", key: ""}*/);
    setup(mmMap, mmClient);
});

function setup(map, mmClient) {
    // TODO fetch bbox from /info
    map.setView([50.9, 13.4], 9);
    var routeLayer = L.geoJson().addTo(map);
    routeLayer.options = {
        // use style provided by the 'properties' entry of the geojson added by addDataToRoutingLayer
        style: function (feature) {
            return feature.properties && feature.properties.style;
        }};
    $("#getmark").click(function (){
        let m = L.marker([$("#latitude-input").val(), $("#longitude-input").val()]).addTo(map);
        let r = new L.Circle([$("#latitude-input").val(), $("#longitude-input").val()], $("#range-input").val(), {
            color: 'yellow', //颜色
            fillColor: '#c6b315',
            fillOpacity: 0.3, //透明度
        }).addTo(map);
        m.on('click',()=>{
            map.removeLayer(m);
            map.removeLayer(r);
        });
    })
    function readSingleFile(e) {
        var file = e.target.files[0];
        if (!file) {
            return;
        }
        var reader = new FileReader();
        reader.onload = function (e) {
            var content = e.target.result;

            var dom = (new DOMParser()).parseFromString(content, 'text/xml');
            var pathOriginal = toGeoJSON.gpx(dom);

            routeLayer.clearLayers();
            if (pathOriginal.features[0]) {

                pathOriginal.features[0].properties = {style: {color: "black", weight: 2, opacity: 0.9}};
                routeLayer.addData(pathOriginal);
                $("#map-matching-response").text("calculate route match ...");
                $("#map-matching-error").text("");
            } else {
                $("#map-matching-error").text("Cannot display original gpx file. No trk/trkseg/trkpt elements found?");
            }

            var vehicle = $("#vehicle-input").val();
            if (!vehicle)
                vehicle = "car";

            var gpsAccuracy = $("#accuracy-input").val();
            if (!gpsAccuracy)
                gpsAccuracy = 20;
            var keyPointNum = $("#keypoint-input").val();
            if (!keyPointNum)
                keyPointNum = 999;
            var kpDisThreshold = $("#kp-dis-input").val();
            if (!kpDisThreshold)
                kpDisThreshold = 50;
            var kpConNumThreshold = $("#kp-connum-input").val();
            if (!kpConNumThreshold)
                kpConNumThreshold = 3;
            mmClient.vehicle = vehicle;
            mmClient.doRequest(content, function (json) {
                if (json.message) {
                    $("#map-matching-response").text("");
                    $("#map-matching-error").text(json.message);
                } else if (json.paths && json.paths.length > 0) {
                    var mm = json.map_matching;
                    var error = (100 * Math.abs(1 - mm.distance / mm.original_distance));
                    error = Math.floor(error * 100) / 100.0;
                    $("#map-matching-response").text("success with " + error + "% difference, "
                            + "distance " + Math.floor(mm.distance) + " vs. original distance " + Math.floor(mm.original_distance));
                    var matchedPath = json.paths[0];
                    var geojsonFeature = {
                        type: "Feature",
                        geometry: matchedPath.points,
                        properties: {style: {color: "#00cc33", weight: 6, opacity: 0.4}}
                    };
                    routeLayer.addData(geojsonFeature);
                    // for(let point of matchedPath.points.coordinates){
                    //     L.circleMarker([point[1],point[0]],{color:'blue',opacity:1}).addTo(map)
                    // }
                    console.log(matchedPath.points);
                    //添加观测点复选框
                    if(json['points']){
                        appendHtml = "";
                        for(let i in json['points']){
                            appendHtml += '<label><input id="check'+i+'" name="'+i+'" type="checkbox" value="1" />点'+i+'</label> ';
                        }
                        $("#pointCheckbox").html(appendHtml)
                        for(let i in json['points']){
                            let lat1 = json['points'][i]['obs_lat'];
                            let lon1 = json['points'][i]['obs_lon'];
                            let lat2 = json['points'][i]['can_lat'];
                            let lon2 = json['points'][i]['can_lon'];
                            //复选框状态改变时，增加或删除标记点
                            $("#check"+i).change(function() {
                                console.log(lat1,lon1,lat2,lon2);
                                if(pointMarkers[i]===undefined || pointMarkers[i]==null){
                                    pointMarkers[i] = {
                                        obsMarker:L.circleMarker([lat1,lon1],{color:'black',opacity:1}).addTo(map),
                                        canMarker:L.circleMarker([lat2,lon2],{color:'green',opacity:1}).addTo(map)
                                    }
                                }else{
                                    map.removeLayer(pointMarkers[i].obsMarker);
                                    map.removeLayer(pointMarkers[i].canMarker);
                                    pointMarkers[i] = null;
                                }
                            });
                        }
                    }


                    if (matchedPath.bbox) {
                        var minLon = matchedPath.bbox[0];
                        var minLat = matchedPath.bbox[1];
                        var maxLon = matchedPath.bbox[2];
                        var maxLat = matchedPath.bbox[3];
                        var tmpB = new L.LatLngBounds(new L.LatLng(minLat, minLon), new L.LatLng(maxLat, maxLon));
                        map.fitBounds(tmpB);
                    }
                } else {
                    $("#map-matching-error").text("unknown error");
                }
            }, {gps_accuracy: gpsAccuracy,keypoint_num:keyPointNum,kp_dis_threshold:kpDisThreshold,kp_connum_threshold:kpConNumThreshold});
        };
        reader.readAsText(file);
    }

    document.getElementById('matching-file-input').addEventListener('change', readSingleFile, false);
}

GraphHopperMapMatching = function (args) {
    this.host = "/";
    this.basePath = "match";
    this.vehicle = "car";
    this.gps_accuracy = 20;
    this.data_type = "json";
    this.max_visited_nodes = 3000;
    this.keypoint_num = 999;
    this.kp_dis_threshold = 50;
    this.kp_connum_threshold = 3;

    graphhopper.util.copyProperties(args, this);
};

GraphHopperMapMatching.prototype.doRequest = function (content, callback, reqArgs) {
    var that = this;
    var args = graphhopper.util.clone(that);
    if (reqArgs)
        args = graphhopper.util.copyProperties(reqArgs, args);

    var url = args.host + args.basePath + "?vehicle=" + args.vehicle
            + "&gps_accuracy=" + args.gps_accuracy
            + "&type=" + args.data_type
            + "&max_visited_nodes=" + args.max_visited_nodes
            + "&keypoint_num=" + args.keypoint_num
            + "&kp_dis_threshold=" + args.kp_dis_threshold
            + "&kp_connum_threshold=" + args.kp_connum_threshold;

    if (args.key)
        url += "&key=" + args.key;

    $.ajax({
        timeout: 20000,
        url: url,
        contentType: "application/xml",
        type: "POST",
        data: content
    }).done(function (json) {
        console.log(json);
        if (json.paths) {
            for (var i = 0; i < json.paths.length; i++) {
                var path = json.paths[i];
                // convert encoded polyline to geo json
                if (path.points_encoded) {
                    var tmpArray = graphhopper.util.decodePath(path.points, that.elevation);
                    path.points = {
                        "type": "LineString",
                        "coordinates": tmpArray
                    };
                }
            }
        }
        callback(json);

    }).fail(function (jqXHR) {

        if (jqXHR.responseJSON && jqXHR.responseJSON.message) {
            callback(jqXHR.responseJSON);

        } else {
            callback({
                "message": "Unknown error",
                "details": "Error for " + url
            });
        }
    });
};


function createMap(divId) {
    var osmAttr = '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors';

    var omniscale = L.tileLayer.wms('https://maps.omniscale.net/v1/mapmatching-23a1e8ea/tile', {
        layers: 'osm',
        attribution: osmAttr + ', &copy; <a href="http://maps.omniscale.com/">Omniscale</a>'
    });

    var osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: osmAttr
    });

    var map = L.map(divId, {layers: [omniscale]});
    L.control.layers({"Omniscale": omniscale,
        "OpenStreetMap": osm, }).addTo(map);
    L.control.scale().addTo(map);
    return map;
}