import QtQuick 2.0
import QtLocation 5.6
import QtPositioning 5.6

import "../scripts/helper.js" as Helper

MapPolyline {
    line.width: 3
    line.color: "#81c784"
    opacity: 1.0
    z: map.z + 5

    property string type: "polyline"

    signal end

    Component.onCompleted: {
        Helper.init_lib(QtPositioning)
    }

    //TODO
    function begin_edit(){console.log("TODO")}
    function click_mod_handler(){console.log("TODO")}
    function pos_changed_mod_handler(){console.log("TODO")}
    function confirm_edit(params){console.log("TODO")}
    function discard_edit(){console.log("TODO")}
    function check_safe(box){console.log("TODO")}

    function click_handler(mouse){
        if (mouse.button & Qt.LeftButton) {
            var p = Qt.point(mouse.x, mouse.y)
            var wp = map.toCoordinate(p)
            if (pathLength() === 0){
                mapMouseArea.hoverEnabled = true
                addCoordinate(wp)
                addCoordinate(wp)
                line.color = "#ffb300"
                return
            }
            else if (pathLength() > 1){
                var lastwp = coordinateAt(pathLength()-2)
                var lastp = map.fromCoordinate(lastwp)
                if(Helper.distance(p, lastp) < 8){
                    removeCoordinate(pathLength()-1)
                    line.color = "#33cc33"
                    mapMouseArea.hoverEnabled = false
                    generate_nurbs()
                    end()
                    return
                }
            }
            line.color = "#ffb300"
            addCoordinate(wp)
        }
    }

    function pos_changed_handler(mouse){
        var p = Qt.point(mouse.x, mouse.y)
        var wp = map.toCoordinate(p)
        replaceCoordinate(pathLength()-1, wp)
        if (pathLength() > 1){
            var lastwp = coordinateAt(pathLength()-2)
            var lastp = map.fromCoordinate(lastwp)
            if(Helper.distance(p, lastp) < 8){
                line.color = "#ffb300"
            } else {
                line.color = "#81c784"
            }
        } else {
            line.color = "#81c784"
        }
    }

    function generate_nurbs(){
        var centroid = Helper.coords_centroid(path)
        var lam = Helper.lat_to_m_coeff(centroid.latitude)
        var lom = Helper.lon_to_m_coeff(centroid.longitude)
        var points = Helper.points_map2euclidean(path, centroid, lam, lom)

        var nurb_l = Helper.generate_nurb_broken_line(points)
        var result = {
            centroid: [centroid.latitude, centroid.longitude],
            curves: [nurb_l]
        }
        //console.log(JSON.stringify(result))
        return result
    }

    function serialize(){
        var values = []
        for (j = 0; j < path.length; j++){
            var p_i = path[j]
            values.push({
                latitude: p_i.latitude,
                longitude: p_i.longitude
            })
        }
        return {
            name: 'PointPath',
            offset: poly_list[i].offset,
            angle: poly_list[i].angle,
            values: values
        }
    }

    function deserialize(values){
        var lat, lon
        for(var j = 0; j < values.length; j++){
            lat = values[j].latitude
            lon = values[j].longitude
            poly_cur.addCoordinate(QtPositioning.coordinate(lat,lon))
        }
    }
}
