import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.7
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

MapComponentForm {

    id: map_component

    property var click_handler : function(){}
    property var pos_changed_handler : function(){}

    mapMouseArea.onClicked: {click_handler(mouse)}
    mapMouseArea.onPositionChanged: {pos_changed_handler(mouse)}

    property list<MapRectangle> rect_list
    property MapRectangle rect_cur

    property list<MapPolygon> poly_list
    property MapPolygon poly_cur

    property list<MapPath> path_list
    property MapPath path_cur

    property MapPolygonSecurity polysec_cur

    property Component rectComponent
    property Component polyComponent
    property Component polysecComponent
    property Component pathComponent

    property var security_defined : 0


    Component.onCompleted: {
        rectComponent = Qt.createComponent("MapRectangle.qml");
        polyComponent = Qt.createComponent("MapPolygon.qml");
        polysecComponent = Qt.createComponent("MapPolygonSecurity.qml");
        pathComponent = Qt.createComponent("MapPath.qml");
    }

    function createRect() {
         if( security_defined === 0){
             toast.show("Define Security Area First!")
             return ;
         }
         if (currentState === generalState.empty){
             currentState = generalState.rect
             if (rect_cur)
                 rect_cur.end.disconnect(endRect)
             rect_cur = rectComponent.createObject(map_component)
             rect_list.push(rect_cur)
             map.addMapItem(rect_cur)
             click_handler = rect_cur.click_handler
             pos_changed_handler = rect_cur.pos_changed_handler
             rect_cur.end.connect(endRect)
         }
    }

    function createPoly() {
        if( security_defined === 0){
            toast.show("Define Security Area First!")
            return ;
        }

        if (currentState === generalState.empty){
            currentState = generalState.poly
            if (poly_cur)
                poly_cur.end.disconnect(endPoly)
            poly_cur = polyComponent.createObject(map_component)
            poly_list.push(poly_cur)
            map.addMapItem(poly_cur)
            click_handler = poly_cur.click_handler
            pos_changed_handler = poly_cur.pos_changed_handler
            poly_cur.end.connect(endPoly)
        }
   }

    function createPolySec() {
            //TODO -> use a menu for editing the polygon
            if(security_defined === 1){
                map.removeMapItem(polysec_cur)
                security_defined = 0
            }
             if (currentState === generalState.empty){
                 currentState = generalState.polysec
                 if (polysec_cur)
                     polysec_cur.end.disconnect(endPolySec)
                 polysec_cur = polysecComponent.createObject(map_component)
                 map.addMapItem(polysec_cur)
                 click_handler = polysec_cur.click_handler
                 pos_changed_handler = polysec_cur.pos_changed_handler
                 polysec_cur.end.connect(endPolySec)
                 security_defined = 1
             }
        }

    function createPath() {
        if( security_defined === 0){
            toast.show("Define Security Area First!")
            return ;
        }

        if (currentState === generalState.empty){
            currentState = generalState.path
            if (path_cur)
                path_cur.end.disconnect(endPath)
            path_cur = pathComponent.createObject(map_component)
            path_list.push(path_cur)
            map.addMapItem(path_cur)
            click_handler = path_cur.click_handler
            pos_changed_handler = path_cur.pos_changed_handler
            path_cur.end.connect(endPath)
        }

    }

    function endRect() {
         if (currentState === generalState.rect){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
         }

    }

    function endPoly() {
         if (currentState === generalState.poly){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
         }
    }

    function endPolySec() {
         if (currentState === generalState.polysec){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
         }
    }

    function endPath(){
        if (currentState === generalState.path){
            click_handler = function(){}
            pos_changed_handler = function(){}
            currentState = generalState.empty
        }
    }


    //TODO !!
    function loadPath(file){

        var jsondata = '{"paths":[{"name":"SecurityPoly","values":[["latitude:44.403194694213305","longitude:8.932898291650417"],["latitude:44.391009258786504","longitude:8.935386641587797"],["latitude:44.39009850156113","longitude:8.951105730206649"],["latitude:44.400853206086964","longitude:8.949224294887046"],["latitude:44.403194694213305","longitude:8.932898291650417"]]}]}'
        var data
        data = JSON.parse(jsondata)

        var i
        var j

        clearAll()


        for(i = 0; i < data.paths.length; i++){
            switch(data.paths[i].name){
            case "RectPath":
                break;
            case "PolyPath":
                break;
            case "PointPath":
                break;
            case "SecurityPoly":
                polysec_cur = polysecComponent.createObject(map_component)
                map.addMapItem(polysec_cur)
                for(j = 0; j < data.paths[i].values.length-1; j++){

                    var p = Qt.point(data.paths[i].values[j][0], data.paths[i].values[j][1])
                }
                security_defined = 1
                break;

            }
        }

    }



    //TODO -> save to a file
    function savePath(){
        var i
        var j

        var l = []
        var all_paths = {};
        all_paths.paths = []

        var single_path = {};
        single_path.name = 'paths'
        single_path.values = []

        //Add all the rectangular paths
        for(i = 0; i < rect_list.length; i++){
            var single_path = {}
            single_path.name = 'RectPath'
            single_path.values = []
            for(j = 0; j < rect_list[0].pathLength(); j++){
                var p_i = rect_list[0].coordinateAt(j)
                l = []
                l.push("latitude:"+p_i.latitude)
                l.push("longitude:"+p_i.longitude)
                single_path.values.push(l)
            }
            all_paths.paths.push(single_path)
        }

        //Add all the polygonal paths
        for(i = 0; i < poly_list.length; i++){
            var single_path = {}
            single_path.name = 'PolyPath'
            single_path.values = []
            for(j = 0; j < poly_list[0].pathLength(); j++){
                var p_i = poly_list[0].coordinateAt(j)
                l = []
                l.push("latitude:"+p_i.latitude)
                l.push("longitude:"+p_i.longitude)
                single_path.values.push(l)
            }
            all_paths.paths.push(single_path)
        }

        //Add all the point paths
        for(i = 0; i < path_list.length; i++){
            var single_path = {}
            single_path.name = 'PointPath'
            single_path.values = []
            for(j = 0; j < path_list[0].pathLength(); j++){
                var p_i = path_list[0].coordinateAt(j)
                l = []
                l.push("latitude:"+p_i.latitude)
                l.push("longitude:"+p_i.longitude)
                single_path.values.push(l)
            }
            all_paths.paths.push(single_path)
        }

        if(polysec_cur){
            var single_path = {}
            single_path.name = 'SecurityPoly'
            single_path.values = []
            for(j = 0; j < polysec_cur.pathLength(); j++){
                var p_i = polysec_cur.coordinateAt(j)
                l = []
                l.push("latitude:"+p_i.latitude)
                l.push("longitude:"+p_i.longitude)
                single_path.values.push(l)

            }
            all_paths.paths.push(single_path)
        }

        console.log(JSON.stringify(all_paths))
    }

    compass.transform: [
        Rotation {
            origin.x: compass.width / 2
            origin.y: compass.height / 2
            angle: 180.0 - map.bearing
        },
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: map.tilt
            axis.x: 1
            axis.y: 0
            axis.z: 0
        }
    ]

    ulisseIcon.transform: [
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: fbkUpdater.ulisse_yaw_deg - map.bearing
        },
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: map.tilt
            axis.x: 1
            axis.y: 0
            axis.z: 0
        }
    ]

    onCenterChanged: {
        ruler.rulerTimer.restart()
    }

    onZoomLevelChanged: {
        ruler.rulerTimer.restart()
    }

    onWidthChanged: {
        ruler.rulerTimer.restart()
    }

    onHeightChanged: {
        ruler.rulerTimer.restart()
    }

    markerIcon.onCoordinateChanged: {
        mapsidebar.markerText = "%1, %2".arg(marker_coords.latitude).arg(
                    marker_coords.longitude)
    }

    Timer {
        interval: 500
        running: true
        repeat: true
        onTriggered: {
            if (ulissePath.firstRun) {
                ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                ulissePath.firstRun = false
            }
            // To reduce the line density (and avoid to overload the gui)
            // we add a new point only every 1.0 meter
            var lastCoord = ulissePath.coordinateAt(
                        ulissePath.pathLength() - 1)
            var distToNext = lastCoord.distanceTo(fbkUpdater.ulisse_pos)
            if (distToNext > 1.0) {
                ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                if (ulissePath.pathLength() > ulissePath.traceSize) {
                    ulissePath.removeCoordinate(0)
                }
            }
        }
    }

    function clearUlisseTrace() {
        ulissePath.path = []
        ulissePath.firstRun = true

        savePath()
        loadPath()
    }

    //TODO -> finish (delete also the lines/circles) and add to a button
    function clearAll(){
        //clearUlisseTrace()
        security_defined = 0
        var i
        for(i = 0; i < rect_list.length; i++){
            map.removeMapItem(rect_list[i])
        }

        for(i = 0; i < poly_list.length; i++){
            map.removeMapItem(poly_list[i])
        }

        for(i = 0; i < path_list.length; i++){
            map.removeMapItem(path_list[i])
        }

        map.removeMapItem(polysec_cur)
    }

    function drawStraightLine(ctx, p1, p2){
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(p1.x,p1.y);
        ctx.lineTo(p2.x,p2.y);
        ctx.closePath();
        ctx.stroke();
        ctx.restore();
    }
}
