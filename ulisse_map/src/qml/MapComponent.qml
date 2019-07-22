import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.7
import QtQml.Models 2.1
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
    property var actualtrack

    mapMouseArea.onClicked: {click_handler(mouse)}
    mapMouseArea.onPositionChanged: {pos_changed_handler(mouse)}

    property MapPolygon poly_obj

    property MapPolygon polysec_cur

    property Component polyComponent
    property Component polysecComponent
    property Component pathComponent
    property Component trackComponent

    //TODO -> Make relative
    property var path_file: "/home/alessio/Desktop/Prova"



    Component.onCompleted: {
        polyComponent = Qt.createComponent("MapPolygon.qml")
        pathComponent = Qt.createComponent("MapPath.qml")
        trackComponent = Qt.createComponent("ElementTrack.qml")

        polysec_cur = polyComponent.createObject(map_component)
        polysec_cur.click_handler = polysec_cur.click_handler_simple
        polysec_cur.pos_changed_handler = polysec_cur.pos_changed_handler_simple
        polysec_cur._method = null
        map.addMapItem(polysec_cur)
        poly_obj = polyComponent.createObject(map_component)
        map.addMapItem(poly_obj)
        map.removeMapItem(poly_obj)
    }


    function createPoly() {
        var poly_cur = polyComponent.createObject(map_component)
        map.addMapItem(poly_cur)
        return poly_cur
    }

    function createRect() {
        var poly_cur = createPoly()
        poly_cur.click_handler = poly_cur.click_handler_rect
        poly_cur.pos_changed_handler = poly_cur.pos_changed_handler_rect
        return poly_cur
    }

    function createPath() {
        var path_cur = pathComponent.createObject(map_component)
        map.addMapItem(path_cur)
        return path_cur
    }

    function loadPath(file){

        var jsondata = cmdWrapper.loadPathFromFile(file)

        var data = JSON.parse(jsondata)

        var i,j,lat,lon,p


        //clearAll()
        for(i = 0; i < data.paths.length; i++){
            switch(data.paths[i].name){
/*
            case "RectPath":
                map.removeMapItem(rect_list)
                if (currentState === generalState.empty){
                    currentState = generalState.poly
                    if (rect_cur)
                        rect_cur.end.disconnect(endPoly)
                    rect_cur = rectComponent.createObject(map_component, {offset:data.paths[i].offset, angle:data.paths[i].angle, debug_c: overlay_canvas, editCircle: editCircle})
                    rect_list.push(rect_cur)
                    map.addMapItem(rect_cur)

                    rect_cur.line.color = "#33cc33"
                    for(j = 0; j < data.paths[i].values.length; j++){
                        lat = data.paths[i].values[j].latitude
                        lon = data.paths[i].values[j].longitude

                        p = QtPositioning.coordinate(lat,lon)
                        rect_cur.addCoordinate(p)
                    }

                    //rect_cur.generate_path()
                    //rect_cur.draw_path()
                    //rect_cur.generate_nurbs()
                    rect_cur.end()

                    rect_cur.end.connect(endPoly)
                    rect_list.push(rect_cur)
                    endPoly()
                }
                break;
*/
            case "PolyPath":
                pathRectPoly.load_poly(data.paths[i])
                break

            case "PointPath":
                map.removeMapItem(uniquelist)

                if (currentState === generalState.empty){
                    currentState = generalState.path
                    if (path_cur)
                        path_cur.end.disconnect(endPath)
                    path_cur = pathComponent.createObject(map_component)
                    uniquelist.push(path_cur)
                    map.addMapItem(path_cur)

                    path_cur.line.color = "#33cc33"
                    for(j = 0; j < data.paths[i].values.length; j++){
                        lat = data.paths[i].values[j].latitude
                        lon = data.paths[i].values[j].longitude

                        p = QtPositioning.coordinate(lat,lon)
                        path_cur.addCoordinate(p)
                    }

                    path_cur.end.connect(endPath)
                    endPath()
                    uniquelist.push(path_cur)
                }
                break;
            case "SecurityPoly":
                map.removeMapItem(polysec_cur)
                if (polysec_cur)
                    polysec_cur.end.disconnect(endPolySec)
                polysec_cur = polysecComponent.createObject(map_component)
                map.addMapItem(polysec_cur)

                polysec_cur.line.color = "#161fc4"
                for(j = 0; j < data.paths[i].values.length; j++){
                    lat = data.paths[i].values[j].latitude
                    lon = data.paths[i].values[j].longitude

                    p = QtPositioning.coordinate(lat,lon)
                    polysec_cur.addCoordinate(p)
                }
                polysec_cur.close_polygon()
                polysec_cur.end.connect(endPolySec)
                endPolySec()
                polysec_cur.closed = true
                break

            }
        }

    }

    function savePath(filePath){
        var i
        var j

        var l = []

        var all_paths = {
            security_box: null,
            paths: []
        }

        for(i = 0; i < uniquelist.length; i++)
            all_paths.paths.push(uniquelist[i].serialize())

        all_paths.paths.push(polysec_cur.create_JSON())

        console.log("JSON to save "+JSON.stringify(all_paths))

        console.log("PATH to save "+filePath)

        cmdWrapper.savePathToFile(filePath, JSON.stringify(all_paths))
        return
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
        for(var i=0; i<el_list.length; i++)
            el_list[i].destroy() //FIXME
            console.log(el_list.length)
        el_list = []
        //loadPath()
    }

    function clearAll(){
        clearUlisseTrace()
    }
}
