import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

MapComponentForm {

    id: map_component

    function clearUlisseTrace() {
        ulissePath.path = []
        ulissePath.firstRun = true
    }

    function createPath() {
         if (currentState === generalState.empty){
             currentState = generalState.path
             deletePath() //TEMPORARY
             mapView.pathCurrentState = pathState.creating
             click_handler = path_click_handler
             pos_changed_handler = function(){}
         }
    }

    property list<MapRectangle> rect_list
    property MapRectangle rect_cur

    property list<MapPolygon> poly_list
    property MapPolygon poly_cur

    property Component rectComponent
    property Component polyComponent

    Component.onCompleted: {
        rectComponent = Qt.createComponent("MapRectangle.qml");
        polyComponent = Qt.createComponent("MapPolygon.qml");

    }

    function createRect() {
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

    function endPath(){
        if (currentState === generalState.path){
            click_handler = function(){}
            pos_changed_handler = function(){}
            currentState = generalState.empty
        }
    }

    function deletePath() {
        if (currentState === generalState.path){
            while (waypointPath.pathLength() > 0) {
                map.removeMapItem(mapCircles[waypointPath.pathLength() - 1])
                mapCircles[waypointPath.pathLength() - 1].destroy()
                waypointPath.removeCoordinate(waypointPath.pathLength() - 1)
            }

            waypointPath.opacity = 0.0
            if (mapView.pathCurrentState != pathState.empty) {
                cmdWrapper.cancelPath()
                mapView.pathCurrentState = pathState.empty
            }
        }
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

    function addWaypoint(waypoint){
        waypointPath.addCoordinate(waypoint)
        mapCircles[waypointPath.pathLength(
                       ) - 1] = mapCircleComponent.createObject(map, {
                                                                    "center.latitude": waypoint.latitude,
                                                                    "center.longitude": waypoint.longitude
                                                                })

        if (mapCircleComponent.status === Component.Ready) {
            map.addMapItem(mapCircles[waypointPath.pathLength() - 1])
            waypointPath.opacity = 1.0
            console.log(("Added waypoint! (size: %1)").arg(
                            waypointPath.pathLength()))

            marker_coords = waypoint
            markerIcon.coordinate = waypoint
            markerIcon.opacity = 1.0
        }
    }

    function removeWaypoint(){
        if (waypointPath.pathLength() > 0) {
            map.removeMapItem(mapCircles[waypointPath.pathLength() - 1])
            mapCircles[waypointPath.pathLength() - 1].destroy()

            waypointPath.removeCoordinate(waypointPath.pathLength() - 1)
        }
    }

    property var click_handler : function(){}
    property var pos_changed_handler : function(){}

    mapMouseArea.onClicked: {
        click_handler(mouse)
    }

    mapMouseArea.onPositionChanged: {
       pos_changed_handler(mouse)
    }

    function path_click_handler(mouse){
        if (pathCurrentState === pathState.creating) {
            if (mouse.button & Qt.LeftButton) {
                var p = Qt.point(mouse.x, mouse.y)
                var wp = map.toCoordinate(p)
                var lastwp = waypointPath.coordinateAt(waypointPath.pathLength()-1)
                var lastp = map.fromCoordinate(lastwp)
                if (Math.sqrt(Math.pow(p.x-lastp.x,2) + Math.pow(p.y-lastp.y,2)) < 10)
                    endPath()
                else
                    addWaypoint(wp)
            }
            if (mouse.button & Qt.RightButton) {
                removeWaypoint()
            }
        } else if (mouse.button & Qt.LeftButton) {
            marker_coords = map.toCoordinate(Qt.point(mouse.x, mouse.y))
            markerIcon.coordinate = map.toCoordinate(Qt.point(mouse.x,
                                                              mouse.y))
            markerIcon.opacity = 1.0
       }
    }
}
