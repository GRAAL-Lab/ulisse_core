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
             mapView.pathCurrentState = pathState.creating
             click_handler = drawlines
             pos_changed_handler = function(){}
         }
    }

     function createRect() {
         if (currentState === generalState.empty){
             currentState = generalState.rect
             //mapView.rectCurrentState = rectState.creating
             rect_phase = 0
             click_handler = rect
             pos_changed_handler = rect_pos_changed_handler
         }
    }

    function startPath() {
        greenFlag.coordinate = waypointPath.path[waypointPath.pathLength() - 1]
        mapView.pathCurrentState = pathState.active
        map.markerIconOpacity = 0.4
        cmdWrapper.startPath()
    }

    function stopPath() {
         if (currentState === generalState.path){
            mapView.pathCurrentState = pathState.stopped
            cmdWrapper.stopPath()
         }
    }

    function stopRect() {
         if (currentState === generalState.rect){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
         }
    }

    function resumePath() {
        if (currentState === generalState.path){
        mapView.pathCurrentState = pathState.active
        cmdWrapper.resumePath()
        }
    }

    function interruptPathIfActive() {
        if (mapView.pathCurrentState == pathState.active) {
            stopPath()
            toast.show("Path Interrupted!", 3000)
        }
    }

    function deletePath() {
        if (currentState === generalState.path){
            while (waypointPath.pathLength() > 0) {
                map.removeMapItem(mapCircles[waypointPath.pathLength() - 1])
                mapCircles[waypointPath.pathLength() - 1].destroy()
                waypointPath.removeCoordinate(waypointPath.pathLength() - 1)
            }
            console.log(("Destroyed Path: pathLength = %1").arg(
                            waypointPath.pathLength()))

            waypointPath.opacity = 0.0
            if (mapView.pathCurrentState != pathState.empty) {
                cmdWrapper.cancelPath()
                mapView.pathCurrentState = pathState.empty
            }
        }
        click_handler = function(){}
        pos_changed_handler = function(){}
        currentState = generalState.empty
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
                    console.log(("Removed waypoint! (size: %1)").arg(
                                    waypointPath.pathLength()))
                }
            }

    property var rect_phase: 0

    function project(p0,m,p1){
        var p2 = Qt.point(0,0);
        if ( (Math.abs(m) === 16331239353195370) || (Math.abs(m) === Infinity)){
            p2.x = p0.x
            p2.y = p1.y
        } else if (m === 0){
            p2.x = p1.x
            p2.y = p0.y
        } else {
            var m_perp = -1/m
            p2.x = (m*p0.x - m_perp*p1.x + p1.y - p0.y)/(m-m_perp)
            p2.y = m*(p2.x - p0.x) + p0.y
        }
        return p2
    }

    property var click_handler : function(){}
    property var pos_changed_handler : function(){}

    mapMouseArea.onClicked: {
        click_handler(mouse)
    }

    mapMouseArea.onPositionChanged: {
       pos_changed_handler(mouse)
    }
function rect_pos_changed_handler(mouse){
    if (rect_phase === 1){
        var p0 = map.fromCoordinate(rectanglePath.coordinateAt(0));
        var p1 = Qt.point(mouse.x, mouse.y)
        if (mouse.modifiers & Qt.ShiftModifier){
            var n_intervals = 16
            var snap_interval = 2*Math.PI/n_intervals
            var theta = Math.atan2(p1.y-p0.y, p1.x-p0.x)
            var snap_idx = Math.floor((theta + snap_interval/2) / snap_interval)
            var snap_theta = snap_idx*snap_interval
            var m = Math.tan(snap_theta)
            p1 = project(p0,m,p1)
        }
        rectanglePath.replaceCoordinate(1, map.toCoordinate(p1))
    } else if (rect_phase === 2){
        var p0 = map.fromCoordinate(rectanglePath.coordinateAt(0));
        var p1 = map.fromCoordinate(rectanglePath.coordinateAt(1));
        var m = (p1.y - p0.y)/(p1.x - p0.x)
        var m_perp = -1/m
        var pm = Qt.point(mouse.x, mouse.y)
        var p2 = project(p1, m_perp, pm)
        var p3 = project(p0, m_perp, pm)
        rectanglePath.replaceCoordinate(2, map.toCoordinate(p2))
        rectanglePath.replaceCoordinate(3, map.toCoordinate(p3))
    }
        }

function rect(mouse){
    if (mouse.button & Qt.LeftButton){
        if (rect_phase === 0){
            for(var i=0; i<5; i++)
                rectanglePath.removeCoordinate(0)
            var p = map.toCoordinate(Qt.point(mouse.x, mouse.y))
            rectanglePath.addCoordinate(p)
            rectanglePath.addCoordinate(p)
            mapMouseArea.hoverEnabled = true
            rect_phase = 1
        } else if (rect_phase === 1){
            var p = map.toCoordinate(Qt.point(mouse.x, mouse.y))
            rectanglePath.addCoordinate(p)
            rectanglePath.addCoordinate(rectanglePath.coordinateAt(0))
            rectanglePath.addCoordinate(rectanglePath.coordinateAt(0))
            rect_phase = 2
        } else if (rect_phase === 2){
            rect_phase = 0
            mapMouseArea.hoverEnabled = false
            stopRect()
        }
    }
}

function drawlines(mouse){
    if (pathCurrentState === pathState.creating) {
        if (mouse.button & Qt.LeftButton) {
            var wp = map.toCoordinate(Qt.point(mouse.x, mouse.y))
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



    Component.onCompleted: {
        console.log (Material.color(
                    Material.Green,
                    Material.Shade300))
    }
  }
