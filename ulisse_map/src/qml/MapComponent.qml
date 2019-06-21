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
        mapView.pathCurrentState = pathState.creating
    }

    function startPath() {
        greenFlag.coordinate = waypointPath.path[waypointPath.pathLength() - 1]
        mapView.pathCurrentState = pathState.active
        map.markerIconOpacity = 0.4
        cmdWrapper.startPath()
    }

    function stopPath() {
        mapView.pathCurrentState = pathState.stopped
        cmdWrapper.stopPath()
    }

    function resumePath() {
        mapView.pathCurrentState = pathState.active
        cmdWrapper.resumePath()
    }

    function interruptPathIfActive() {
        if (mapView.pathCurrentState == pathState.active) {
            stopPath()
            toast.show("Path Interrupted!", 3000)
        }
    }

    function deletePath() {
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

    mapMouseArea.onClicked: {

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
