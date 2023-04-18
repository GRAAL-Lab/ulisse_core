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

    //property alias mapTextOverlay: mapTextOverlay
    property var marker_coords: markerIcon.coordinate

    function click_goto_handler(mouse) {
        if(mouse.button & Qt.LeftButton){
            markerIcon.coordinate = toCoordinate(Qt.point(mouse.x, mouse.y))
            markerIcon.opacity = 1
        }
    }

    property var clickHandler: click_goto_handler
    property var posChangedHandler: function () {}

    mapMouseArea.onClicked: {
        clickHandler(mouse)
    }
    mapMouseArea.onPositionChanged: {
        posChangedHandler(mouse)
    }

    property MapPolyPath safety_polygon

    // These components will create all the dynamic objects
    property Component polyPathComponent
    property Component pointPathComponent
    property Component pathButtonComponent

    Component.onCompleted: {

        polyPathComponent = Qt.createComponent("MapPolyPath.qml")
        pointPathComponent = Qt.createComponent("MapPointPath.qml")
        pathButtonComponent = Qt.createComponent("PathButton.qml")

        createSafetyPolygon()
    }

    function createSafetyPolygon() {
        safety_polygon = polyPathComponent.createObject(map_component, {type: "SafetyBoundary", pathName: "Safety Area", opacity: 1.0})
        safety_polygon.clickHandler = safety_polygon.click_handler_non_intersecting
        safety_polygon.posChangedHandler = safety_polygon.pos_changed_handler_simple
        safety_polygon._angle = 0
        safety_polygon._size_1 = safety_polygon._size_2 = 0

        if (settings.savedBoundary != "null") {
            var data = JSON.parse(settings.savedBoundary)
            safety_polygon.deserialize(data)
            safety_polygon.close_polygon()
        }

        safety_polygon._polypathType = null
        map.addMapItem(safety_polygon)
    }

    function createPolySweepPath() {
        var poly_cur = polyPathComponent.createObject(map_component)
        map.addMapItem(poly_cur)
        return poly_cur
    }

    function createHippodromePath() {
        var poly_cur = polyPathComponent.createObject(map_component)
        poly_cur.clickHandler = poly_cur.click_handler_point
        //poly_cur.clickHandler = poly_cur.pos_changed_handler_simple
        poly_cur._polypathType = "Hippodrome"
        map.addMapItem(poly_cur)
        return poly_cur
    }

    function createRectSweepPath() {
        var poly_cur = createPolySweepPath()
        poly_cur.clickHandler = poly_cur.click_handler_rect
        poly_cur.posChangedHandler = poly_cur.pos_changed_handler_rect
        return poly_cur
    }

    function createPolylinePath() {
        var path_cur = pointPathComponent.createObject(map_component)
        map.addMapItem(path_cur)
        return path_cur
    }

    AddonsBridgeVisualizer {
        id: addonsBridgeVisualizer
    }

    compass.transform: [
        Rotation {
            origin.x: compass.width / 2
            origin.y: compass.height / 2
            angle: - map.bearing
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

    currentArrow.transform: [
        Rotation {
            origin.x: currentArrow.width / 2
            origin.y: currentArrow.height / 2
            angle: fbkUpdater.water_current_deg - map.bearing
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

    currentLabel.text:"Current\n%1 m/s".arg(fbkUpdater.water_current_norm.toFixed(2))

    ulisseIcon.transform: [
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: fbkUpdater.ulisse_rpy_deg.z - map.bearing
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

    ulisseGPSIcon.transform: [
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: fbkUpdater.ulisse_rpy_deg.z - map.bearing
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
        settings.mapCenter = map.center
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
        mapsidebar.markerText = "%1, %2".arg(marker_coords.latitude.toFixed(8)).arg(
                    marker_coords.longitude.toFixed(8))
    }

    // This timer centers the map on the vehicle at startup and then draws the trace of the vehicle
    Timer {
        interval: 500
        running: true
        repeat: true
        onTriggered: {
            if (ulissePath.firstRun) {
                ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                ulissePath.firstRun = false
                ulisseIcon.visible = true;
            }
            // To reduce the line density (and avoid to overload the gui)
            // we add a new point only every 1.0 meter
            var lastCoord = ulissePath.coordinateAt(ulissePath.pathLength() - 1)
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
    }

    function clearAll() {
        clearUlisseTrace()
    }

}
