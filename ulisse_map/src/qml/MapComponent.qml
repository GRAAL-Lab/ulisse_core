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

    property var marker_coords: markerIcon.coordinate

    function click_goto_handler(mouse) {
        if(mouse.button & Qt.LeftButton){
            markerIcon.coordinate = toCoordinate(Qt.point(mouse.x, mouse.y))
            markerIcon.opacity = 1
        }
    }

    property var click_handler: click_goto_handler
    property var pos_changed_handler: function () {}

    mapMouseArea.onClicked: {
        click_handler(mouse)
    }
    mapMouseArea.onPositionChanged: {
        pos_changed_handler(mouse)
    }

    property MapPolygon poly_obj

    property MapPolygon polysec_cur

    property Component polyComponent
    property Component polysecComponent
    property Component pathComponent
    property Component trackComponent

    property var path_file: home_dir + "/Ulisse_Data/Path_Files/"

    Component.onCompleted: {
        polyComponent = Qt.createComponent("MapPolygon.qml")
        pathComponent = Qt.createComponent("MapPath.qml")
        trackComponent = Qt.createComponent("PathButton.qml")

        polysec_cur = polyComponent.createObject(map_component)
        polysec_cur.click_handler = polysec_cur.click_handler_simple
        polysec_cur.pos_changed_handler = polysec_cur.pos_changed_handler_simple
        polysec_cur._method = null
        map.addMapItem(polysec_cur)
        poly_obj = polyComponent.createObject(map_component)
        map.addMapItem(poly_obj)
        map.removeMapItem(poly_obj)
        map.center = fbkUpdater.ulisse_pos
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

    ulisseGPSIcon.transform: [
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
        settings.mapCenter = map.center
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
