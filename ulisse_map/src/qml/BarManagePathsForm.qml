import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

RowLayout {
    property alias b_polySweep: b_polySweep
    property alias b_rectSweep: b_rectSweep
    property alias b_polyline: b_polyline
    property alias b_Racetrack: b_Racetrack

    property alias cancelPathChoice: cancelPathChoice

    property alias buttonEdit: buttonEdit
    property alias buttonPlay: buttonPlay
    property alias buttonToggle: buttonToggle
    property alias buttonDeselectAll: buttonDeselectAll

    property alias rowChoices: rowChoices
    property alias panelPathChoice: panelPathChoice
    property alias panelParamsPolygon: panelParamsPolygon
    property alias panelParamsPolyline: panelParamsPolyline
    property alias panelManage: panelManage
    property var panels: [panelPathChoice, panelParamsPolygon, panelParamsPolyline, panelManage]

    id: rowChoices

    RowLayout {
        id: panelPathChoice
        height: 100
        visible: false

        Button {
            id: b_polySweep
            text: qsTr("Polygon Sweep")
        }

        Button {
            id: b_rectSweep
            text: qsTr("Rectangle Sweep")
        }

        Button {
            id: b_Racetrack
            text: qsTr("Racetrack")
        }

        Button {
            id: b_polyline
            text: qsTr("Polyline")
        }


        Button {
            id: cancelPathChoice
            text: qsTr("Cancel")
        }
    }

    BarParamsPolygon {
        id: panelParamsPolygon
        visible: false
    }

    BarParamsPolyline {
        id: panelParamsPolyline
        visible: false
    }

    // Row for edit/play
    RowLayout {
        id: panelManage
        width: 100
        height: 100
        visible: false

        Button {
            id: buttonToggle
            text: qsTr("Toggle A/B")
        }

        Button {
            id: buttonPlay
            text: qsTr("Play")
        }

        Button {
            id: buttonEdit
            text: qsTr("Edit")
        }

        Button {
            id: buttonDeselectAll
            text: qsTr("X")
            Material.background: red
            Material.foreground: "white"
            Layout.leftMargin: 10
        }
    }
}
