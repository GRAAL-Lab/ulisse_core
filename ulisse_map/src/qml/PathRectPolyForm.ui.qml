import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

RowLayout {
    property alias b_path: b_path
    property alias b_rect: b_rect
    property alias b_poly: b_poly
    property alias b_polysec: b_polysec

    property alias cancelPathChoice: cancelPathChoice

    property alias buttonEdit: buttonEdit
    property alias buttonPlay: buttonPlay

    property alias panelPathChoice: panelPathChoice
    property alias panelParamsPolygon: panelParamsPolygon
    property alias panelParamsPolyline: panelParamsPolyline
    property alias panelManage: panelManage
    property var panels: [panelPathChoice, panelParamsPolygon, panelParamsPolyline, panelManage]

    id: rowChoices
    anchors.fill: map

    //Row for shapes
    RowLayout {
        id: panelPathChoice
        height: 100
        visible: false

        Button {
            id: b_poly
            text: qsTr("Polygon")
        }

        Button {
            id: b_rect
            text: qsTr("Rectangle")
        }

        Button {
            id: b_path
            text: qsTr("Path")
        }

        Button {
            id: b_polysec
            text: qsTr("Security")
        }

        Button {
            id: cancelPathChoice
            text: qsTr("Cancel")
        }
    }

    BarPolygonParams {
        id: panelParamsPolygon
    }

    BarPolylineParams {
        id: panelParamsPolyline
    }

    //Row for edit/play
    RowLayout {
        id: panelManage
        width: 100
        height: 100
        visible: false

        Button {
            id: buttonPlay
            text: qsTr("play")
        }

        Button {
            id: buttonEdit
            text: qsTr("edit")
        }
    }
}
