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
    property alias cancel_menuShape: cancel_menuShape
    //property alias buttonEdit: buttonEdit
    property alias buttonSave: buttonSave
    property alias buttonDiscard: buttonDiscard
    property alias idxField: idxField

    property alias rowFigure: rowFigure
    property alias rowPolyParams: rowPolyParams
    property alias rowEditPlay: rowEditPlay

    id: rowChoices
    anchors.fill: map

    //Row for shapes
    RowLayout {
        id: rowFigure
        height: 100
        visible: false

        Button {
            id: b_poly
            text: qsTr("Polygon")
            Layout.fillHeight: false
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
            id: cancel_menuShape
            Layout.preferredWidth: 17
            Layout.preferredHeight: 17
            text: qsTr("X")
        }
    }

    //Row for Offset
    BarPolyParams {
        id: rowPolyParams
    }

    //Row for edit/play
    RowLayout {
        id: rowEditPlay
        width: 100
        height: 100
        visible: false

        //        Button {

        //            id: buttonEdit
        //            text: qsTr("edit")
        //        }
        Button {
            id: buttonSave
            enabled: false
            text: qsTr("save")
        }

        Button {
            id: buttonDiscard
            enabled: false
            text: qsTr("abort")
        }
        //da qui puoi scegliere in che modo riempire i poligoni/rettangoli
        ComboBox {
            id: idxField
            model: ["single_winding", "2curves", "helix"]
            enabled: false
        }
    }
}
