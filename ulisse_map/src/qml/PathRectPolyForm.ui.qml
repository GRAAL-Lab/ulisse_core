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
    property alias offsetField: offsetField
    property alias angleField: angleField
    //property alias buttonEdit: buttonEdit
    property alias buttonSave: buttonSave
    property alias buttonDiscard: buttonDiscard
    property alias idxField: idxField

    property alias rowFigure: rowFigure
    property alias rowLayout: rowLayout
    property alias rowEditPlay: rowEditPlay

    id: rowChoices
    anchors.fill: map
    state: {
        0: "empty",
                1: "path",
                2: "rect",
                3: "poly",
                4: "polysec",
                5: "editmode"
    }[mapView.currentState]

    //Row for shapes
    RowLayout {
        id: rowFigure
        height: 100
        visible: false

        Button {
            id: b_poly
            text: qsTr("Polygon")
            Layout.fillHeight: false
            enabled: false
            highlighted: false
        }

        Button {
            id: b_rect
            text: qsTr("Rectangle")
            enabled: false
            highlighted: false
        }

        Button {
            id: b_path
            text: qsTr("Path")
            enabled: false
            highlighted: false
        }

        Button {
            id: b_polysec
            text: qsTr("Security")
            enabled: false
            highlighted: false
        }
    }

    //Row for Offset
    RowLayout {
        id: rowLayout
        width: 100
        height: 100
        enabled: true
        visible: false

        TextField {
            id: offsetField
            text: qsTr("30")
            placeholderText: "Offset"
            enabled: map.polysec_cur.closed ? true : false
        }

        TextField {
            id: angleField
            text: qsTr("30")
            placeholderText: "Angle"
            enabled: map.polysec_cur.closed ? true : false
        }
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

    states: [
        State {
            name: "empty"
            PropertyChanges {
                target: b_path
                enabled: true
            }
            PropertyChanges {
                target: b_rect
                enabled: true
            }
            PropertyChanges {
                target: b_poly
                enabled: true
            }
            PropertyChanges {
                target: b_polysec
                enabled: true
            }
        },
        State {
            name: "path"
            PropertyChanges {
                target: b_path
                enabled: true
                highlighted: true
            }
            PropertyChanges {
                target: rowLayout
                enabled: false
            }
            PropertyChanges {
                target: idxField
                enabled: false
            }
        },
        State {
            name: "rect"
            PropertyChanges {
                target: b_rect
                enabled: true
                highlighted: true
            }
        },
        State {
            name: "poly"
            PropertyChanges {
                target: b_poly
                enabled: true
                highlighted: true
            }
        },
        State {
            name: "polysec"
            PropertyChanges {
                target: b_polysec
                enabled: true
                highlighted: true
            }
            PropertyChanges {
                target: rowLayout
                enabled: false
            }
            PropertyChanges {
                target: idxField
                enabled: false
            }
        },
        State {
            name: "editmode"
            PropertyChanges {
                target: buttonSave
                enabled: true
            }
            PropertyChanges {
                target: buttonDiscard
                enabled: true
            }
            PropertyChanges {
                target: idxField
                enabled: true
            }
        }
    ]
}
