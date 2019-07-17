import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

ColumnLayout {
    property alias b_path: b_path
    property alias b_rect: b_rect
    property alias b_poly: b_poly
    property alias offsetField: offsetField
    property alias angleField: angleField
    property alias buttonEdit: buttonEdit
    property alias buttonSave: buttonSave
    property alias buttonDiscard: buttonDiscard
    property alias idxField: idxField
    id: cl1
    state: {
        0: "empty",
                1: "path",
                2: "rect",
                3: "poly"
    }[mapView.currentState]

    RowLayout {
        id: rl1
        width: 100
        height: 100
        Layout.fillWidth: true

        Button {
            id: b_poly
            text: qsTr("Polygon")
            Layout.fillHeight: false
            Layout.fillWidth: true
            enabled: false
            highlighted: false
        }

        Button {
            id: b_rect
            text: qsTr("Rectangle")
            Layout.fillWidth: true
            enabled: false
            highlighted: false
        }

        Button {
            id: b_path
            text: qsTr("Path")
            Layout.fillWidth: true
            enabled: false
            highlighted: false
        }
    }

    RowLayout {
        id: rowLayout
        width: 100
        height: 100

        TextField {
            id: offsetField
            text: qsTr("30")
            placeholderText: "Offset"
        }

        TextField {
            id: angleField
            text: qsTr("30")
            placeholderText: "Angle"
        }
    }

    RowLayout {
        id: rowLayout1
        width: 100
        height: 100

        Button {
            id: buttonEdit
            text: qsTr("edit")
        }

        Button {
            id: buttonSave
            text: qsTr("save")
        }

        Button {
            id: buttonDiscard
            text: qsTr("abort")
        }

        TextField {
            id: idxField
            text: qsTr("0")
            placeholderText: "Offset"
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
        },
        State {
            name: "path"
            PropertyChanges {
                target: b_path
                enabled: true
                highlighted: true
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
        }
    ]
}
