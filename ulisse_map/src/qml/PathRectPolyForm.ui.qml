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
    id: rowLayout1
    state: {
        0: "empty",
                1: "path",
                2: "rect",
                3: "poly",
                4: "polysec"
    }[mapView.currentState]

    Button {
        id: b_path
        text: qsTr("Path")
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
        id: b_poly
        text: qsTr("Polygon")
        enabled: false
        highlighted: false
    }

    Button {
        id: b_polysec
        text: qsTr("Security")
        enabled: false
        highlighted: false
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
        }
    ]
}
