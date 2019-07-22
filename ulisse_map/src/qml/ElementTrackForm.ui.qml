import QtQuick 2.6
import QtQml.Models 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.7
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

import "../scripts/helper.js" as Helper

RowLayout {
    id: tracklistlayout
    //elementi che definiscono l'elemento
    property int ntrack: -1
    property var _comp
    property var nametrack
    //se il nome del track non dipende dalla posizione
    property alias name: name
    property alias tracklistlayout: tracklistlayout
    property alias backbut: backbut

    property bool toggled: false
    property bool expanded: false

    signal selected(var path)
    signal edit(var path)

    state: {
        0: "empty",
                1: "path",
                2: "rect",
                3: "poly",
                4: "polysec",
                5: "editmode",
                6: "deletemode"
    }[mapView.currentState]

    // modo per distanziare
    height: 30
    scale: 1
    antialiasing: true

    Button {
        id: name
        text: expanded ? _comp._pathName : ntrack
        Layout.fillHeight: true
        Layout.fillWidth: true
        antialiasing: false
        enabled: true

        background: Rectangle {
            visible: true
            id: backbut
            opacity: 1
            color: "#abcdef"
            border.width: 1
            radius: 2
        }
    }
    states: [
        State {
            name: "editmode"
            PropertyChanges {
                target: deleteItem
                enabled: false
            }
        }
    ]
}
