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
    property int ntrack
    property var _comp
    property var _sliderL
    property var  nametrack: qsTr(ntrack)
    //se il nome del track non dipende dalla posizione
    property alias menu: menu
    property alias menubutton: menubutton
    property alias editItem: editItem
    property alias deleteItem: deleteItem
    property alias name: name

    signal edit(var a)
    property alias tracklistlayout: tracklistlayout
    property alias backbut: backbut

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
    width: _sliderL.expanded?  _sliderL.sliderW + 120 : _sliderL.sliderW
    height: 30
    scale: 1
    antialiasing: true

    ToolButton {
        text: qsTr("‹")
        z: 1
        highlighted: false
        Layout.fillHeight: true
        Layout.fillWidth: true
    }
    Button {
        id: name
        text: _sliderL.expanded? ntrack : nametrack
        anchors.fill: parent
        Layout.fillHeight: true
        antialiasing: false
        enabled: false
        //elide: Button.ElideRight
        //                horizontalAlignment: Qt.AlignBottom
        //                verticalAlignment: Qt.AlignBottom
        Layout.fillWidth: false

        background: Rectangle {
            visible: false
            id: backbut
            opacity: 1
            color: "#0000ff"
            border.width: 1
            radius: 2
        }

        TextInput {
            id: textField
            x: 0
            y: 0
            width: 1
            height: 40
            color: "#fcaf3e"
            text: nametrack
            maximumLength: 16
            anchors.horizontalCenter: parent.horizontalCenter
            clip: false
            visible: false
            smooth: true
            enabled: false
            horizontalAlignment: Text.AlignHCenter

        }
    }
    ToolButton {
        id: menubutton
        text: qsTr("⋮")
        Layout.fillHeight: true
        Layout.fillWidth: true
        wheelEnabled: false
    }

    Menu {
        id: menu

        MenuItem {
            id: editItem
            text: qsTr("Edit...")
            enabled: true
        }
        MenuItem {
            id: deleteItem
            text: qsTr("Delete")
        }
        MenuItem {
            id: playItem
            text: qsTr("Play")
        }
    }
    states: [
        State {
            name: "editmode"
            PropertyChanges {
                target: deleteItem
                enabled: false
            }
        },
        State {
            name: "deletemode"
            PropertyChanges {
                target: name
                enabled: true
            }
            PropertyChanges {
                target: backbut
                visible: true
            }
        }
    ]
}
