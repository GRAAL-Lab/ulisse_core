import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

RowLayout {
    id: root

    property bool buttons: false
    property var nameTrack: textnametrack.text
    property var nameTrack_
    signal accept
    signal discard

    function getParams() {
        return {

        }
    }

    function fill_cur_values(values) {
        nameTrack_ = values.name
    }

    GroupBox {
        id: groupBox
        y: -10
        width: 200
        height: textnametrack.height + 40
        clip: true
        title: qsTr("Name")

        label: Label {
            x: groupBox.leftPadding
            y: 20
            width: groupBox.availableWidth
            text: groupBox.title
            elide: Text.ElideRight
            background: Rectangle {
                y: 0
                width: 0
                height: 0
                color: "transparent"
                border.color: "#ffffff"
            }
        }

        background: Rectangle {
            y: 0
            width: 0
            height: 0
            color: "transparent"
            border.color: "transparent"
        }

        TextField {
            id: textnametrack
            text: (nameTrack_ !== undefined) ? nameTrack_ : "Path"

            placeholderText: qsTr("insert name")
            enabled: true
        }
    }

    Button {
        id: cancelPolyEdit
        text: qsTr("Cancel")
        visible: buttons
        Layout.fillWidth: false
        onClicked: function () {
            discard()
        }
    }

    Button {
        id: confirmPolyEdit
        text: qsTr("Confirm")
        visible: buttons
        onClicked: function () {
            accept()
        }
    }
}
