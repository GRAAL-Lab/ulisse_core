import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0

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

    LabelledField {
        id: groupBox
        title: qsTr("Name")

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
