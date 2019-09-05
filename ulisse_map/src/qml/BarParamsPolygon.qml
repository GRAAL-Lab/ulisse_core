import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
import "."

RowLayout {
    id: root
    property bool buttons: false
    property var angle: parseInt(angleField.text)
    property var offset: parseInt(offsetField.text)
    property var method: methodField.currentText
    property var nameTrack: textnametrack.text
    property var nameTrack_
    property var offset_
    property var angle_
    property var method_
    signal accept
    signal discard

    function getParams() {
        return {
            angle: angle,
            offset: offset,
            method: method
        }
    }

    function fill_cur_values(values) {
        angle_ = (values.params.angle !== undefined) ? values.params.angle : 30
        offset_ = (values.params.offset !== undefined) ? values.params.offset : 30
        method_ = (values.params.method !== undefined) ? values.params.method : "simple"
        nameTrack_ = (values.name !== undefined) ? values.name : "Path"
    }

    LabelledField {
        id: groupBoxname
        title: qsTr("Name")

        TextField {
            id: textnametrack
            text: (nameTrack_ !== undefined) ? nameTrack_ : ""
            font.capitalization: Font.AllUppercase
            placeholderText: qsTr("insert name")
            horizontalAlignment: Text.AlignHCenter
        }
    }

    LabelledField {
        id: groupBoxoffset
        title: qsTr("Offset")

        TextField {
            id: offsetField
            text: (offset_ !== undefined) ? offset_ : ""
            placeholderText: "Offset"
            horizontalAlignment: Text.AlignHCenter
        }
    }

    LabelledField {
        id: groupBoxangle
        title: qsTr("Angle")

        TextField {
            id: angleField
            text: (angle_ !== undefined) ? angle_ : ""
            placeholderText: "Angle"
            horizontalAlignment: TextInput.AlignHCenter
        }
    }

    LabelledField {
        id: groupBoxnurbs
        height: methodField.height + 40
        title: qsTr("Path Type")

        ComboBox {
            id: methodField
            hoverEnabled: true
            Layout.fillWidth: true
            model: ["simple", "single_winding"]
        }
    }

    Button {
        id: cancelPolyEdit
        text: qsTr("Cancel")
        visible: buttons
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
