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
    Layout.fillWidth: true
    Layout.fillHeight: true
    z: 100

    property bool buttons: false
    property var angle: parseInt(angleField.text)
    property var offset: parseInt(offsetField.text)
    property var method: methodField.currentText
    property var  nameTrack: textnametrack.text
    property var nameTrack_
    property var offset_
    property var angle_
    property var method_
    signal accept
    signal discard

    function getParams(){
        return {
            angle: angle,
            offset: offset,
            method: method
        }
    }

    function fill_cur_values(values){
            angle_= values.params.angle
            offset_= values.params.offset
            method_=values.params.method
            nameTrack_=values.name
    }

    Label {
        id:nameTrackLabel
        text: "INSERT PATH NAME:"
    }

    TextField {
        id: textnametrack
        text:  nameTrack_
        placeholderText:  qsTr("insert name")
        horizontalAlignment: TextInput.AlignHCenter
    }

    Label {
        id:offsettLabel
        text: "INSERT OFFSET:"
    }

    TextField {
        id: offsetField
        text:offset_
        placeholderText: "Offset"
        horizontalAlignment: TextInput.AlignHCenter
    }

    Label {
        id:angleLabel
        text: "INSERT ANGLE:"
    }

    TextField {
        id: angleField
        text: angle_
        placeholderText: "Angle"
        horizontalAlignment: TextInput.AlignHCenter
    }

    Label {
        id:methodLabel
        text: "CHOOSE  NURBS:"
    }
    ComboBox {
        id: methodField
        Layout.fillWidth: true
        model: ["simple", "single_winding"]
    }

    Button {
        id: cancelPolyEdit
        text: qsTr("Cancel")
        visible: buttons
        onClicked: function(){
            discard()
        }
    }

    Button {
        id: confirmPolyEdit
        text: qsTr("Confirm")
        visible: buttons
        onClicked: function(){
            accept()
        }
    }
}
