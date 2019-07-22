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
    width: 100
    height: 100
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
    property bool editmode : false
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
            editmode = true
            angle_= values.params.angle
            offset_= values.params.offset
            method_=values.type
            nameTrack_=values.name
    }

    TextField {
        id: textnametrack
        text: editmode? nameTrack_ : "Path"
        placeholderText:  qsTr("insert name")
        enabled: true
    }

    TextField {
        id: offsetField
        text:editmode? offset_: qsTr("30")
        placeholderText: "Offset"
        enabled: true
    }

    TextField {
        id: angleField
        text: editmode? angle_: qsTr("30")
        placeholderText: "Angle"
        enabled: true
    }

    ComboBox {
        id: methodField
        model: ["simple", "single_winding"]
        enabled: true
    }

    Button {
        id: cancelPolyEdit
        text: qsTr("Cancel")
        visible: buttons
        onClicked: function(){
            discard()
            editmode = false
        }
    }

    Button {
        id: confirmPolyEdit
        text: qsTr("Confirm")
        visible: buttons
        onClicked: function(){
            accept()
            editmode = false
        }
    }
}
