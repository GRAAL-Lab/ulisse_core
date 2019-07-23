import QtQuick 2.9
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
            angle_= (values.params.angle !== undefined)? values.params.angle : 30
            offset_= (values.params.offset !== undefined)? values.params.offset : 30
            method_= (values.params.method !== undefined)? values.params.method : "simple"
            nameTrack_= (values.name!== undefined)? values.name : "Path"
    }

    TextField {
        id: textnametrack
        text:  (nameTrack_ !== undefined)? nameTrack : ""
        placeholderText:  qsTr("insert name")
    }

    TextField {
        id: offsetField
        text: (offset_ !== undefined)? offset_ : ""
        placeholderText: "Offset"
    }

    TextField {
        id: angleField
        text: (angle_ !== undefined)? angle_ : ""
        placeholderText: "Angle"
    }

    ComboBox {
        id: methodField
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
