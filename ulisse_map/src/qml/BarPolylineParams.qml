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
    property var  nameTrack: textnametrack.text
    property var nameTrack_
    signal accept
    signal discard

    function getParams(){
        return {}
    }

    function fill_cur_values(values){
            nameTrack_=values.name
    }



    TextField {
        id: textnametrack
        text: nameTrack_
        placeholderText:  qsTr("insert name")
        enabled: true
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
