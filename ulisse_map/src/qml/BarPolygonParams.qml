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
    signal accept
    signal discard

    function getParams(){
        return {
            angle: angle,
            offset: offset,
            method: method
        }
    }

    TextField {
        id: offsetField
        text: qsTr("30")
        placeholderText: "Offset"
        enabled: true
    }

    TextField {
        id: angleField
        text: qsTr("30")
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
        onClicked: function(){discard()}
    }

    Button {
        id: confirmPolyEdit
        text: qsTr("Confirm")
        visible: buttons
        onClicked: function(){accept()}
    }
}
