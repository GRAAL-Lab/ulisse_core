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

    property bool edit: false
    property var angle: parseInt(angleField.text)
    property var offset: parseInt(offsetField.text)
    signal accept
    signal discard

    onVisibleChanged: function(){console.log("V" + visible)}

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
        id: idxField
        model: ["single_winding", "2curves", "helix"]
        enabled: false
    }


    Button {
        id: cancelPolyEdit
        Layout.preferredWidth: 17
        Layout.preferredHeight: 17
        text: qsTr("x")
        visible: edit
        onClicked: function(){discard()}
    }

    Button {
        id: confirmPolyEdit
        Layout.preferredWidth: 17
        Layout.preferredHeight: 17
        text: qsTr("v")
        visible: edit
        onClicked: function(){accept()}
    }


}
