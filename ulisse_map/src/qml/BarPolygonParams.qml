import QtQuick 2.9
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."
RowLayout {
    id: root
    Layout.fillWidth: true
    Layout.fillHeight: true

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

    GroupBox {
        id: groupBoxname
        y: -10
        width: 200
        height: textnametrack.height+40
        font.capitalization: Font.AllUppercase
        clip: true
        title: qsTr("Name")

        label: Label {
                x: groupBoxname.leftPadding
                y: 20
                width: groupBoxname.availableWidth
                text: groupBoxname.title
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
                background: Rectangle {
                    y: 0
                    width: 0
                    height: 0
                    color: "transparent"
                    border.color: "transparent"
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
            text:  (nameTrack_ !== undefined)? nameTrack_ : ""
            font.capitalization: Font.AllUppercase
            placeholderText:  qsTr("insert name")
            horizontalAlignment: Text.AlignHCenter
        }
    }

    GroupBox {
        id: groupBoxoffset
        y: -10
        width: 200
        height: textnametrack.height+40
        font.capitalization: Font.AllUppercase
        clip: true
        title: qsTr("Path Name")

        label: Label {
                x: groupBoxoffset.leftPadding
                y: 20
                width: groupBoxoffset.availableWidth
                text: groupBoxoffset.title
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
                background: Rectangle {
                    y: 0
                    width: 0
                    height: 0
                    color: "transparent"
                    border.color: "transparent"
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
            id: offsetField
            text: (offset_ !== undefined)? offset_ : ""
            placeholderText: "Offset"
            horizontalAlignment: Text.AlignHCenter
        }
    }

    GroupBox {
        id: groupBoxangle
        y: -10
        width: 200
        height: textnametrack.height+40
        font.capitalization: Font.AllUppercase
        clip: true
        title: qsTr("Angle")

        label: Label {
                x: groupBoxangle.leftPadding
                y: 20
                width: groupBoxangle.availableWidth
                text: groupBoxangle.title
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
                background: Rectangle {
                    y: 0
                    width: 0
                    height: 0
                    color: "transparent"
                    border.color: "transparent"
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
            id: angleField
            text: (angle_ !== undefined)? angle_ : ""
            placeholderText: "Angle"
            horizontalAlignment: TextInput.AlignHCenter
        }
    }

    GroupBox {
        id: groupBoxnurbs
        y: -10
        width: 200
        height: textnametrack.height+40
        font.capitalization: Font.AllUppercase
        clip: true
        title: qsTr("Path Type")

        label: Label {
                x: groupBoxnurbs.leftPadding
                y: 20
                width: groupBoxnurbs.availableWidth
                text: groupBoxnurbs.title
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
                background: Rectangle {
                    y: 0
                    width: 0
                    height: 0
                    color: "transparent"
                    border.color: "transparent"
                }
            }
        background: Rectangle {
                y: 0
                width: 0
                height: 0
                color: "transparent"
                border.color: "transparent"
            }
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
