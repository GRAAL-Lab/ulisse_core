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
    property bool buttons: false

    property string nameTrack: textnametrack.text
    property var size_1: parseInt(size_1Field.text)
    property var size_2: parseInt(size_2Field.text)
    property var angle: parseInt(angleField.text)
    property string polypath_type: polypathType.currentText

    property var nameTrack_
    property var size_1_
    property var size_2_
    property var angle_
    property var polypath_type_

    signal accept
    signal discard

    //width: parent.width

    function getParams() {
        return {
            angle: angle,
            size_1: size_1,
            size_2: size_2,
            polypath_type: polypath_type
        }
    }

    function fill_cur_values(values) {
        angle_ = (values.params.angle !== undefined) ? values.params.angle : 0
        size_1_ = (values.params.size_1 !== undefined) ? values.params.size_1 : 20
        size_2_ = (values.params.size_2 !== undefined) ? values.params.size_2 : 10
        polypath_type_ = (values.params.polypath_type !== undefined) ? values.params.polypath_type : "Serpentine"
        nameTrack_ = (values.name !== undefined) ? values.name : "Path"
    }

    LabelledField {
        id: groupBoxname
        title: qsTr("Name")
        Layout.preferredWidth: 180

        TextField {
            id: textnametrack
            width: parent.width
            text: (nameTrack_ !== undefined) ? nameTrack_ : ""
            //font.capitalization: Font.AllUppercase
            placeholderText: qsTr("insert name")
            horizontalAlignment: Text.AlignHCenter
        }
    }

    LabelledField {
        title: qsTr("Size 1")
        Layout.preferredWidth: 100

        TextField {
            id: size_1Field
            width: parent.width
            text: (size_1_ !== undefined) ? size_1_ : ""
            placeholderText: "Size 1"
            horizontalAlignment: Text.AlignHCenter
            validator: DoubleValidator { bottom:0.0; top: 1000.0 }
        }
    }

    LabelledField {
        title: qsTr("Size 2")
        Layout.preferredWidth: 100
        visible: (polypath_type == "RaceTrack" | polypath_type == "Hippodrome") ? true : false

        TextField {
            id: size_2Field
            width: parent.width
            text: (size_2_ !== undefined) ? size_2_ : ""
            placeholderText: "Size 2"
            horizontalAlignment: Text.AlignHCenter
            validator: DoubleValidator { bottom:0.0; top: 1000.0 }
        }
    }

    LabelledField {
        title: qsTr("Angle")
        Layout.preferredWidth: 80

        TextField {
            id: angleField
            width: parent.width
            text: (angle_ !== undefined) ? angle_ : ""
            placeholderText: "Angle"
            horizontalAlignment: Text.AlignHCenter
            validator: DoubleValidator { bottom:0.0; top: 360.0 }
        }
    }

    LabelledField {
        id: polyCoverageMethod
        height: polypathType.height + 40
        width: polypathType.width + 50
        title: qsTr("Path Type")

        ComboBox {
            id: polypathType
            hoverEnabled: true
            width: 140
            model: cmdWrapper !== null ? cmdWrapper.polypath_types : []
            currentIndex: cmdWrapper !== null ? cmdWrapper.polypath_types.indexOf(polypath_type_) : -1
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
