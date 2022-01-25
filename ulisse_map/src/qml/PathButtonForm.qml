import QtQuick 2.6
import QtQml.Models 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.7
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."
import "../scripts/helper.js" as Helper

RowLayout {
    id: pathButtonRoot
    property variant managedPath
    property alias pathButton: pathButton
    property bool toggled: false

    signal selected(var path)
    signal edit(var path)

    // modo per distanziare
    scale: 1
    antialiasing: true

    Button {
        id: pathButton
        text: managedPath === undefined ? "Path" : managedPath.pathName
        Layout.fillHeight: true
        Layout.fillWidth: true
        antialiasing: false
        enabled: true
    }

    Button {
        id: savePathButton
        text: "\uE816" // save-icon
        font.family: "fontello"
        font.pointSize: 10
        padding: 5
        Layout.maximumWidth: 30
        Layout.fillHeight: true
        ToolTip.delay: 1000
        ToolTip.timeout: 5000
        ToolTip.visible: hovered
        ToolTip.text: qsTr("Save Path")

        onClicked: {
            if (managedPath === undefined){
                console.log("managedPath is undefined")
            }

            savePathDialog.pathToSave = managedPath;
            savePathDialog.open()
        }
    }

    Button {
        id: deletePathButton
        text: "\uE82A" // trash-icon
        font.family: "fontello"
        font.pointSize: 10
        padding: 5
        Layout.maximumWidth: 30
        Layout.fillHeight: true
        hoverEnabled: true
        ToolTip.delay: 1000
        ToolTip.timeout: 5000
        ToolTip.visible: hovered
        ToolTip.text: qsTr("Delete Path")

        onClicked: {
            confirmDialog.open()
        }


        Dialog {
            id: confirmDialog
            title: "Delete Path?"
            //icon: StandardIcon.Question
            //text: "Delete the selected path?"
            standardButtons: Dialog.Ok// | Dialog.Cancel
            //Component.onCompleted: visible = true
            onAccepted: {
                managedPath.deregister_map_items()
                map.removeMapItem(managedPath)
                pathButtonRoot.destroy()
                bar_manage.hide_all()
            }
        }
    }


    /*Component.onCompleted: {
        pathButtonRoot.state = "reparented"
        console.log("reparented")
    }*/


    states: [
        /*State {
            name: "reparented"
            ParentChange { target: pathButtonRoot; parent: pathCmdPane.pathButtonsColumn }
        },*/

        State {
            name: "editmode"
            PropertyChanges {
                target: deleteItem
                enabled: false
            }

            PropertyChanges {
                target: pathButton
            }
        }
    ]
}
