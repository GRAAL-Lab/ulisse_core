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
    property bool isSelected: false

    signal selected(var path)
    signal edit(var path)

    // Modo per distanziare
    scale: 1
    //antialiasing: true

    Button {
        id: pathButton
        text: managedPath === undefined ? "Path" : managedPath.pathName
        Layout.fillHeight: true
        Layout.fillWidth: true
        antialiasing: false
        enabled: true
        Material.background: isSelected ? orange : lightergrey
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
        Material.background: lightergrey

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
        Material.background: lightergrey

        onClicked: {
            confirmDialog.open()
        }

        onHoveredChanged: {
            highlighted = hovered
            Material.background = hovered ? red : lightergrey
        }

        Dialog {
            id: confirmDialog
            title: "Delete Path?"
            standardButtons: Dialog.Ok
            onAccepted: {
                if (pathButtonRoot.isSelected){
                    bar_manage.hide_all()
                }
                managedPath.deregister_map_items()
                map.removeMapItem(managedPath)
                pathButtonRoot.destroy()
            }
        }
    }



    states: [
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
