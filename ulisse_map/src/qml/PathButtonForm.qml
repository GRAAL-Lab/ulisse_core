import QtQuick 2.6
import QtQml.Models 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.7
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."
import "../scripts/helper.js" as Helper

RowLayout {
    //id: tracklistlayout
    property int ntrack: -1
    property var managedPath
    property alias pathButton: pathButton
    //property alias tracklistlayout: tracklistlayout
    property bool toggled: false

    signal selected(var path)
    signal edit(var path)

    // modo per distanziare
    scale: 1
    antialiasing: true

    Button {
        id: pathButton
        text: "Path"
        Layout.fillHeight: true
        Layout.fillWidth: true
        antialiasing: false
        enabled: true
    }

    onManagedPathChanged: {
        pathButton.text = managedPath.pathName;
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
