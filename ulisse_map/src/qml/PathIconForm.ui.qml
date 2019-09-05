import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Controls.Universal 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtQuick.Window 2.4
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
import QtQml.Models 2.1
import "."
import "../scripts/helper.js" as Helper

RowLayout {
    id: tracklistlayout
    //elementi che definiscono l'elemento
    property int ntrack: -1
    property var managed_path
    property alias name: name
    property alias tracklistlayout: tracklistlayout
    property bool toggled: false
    property bool expanded: false

    signal selected(var path)
    signal edit(var path)

    // modo per distanziare
    scale: 1
    antialiasing: true

    Button {
        id: name
        text: expanded ? managed_path._pathName : ntrack
        font.weight: Font.ExtraBold
        highlighted: true
        font.pointSize: 11
        Layout.fillHeight: true
        Layout.fillWidth: true
        antialiasing: false
        enabled: true
    }
    states: [
        State {
            name: "editmode"
            PropertyChanges {
                target: deleteItem
                enabled: false
            }

            PropertyChanges {
                target: name
            }
        }
    ]
}
