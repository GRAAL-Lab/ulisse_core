import QtQuick 2.6
import QtQuick.Window 2.0
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtQuick.Controls.Styles 1.4
import "./qml"

ApplicationWindow {
    width: 800
    height: 600
    visible: true

    minimumHeight: 200
    minimumWidth: 300

    Material.theme: Material.Light
    Material.accent: Material.Orange

    /* Halting catamaran when space is pressed */
    Shortcut {
        sequence: " "
        //onActivated: /** Insert function **/
    }

    Settings {
        id: settings
        property string style: "Material"
    }



    MapView {
        id: mapview
        anchors.fill: parent
    }

    ToastManager {
        id: toast
        objectName: "toastManager"
    }
}
