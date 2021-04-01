import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQml.Models 2.1
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

PathButtonForm {
    name.onClicked: function () {
        selected(managed_path)
    }

    function toggle() {
        toggled = !toggled
        name.Material.background = toggled ? orange : green
        managed_path.highlighted(toggled)
    }

    function highlight(yes) {
        toggled = yes
        name.Material.background = yes ? orange : green
        managed_path.highlighted(yes)
    }

    Material.background: green

    //tracklistlayout.y: tracklistlayout.height*(tracklistlayout.ntrack)
    //expanded: parent.expanded
    width: parent.width
    //nametrack: qsTr(ntrack.toString())
}
