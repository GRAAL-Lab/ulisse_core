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
import "."
import "../scripts/helper.js" as Helper

PathIconForm {
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
    expanded: parent.expanded
    width: parent.width
    //nametrack: qsTr(ntrack.toString())
}
