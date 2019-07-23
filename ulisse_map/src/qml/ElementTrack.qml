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

ElementTrackForm{
    name.onClicked: function(){
        selected(_comp)
    }

    function toggle(){
        toggled = !toggled
        backbut.color = toggled ? "#ffffff" : "#abcdef"
    }

    function highlight(yes){
        toggled = yes
        backbut.color = yes ? "#ffffff" : "#abcdef"
        _comp.highlighted(yes)
    }

    //tracklistlayout.y: tracklistlayout.height*(tracklistlayout.ntrack)

    expanded: parent.expanded
    width: parent.width
    //nametrack: qsTr(ntrack.toString())
}
