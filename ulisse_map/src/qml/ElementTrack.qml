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

    name.onDoubleClicked: {

}
    tracklistlayout.y: {tracklistlayout.y = tracklistlayout.height*(ntrack)
    }
    menubutton.onClicked: menu.open()

    editItem.onTriggered: function()
    {
        _comp.actualtrack = ntrack
        _comp.modify(ntrack)
    }

    deleteItem.onTriggered: _comp.deletenel(ntrack)
    name.onClicked: {
        backbut.color= (backbut.color==="#ff0000")? "#0000ff" : "#ff0000"
    }
}
