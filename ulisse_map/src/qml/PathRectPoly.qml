import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

PathRectPolyForm {
    id: rowLayout1

    b_path.onClicked:{
        map.createPath()
    }
    b_rect.onClicked: {
        var offset = parseInt(offsetField.text)
        var angle = parseInt(angleField.text)
        map.createRect(offset, angle)
    }
    b_poly.onClicked: {
        var offset = parseInt(offsetField.text)
        var angle = parseInt(angleField.text)
        map.createPoly(offset, angle)
    }
    b_polysec.onClicked: {
        map.center = fbkUpdater.ulisse_pos
        var offset = parseInt(offsetField.text)
        var angle = parseInt(angleField.text)
        map.createPolySec(offset, angle)
    }

    buttonEdit.onClicked: {
        map.modify(parseInt(idxField.text))
    }
    buttonSave.onClicked: {
        map.save_mod(parseInt(idxField.text))
    }
    buttonDiscard.onClicked: {
        map.abort_mod(parseInt(idxField.text))
    }
}

