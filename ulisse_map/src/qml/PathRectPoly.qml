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
        pathRectPoly.rowFigure.visible = false
        pathRectPoly.rowLayout.visible = true
        pathRectPoly.rowEditPlay.visible = false
        var offset = parseInt(offsetField.text)
        var angle = parseInt(angleField.text)
        map.createRect(offset, angle) //, idxField.currentText)
    }
    b_poly.onClicked: {
        pathRectPoly.rowFigure.visible = false
        pathRectPoly.rowLayout.visible = true
        pathRectPoly.rowEditPlay.visible = false
        var offset = parseInt(offsetField.text)
        var angle = parseInt(angleField.text)
        //console.log(idxField.currentText)
        map.createPoly(offset, angle)//, idxField.currentText)
    }
    b_polysec.onClicked: {
        map.center = fbkUpdater.ulisse_pos
        map.createPolySec()
    }

    rowLayout.onVisibleChanged: function(){
        console.log(pathRectPoly.rowLayout.visible)
    }


    cancel_menuShape.onClicked:{
        pathRectPoly.rowFigure.visible = false
        pathRectPoly.rowLayout.visible = false
        pathRectPoly.rowEditPlay.visible = false
    }

    cancel_menuOffset.onClicked:{
        pathRectPoly.rowFigure.visible = true
        pathRectPoly.rowLayout.visible = false
        pathRectPoly.rowEditPlay.visible = false

        map.removeLastPath()
    }

    confermPolygonOffset.onClicked:{
        pathRectPoly.rowFigure.visible = false
        pathRectPoly.rowLayout.visible = false
        pathRectPoly.rowEditPlay.visible = false
    }

    idxField.enabled: map.polysec_cur.closed? true : false
    buttonSave.onClicked:map.save_mod(map.actualtrack, angleField.text, offsetField.text )
    buttonDiscard.onClicked : map.abort_mod(map.actualtrack)
}

