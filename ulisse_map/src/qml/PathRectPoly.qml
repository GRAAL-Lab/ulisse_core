import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2

PathRectPolyForm {
    id: root

    property var trackComponent

    Component.onCompleted: {
        hide_all()
        trackComponent = Qt.createComponent("ElementTrack.qml")
    }


    /*-------------- POLY CREATION/EDITING ----------------*/

    property var poly_cur

    b_poly.onClicked: function(){start_poly()}
    rowPolyParams.onAccept: function(){confirm_poly()}
    rowPolyParams.onDiscard: function(){discard_poly()}

    function start_poly(){
        poly_cur = map.createPoly()
        if (poly_cur === null) return
        show_poly_create()
        poly_cur.end.connect(end_poly)
        map.click_handler = poly_cur.click_handler
        map.pos_changed_handler = poly_cur.pos_changed_handler
    }

    function edit_poly(poly){
        if (poly_cur === null) return
        poly_cur = poly
        show_poly_edit()
        poly_cur.begin_edit()
        map.click_handler = poly_cur.click_mod_handler
        map.pos_changed_handler = poly_cur.pos_changed_mod_handler
    }

    property var v

    function end_poly(){
        poly_cur.end.disconnect(end_poly)
        confirm_poly()
        v = trackComponent.createObject(slidersLeft.columnTrack)
        v._comp = poly_cur
        v.edit.connect(edit_poly)
    }

    function confirm_poly() {
        map.click_handler = function(){}
        map.pos_changed_handler = function(){}
        poly_cur.confirm_edit(rowPolyParams.angle, rowPolyParams.offset, "simple")
        poly_cur.check_safe(map.polysec_cur)
        hide_all()
    }

    function discard_poly() {
        map.click_handler = function(){}
        map.pos_changed_handler = function(){}
        poly_cur.discard_edit()
        hide_all()
    }

    /*-----------------------------------------------------------*/

    b_polysec.onClicked: {
        map.center = fbkUpdater.ulisse_pos
        map.createPolySec()
    }

    cancel_menuShape.onClicked:{
        rowFigure.visible = false
        rowPolyParams.visible = false
        rowEditPlay.visible = false
    }

    idxField.enabled: map.polysec_cur.closed? true : false
    buttonSave.onClicked:map.save_mod(map.actualtrack, angleField.text, offsetField.text )
    buttonDiscard.onClicked : map.abort_mod(map.actualtrack)

    function show_shape_choice(){
        rowFigure.visible = true
        rowPolyParams.visible = false
        rowEditPlay.visible = false
    }

    function show_poly_create(){
        rowFigure.visible = false
        rowPolyParams.visible = true
        rowPolyParams.edit = false
        rowEditPlay.visible = false
    }

    function show_poly_edit(){
        rowFigure.visible = false
        rowPolyParams.visible = true
        rowPolyParams.edit = true
        rowEditPlay.visible = false
    }

    function hide_all(){
        rowFigure.visible = false
        rowPolyParams.visible = false
        rowEditPlay.visible = false
    }
}

