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

    property var cur_managed

    b_poly.onClicked: function(){start_poly()}
    rowPolyParams.onAccept: function(){confirm_poly()}
    rowPolyParams.onDiscard: function(){discard_poly()}

    function start_poly(){
        cur_managed = map.createPoly(30,30)
        if (cur_managed === null) return
        show_poly_create()
        cur_managed.end.connect(end_poly)
        map.click_handler = cur_managed.click_handler
        map.pos_changed_handler = cur_managed.pos_changed_handler
    }

    function edit_poly(){
        if (cur_managed === null) return
        show_poly_edit()
        cur_managed.begin_edit()
        map.click_handler = cur_managed.click_mod_handler
        map.pos_changed_handler = cur_managed.pos_changed_mod_handler
    }

    property var v

    function end_poly(){
        cur_managed.end.disconnect(end_poly)
        confirm_poly()
        v = trackComponent.createObject(slidersLeft.columnTrack)
        v._comp = cur_managed
        v.ntrack = map.uniquelist.length
        v.edit.connect(edit_poly)
        v.selected.connect(function (poly){
            slidersLeft.update_selection(poly)
            manage(poly)
        })
        slidersLeft.update_selection(cur_managed)
        manage(cur_managed)
    }

    function confirm_poly() {
        map.click_handler = function(){}
        map.pos_changed_handler = function(){}
        cur_managed.confirm_edit(rowPolyParams.angle, rowPolyParams.offset, "simple")
        cur_managed.check_safe(map.polysec_cur)
        show_path_manage()
    }

    function discard_poly() {
        map.click_handler = function(){}
        map.pos_changed_handler = function(){}
        cur_managed.discard_edit()
        show_path_manage()
    }

    function load_poly(data){
        cur_managed = map.createPoly(data.offset, data.angle)
        if (cur_managed === null) return
        show_poly_create()

        cur_managed.deserialize(data.values)
        cur_managed.draw_deferred()

        cur_managed.end.disconnect(end_poly)
        v = trackComponent.createObject(slidersLeft.columnTrack)
        v._comp = cur_managed
        v.ntrack = map.uniquelist.length
        v.edit.connect(edit_poly)
        v.selected.connect(function (poly){
            slidersLeft.update_selection(poly)
            manage(poly)
        })
        slidersLeft.update_selection(cur_managed)
        manage(cur_managed)

    }


    /*-----------------------------------------------------------*/


    /*------------------ PATH MANAGEMENT ------------------------*/

    function manage(path) {
        cur_managed = path
        show_path_manage()
    }

    buttonEdit.onClicked: function(){
        //if cur_managed is a polygonalpath
        edit_poly()
    }

    buttonPlay.onClicked: function(){
        //TODO
    }

    /*------------------------------------------------------------*/


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

    function show_path_manage(){
        slidersLeft.enableBtns(true)
        rowFigure.visible = false
        rowPolyParams.visible = false
        rowEditPlay.visible = true
    }

    function hide_all(){
        rowFigure.visible = false
        rowPolyParams.visible = false
        rowEditPlay.visible = false
    }

    /*----------------------------------------------------------*/

    b_polysec.onClicked: {
        map.center = fbkUpdater.ulisse_pos
        map.createPolySec()
    }

    cancel_menuShape.onClicked:{
        rowFigure.visible = false
        rowPolyParams.visible = false
        rowEditPlay.visible = false
    }
}
