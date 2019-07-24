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

    Component.onCompleted: function(){
        hide_all()
        trackComponent = Qt.createComponent("ElementTrack.qml")
    }

    cancelPathChoice.onClicked: function(){
        slidersLeft.deselect_all()
        slidersLeft.enableBtns(true)
        hide_all()
    }

    buttonToggle.onClicked: function(){
        if (cur_managed === null) return
        cur_managed.toggle_dir()
    }


    /*-------------- POLY CREATION/EDITING ----------------*/

    property var cur_managed
    property var params_panel

    b_poly.onClicked: function(){
        cur_managed = map.createPoly()
        params_panel = panelParamsPolygon
        start()
    }

    b_rect.onClicked: function(){
        cur_managed = map.createRect()
        params_panel = panelParamsPolygon
        start()
    }

    b_path.onClicked: function(){
        cur_managed = map.createPath()
        params_panel = panelParamsPolyline
        start()
    }

    panelParamsPolygon.onAccept: function(){confirm()}
    panelParamsPolygon.onDiscard: function(){discard()}
    panelParamsPolyline.onAccept: function(){confirm()}
    panelParamsPolyline.onDiscard: function(){discard()}

    function start(){
        if (cur_managed === null) return
        show_create()
        var cur_val=cur_managed.get_params()
        params_panel.fill_cur_values(cur_val)
        cur_managed.end.connect(end)
        map.click_handler = cur_managed.click_handler
        map.pos_changed_handler = cur_managed.pos_changed_handler
    }

    function edit(){
        if (cur_managed === null) return
        slidersLeft.enableBtns(false)
        var cur_val=cur_managed.get_params()
        params_panel.fill_cur_values(cur_val)
        show_edit()
        cur_managed.begin_edit()
        map.click_handler = cur_managed.click_mod_handler
        map.pos_changed_handler = cur_managed.pos_changed_mod_handler
    }

    property int n : 0
    function end(){
        cur_managed.end.disconnect(end)
        confirm()
        var v = trackComponent.createObject(slidersLeft.columnTrack)
        v._comp = cur_managed
        v.ntrack = ++n
        v.selected.connect(function (path){
            slidersLeft.update_selection(path)
            manage(path)
        })
        cur_managed.check_safe(map.polysec_cur)
        slidersLeft.update_selection(cur_managed)
        manage(cur_managed)
    }

    function confirm() {
        map.click_handler = map.click_goto_handler
        map.pos_changed_handler = function(){}
        var p = params_panel.getParams()
        cur_managed.enable_ab_markers()
        cur_managed.confirm_edit(params_panel.nameTrack, p)
        cur_managed.check_safe(map.polysec_cur)
        show_manage()
    }

    function discard() {
        map.click_handler = map.click_goto_handler
        map.pos_changed_handler = function(){}
        cur_managed.enable_ab_markers()
        cur_managed.check_safe(map.polysec_cur)
        cur_managed.discard_edit()
        show_manage()
    }


    /*-----------------------------------------------------------*/


    /*------------------ PATH MANAGEMENT ------------------------*/

    function manage(path) {
        cur_managed = path
        for (var i = 0; i<slidersLeft.columnTrack.children.length; i++)
            slidersLeft.columnTrack.children[i]._comp.disable_ab_markers()
        cur_managed.enable_ab_markers()
        show_manage()
    }

    buttonEdit.onClicked: function(){
        if (cur_managed === null) return
        edit()
        slidersLeft.enableBtns(false)
    }

    buttonPlay.onClicked: function(){
        if (cur_managed === null) return
        if (!cur_managed.safe) toast.show("Unsafe path due to operational space limits.", 2000)
        else
            cmdWrapper.sendPath(JSON.stringify(cur_managed.generate_nurbs()))
    }

    /*------------------------------------------------------------*/

    function show_shape_choice(){
        show_panel(panelPathChoice)
    }

    function show_panel(panel){
        manageToolbar.visible = true
        for (var i=0; i<panels.length; i++)
            panels[i].visible = (panel === panels[i])
    }

    function show_create(){
        show_panel(params_panel)
        params_panel.buttons = false
    }

    function show_edit(){
        show_panel(params_panel)
        params_panel.buttons = true
    }

    function show_manage(){
        show_panel(panelManage)
        slidersLeft.enableBtns(true)
    }

    function hide_all(){
        for (var i = 0; i<slidersLeft.columnTrack.children.length; i++)
            slidersLeft.columnTrack.children[i]._comp.disable_ab_markers()
        for (var i=0; i<panels.length; i++)
            panels[i].visible = false
        manageToolbar.visible = false
    }

    function enableBtns(y){
        panelManage.visible=y
    }
}
