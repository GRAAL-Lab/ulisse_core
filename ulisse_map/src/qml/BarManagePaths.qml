import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
//import QtQuick.Dialogs 1.2
import "."

BarManagePathsForm {
    
    property bool inhibit: false
    property var pathButtonComponent
    
    Component.onCompleted: function () {
        hide_all()
        pathButtonComponent = Qt.createComponent("PathButton.qml")
    }
    
    cancelPathChoice.onClicked: function () {
        pathCmdPane.cancelPathCreation()
    }
    
    buttonToggle.onClicked: function () {
        if (cur_managed === null)
            return
        cur_managed.toggle_dir()
    }
    
    buttonDeselectAll.onClicked: function () {
        pathCmdPane.deselect_all()
        hide_all()
    }
    
    /*-------------- POLY CREATION/EDITING ----------------*/
    property var cur_managed
    property var params_panel
    
    b_polySweep.onClicked: function () {
        cur_managed = map.createPolySweepPath()
        params_panel = panelParamsPolygon
        start()
    }
    
    b_rectSweep.onClicked: function () {
        cur_managed = map.createRectSweepPath()
        params_panel = panelParamsPolygon
        start()
    }

    b_hippodrome.onClicked: function () {
        cur_managed = map.createHippodromePath()
        params_panel = panelParamsPolygon
        panelParamsPolygon.polypath_type_ = "Hippodrome"
        start()
    }
    
    b_polyline.onClicked: function () {
        cur_managed = map.createPolylinePath()
        params_panel = panelParamsPolyline
        start()
    }
    
    panelParamsPolygon.onAccept: function () {
        confirm()
    }
    panelParamsPolygon.onDiscard: function () {
        discard()
    }
    panelParamsPolyline.onAccept: function () {
        confirm()
    }
    panelParamsPolyline.onDiscard: function () {
        discard()
    }
    
    function start() {
        // Here the click handlers are assigned and the parameters are shown
        if (cur_managed === null)
            return
        show_create()
        var cur_val = cur_managed.get_params()
        params_panel.fill_cur_values(cur_val)
        cur_managed.end.connect(end)
        window.sig_escape.connect(abort_h)
        map.clickHandler = cur_managed.clickHandler
        map.posChangedHandler = cur_managed.posChangedHandler
    }
    
    function abort_h(){
        //console.log("[BarManagePaths] abort_h()")
        map.clickHandler = map.click_goto_handler
        map.posChangedHandler = function () {}
        pathCmdPane.enableBtns(true)
        map.mapMouseArea.hoverEnabled = false
        if (cur_managed !== undefined) {
            //console.log("[BarManagePaths] abort_h() - cur_managed !== undefined")
            cur_managed.deregister_map_items()
            map.removeMapItem(cur_managed)
            cur_managed.destroy()
        }
        hide_all()
        window.sig_escape.disconnect(abort_h)
        //map.mapTextOverlay.visible = false
    }

    function edit() {
        if (cur_managed === null)
            return
        pathCmdPane.enableBtns(false)
        var cur_val = cur_managed.get_params()
        //console.log("Type: " + cur_managed.type)
        switch (cur_managed.type) {
        case "PolyPath":
            params_panel = panelParamsPolygon
            break
        case "PointPath":
            params_panel = panelParamsPolyline
            break
        }
        params_panel.fill_cur_values(cur_val)
        show_edit()
        cur_managed.begin_edit()
        map.clickHandler = cur_managed.click_mod_handler
        map.posChangedHandler = cur_managed.pos_changed_mod_handler
    }

    function end() {
        //console.log("[BarManagePaths] end()")
        window.sig_escape.disconnect(abort_h)
        cur_managed.end.disconnect(end)
        confirm()
        var v = pathButtonComponent.createObject(pathCmdPane.pathButtonsColumn)

        v.managedPath = cur_managed
        v.selected.connect(function (path) {
            pathCmdPane.update_selection(path)
            manage(path)
        })
        cur_managed.check_safe(map.safety_polygon)
        pathCmdPane.update_selection(cur_managed)
        manage(cur_managed)
        map.mapTextOverlay.visible = false
    }

    function confirm() {
        console.time("BarManagePaths::confirm()")
        //console.log("[BarManagePaths] confirm()")
        map.clickHandler = map.click_goto_handler
        map.posChangedHandler = function () {}
        var p = params_panel.getParams()
        cur_managed.enable_ab_markers()
        cur_managed.confirm_edit(params_panel.nameTrack, p)
        cur_managed.check_safe(map.safety_polygon)
        pathCmdPane.enableBtns(true)
        show_manage()
        console.timeEnd("BarManagePaths::confirm()")
    }

    function discard() {
        //console.log("[BarManagePaths] discard()")
        map.clickHandler = map.click_goto_handler
        map.posChangedHandler = function () {}
        if (cur_managed !== undefined){
            if( cur_managed.path.length !== 0 )    {
                //console.log("[BarManagePaths] discard() - cur_managed !== undefined")
                //console.log("cur_managed: " + cur_managed.pathName)
                cur_managed.enable_ab_markers()
                cur_managed.discard_edit()

                cur_managed.check_safe(map.safety_polygon)
                show_manage()
            }
        }
        pathCmdPane.enableBtns(true)
    }

    /*-----------------------------------------------------------*/

    /*------------------ PATH MANAGEMENT ------------------------*/
    function manage(path) {
        for (var e in pathCmdPane.pathButtonsColumn.children)
            pathCmdPane.pathButtonsColumn.children[e].managedPath.disable_ab_markers()
        path.enable_ab_markers()
        if (inhibit) return
        cur_managed = path
        show_manage()
    }

    buttonEdit.onClicked: function () {
        if (cur_managed === null)
            return
        edit()
        pathCmdPane.enableBtns(false)
    }

    buttonPlay.onClicked: function () {
        if (cur_managed === null)
            return
        if (!cur_managed.safe) {
            toast.show("Unsafe path due to operational space limits.", 2000)
        } else {
            cmdWrapper.sendPath(JSON.stringify(cur_managed.serialize()))
        }
    }

    /*------------------------------------------------------------*/
    function show_shape_choice() {
        // KEEP
        show_panel(panelPathChoice)
    }

    function show_panel(panel) {
        // KEEP
        pathManageToolbar.visible = true
        for (var i in panels)
            panels[i].visible = (panel === panels[i])
    }

    function show_create() {
        show_panel(params_panel)
        params_panel.buttons = false
    }

    function show_edit() {
        show_panel(params_panel)
        params_panel.buttons = true
    }

    function show_manage() {
        show_panel(panelManage)
    }

    function hide_all() {
        for (var i in pathCmdPane.pathButtonsColumn.children)
            pathCmdPane.pathButtonsColumn.children[i].managedPath.disable_ab_markers()
        for (var j in panels)
            panels[j].visible = false
        pathManageToolbar.visible = false
        map.mapTextOverlay.visible = false
        cur_managed = undefined; // (?)
    }

    function enableBtns(y) {
        panelManage.visible = y
    }

}
