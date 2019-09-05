import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
import "."

BarManagePathsForm {
    id: root

    property bool inhibit: false
    property var trackComponent

    Component.onCompleted: function () {
        hide_all()
        trackComponent = Qt.createComponent("PathIcon.qml")
    }

    cancelPathChoice.onClicked: function () {
        sidebar_manage.deselect_all()
        sidebar_manage.enableBtns(true)
        hide_all()
    }

    buttonToggle.onClicked: function () {
        if (cur_managed === null)
            return
        cur_managed.toggle_dir()
    }

    /*-------------- POLY CREATION/EDITING ----------------*/
    property var cur_managed
    property var params_panel

    b_poly.onClicked: function () {
        cur_managed = map.createPoly()
        params_panel = panelParamsPolygon
        start()
    }

    b_rect.onClicked: function () {
        cur_managed = map.createRect()
        params_panel = panelParamsPolygon
        start()
    }

    b_path.onClicked: function () {
        cur_managed = map.createPath()
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
        if (cur_managed === null)
            return
        show_create()
        var cur_val = cur_managed.get_params()
        params_panel.fill_cur_values(cur_val)
        cur_managed.end.connect(end)
        map.click_handler = cur_managed.click_handler
        map.pos_changed_handler = cur_managed.pos_changed_handler
    }

    function edit() {
        if (cur_managed === null)
            return
        sidebar_manage.enableBtns(false)
        var cur_val = cur_managed.get_params()
        params_panel.fill_cur_values(cur_val)
        show_edit()
        cur_managed.begin_edit()
        map.click_handler = cur_managed.click_mod_handler
        map.pos_changed_handler = cur_managed.pos_changed_mod_handler
    }

    property int n: 0
    function end() {
        cur_managed.end.disconnect(end)
        confirm()
        var v = trackComponent.createObject(sidebar_manage.columnTrack)
        v.managed_path = cur_managed
        v.ntrack = ++n
        v.selected.connect(function (path) {
            sidebar_manage.update_selection(path)
            manage(path)
        })
        cur_managed.check_safe(map.polysec_cur)
        sidebar_manage.update_selection(cur_managed)
        manage(cur_managed)
    }

    function confirm() {
        map.click_handler = map.click_goto_handler
        map.pos_changed_handler = function () {}
        var p = params_panel.getParams()
        cur_managed.enable_ab_markers()
        cur_managed.confirm_edit(params_panel.nameTrack, p)
        cur_managed.check_safe(map.polysec_cur)
        sidebar_manage.enableBtns(true)
        show_manage()
    }

    function discard() {
        map.click_handler = map.click_goto_handler
        map.pos_changed_handler = function () {}
        cur_managed.enable_ab_markers()
        cur_managed.discard_edit()
        cur_managed.check_safe(map.polysec_cur)
        sidebar_manage.enableBtns(true)
        show_manage()
    }

    /*-----------------------------------------------------------*/

    /*------------------ PATH MANAGEMENT ------------------------*/
    function manage(path) {
        for (var e in sidebar_manage.columnTrack.children)
            sidebar_manage.columnTrack.children[e].managed_path.disable_ab_markers()
        path.enable_ab_markers()
        if (inhibit) return
        cur_managed = path
        show_manage()
    }

    buttonEdit.onClicked: function () {
        if (cur_managed === null)
            return
        edit()
        sidebar_manage.enableBtns(false)
    }

    buttonPlay.onClicked: function () {
        if (cur_managed === null)
            return
        if (!cur_managed.safe)
            toast.show("Unsafe path due to operational space limits.", 2000)
        else
            cmdWrapper.sendPath(JSON.stringify(cur_managed.generate_nurbs()))
    }

    /*------------------------------------------------------------*/
    function show_shape_choice() {
        show_panel(panelPathChoice)
    }

    function show_panel(panel) {
        manageToolbar.visible = true
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
        for (var i in sidebar_manage.columnTrack.children)
            sidebar_manage.columnTrack.children[i].managed_path.disable_ab_markers()
        for (var i in panels)
            panels[i].visible = false
        manageToolbar.visible = false
    }

    function enableBtns(y) {
        panelManage.visible = y
    }
}
