import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.7
import QtPositioning 5.6
import Qt.labs.settings 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Universal 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."


MapComponentForm {

    id: map_component

    property var click_handler : function(){}
    property var pos_changed_handler : function(){}

    mapMouseArea.onClicked: {click_handler(mouse)}
    mapMouseArea.onPositionChanged: {pos_changed_handler(mouse)}

    property list<MapRectangle> rect_list
    property MapRectangle rect_cur

    property list<MapPolygon> poly_list
    property MapPolygon poly_cur
    property MapPolygon poly_obj

    property list<MapPath> path_list
    property MapPath path_cur

    property MapPolygonSecurity polysec_cur

    property Component rectComponent
    property Component polyComponent
    property Component polysecComponent
    property Component pathComponent

    property var path_file: "/home/alessio/Desktop/Prova"

    Component.onCompleted: {
        rectComponent = Qt.createComponent("MapRectangle.qml")
        polyComponent = Qt.createComponent("MapPolygon.qml")
        polysecComponent = Qt.createComponent("MapPolygonSecurity.qml")
        pathComponent = Qt.createComponent("MapPath.qml")
        polysec_cur = polysecComponent.createObject(map_component)
        map.addMapItem(polysec_cur)
        poly_obj = polyComponent.createObject(map_component)
        map.addMapItem(poly_obj)
        map.removeMapItem(poly_obj)

    }

    function createRect(offset, angle) {
        if(!polysec_cur.closed){
            toast.show("Define Security Area First!")
            return
        }
         if (currentState === generalState.empty){
             currentState = generalState.rect
             if (rect_cur)
                 rect_cur.end.disconnect(endRect)
             rect_cur = rectComponent.createObject(map_component, {offset:offset, angle:angle, debug_c: overlay_canvas, editCircle: editCircle})
             rect_list.push(rect_cur)
             map.addMapItem(rect_cur)
             click_handler = rect_cur.click_handler
             pos_changed_handler = rect_cur.pos_changed_handler
             rect_cur.end.connect(endRect)
         }

    }

    function modify(idx){
        poly_list[idx].begin_edit()
        mapMouseArea.hoverEnabled = true
        click_handler = poly_list[idx].click_mod_handler
        pos_changed_handler = poly_list[idx].pos_changed_mod_handler
    }

    function save_mod(idx, angle, offset){
        mapMouseArea.hoverEnabled = false
        click_handler = null
        pos_changed_handler = null
        poly_list[idx].confirm_edit(angle, offset)
    }

    function abort_mod(idx){
        mapMouseArea.hoverEnabled = false
        click_handler = null
        pos_changed_handler = null
        poly_list[idx].discard_edit()
    }

    function createPoly(offset, angle) {
        if(!polysec_cur.closed){
            toast.show("Define Security Area First!")
            return
        }
        if (currentState === generalState.empty){
            currentState = generalState.poly
            poly_cur = polyComponent.createObject(map_component, {offset:offset, angle:angle, debug_c: overlay_canvas, editCircle: editCircle})
            poly_list.push(poly_cur)
            map.addMapItem(poly_cur)
            click_handler = poly_cur.click_handler
            pos_changed_handler = poly_cur.pos_changed_handler
            poly_cur.end.connect(endPoly)
        }
   }

    function createPolySec() {
            //TODO -> use a menu for editing the polygon
             if (currentState === generalState.empty){
                 currentState = generalState.polysec

                 polysec_cur.clear_path()

                 click_handler = polysec_cur.click_handler
                 pos_changed_handler = polysec_cur.pos_changed_handler
                 polysec_cur.end.connect(endPolySec)
             }
        }

    function createPath() {
        if(!polysec_cur.closed){
            toast.show("Define Security Area First!")
            return
        }

        if (currentState === generalState.empty){
            currentState = generalState.path
            if (path_cur)
                path_cur.end.disconnect(endPath)
            path_cur = pathComponent.createObject(map_component)
            path_list.push(path_cur)
            map.addMapItem(path_cur)
            click_handler = path_cur.click_handler
            pos_changed_handler = path_cur.pos_changed_handler
            path_cur.end.connect(endPath)
        }

    }

    function endRect() {
         if (currentState === generalState.rect){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
         }

    }

    function endPoly() {
         if (currentState === generalState.poly){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
             poly_cur.end.disconnect(endPoly)
         }
    }

    function endPolySec() {
         if (currentState === generalState.polysec){
             click_handler = function(){}
             pos_changed_handler = function(){}
             currentState = generalState.empty
         }
    }

    function endPath(){
        if (currentState === generalState.path){
            click_handler = function(){}
            pos_changed_handler = function(){}
            currentState = generalState.empty
        }
    }

    function openFile(fileName) {
        myFile.source = fileName
        var data = myFile.read()
        console.log(data)
        return data
    }

    function saveFile(fileName, text) {
        myFile.source = fileName
        var done = myFile.write(text)
        if(!done){
            console.log("Error in writing")
        }

    }


    //TODO, acquire from file
    function loadPath(file){

        var jsondata = openFile(path_file)

        //var jsondata = '{"paths":[{"name":"RectPath","offset":30,"angle":30,"values":[{"latitude":44.40107591944535,"longitude":8.938550991344044},{"latitude":44.40047969448704,"longitude":8.93841443556687},{"latitude":44.40007608275216,"longitude":8.940176670711962},{"latitude":44.400672307710465,"longitude":8.940313226489137},{"latitude":44.40107591944535,"longitude":8.938550991344044}]},{"name":"RectPath","offset":30,"angle":30,"values":[{"latitude":44.40069108402936,"longitude":8.940477945263183},{"latitude":44.4001436377436,"longitude":8.94060691461786},{"latitude":44.400414241381306,"longitude":8.941755566999255},{"latitude":44.40096168766707,"longitude":8.941626597644579},{"latitude":44.40069108402936,"longitude":8.940477945263183}]},{"name":"PolyPath","offset":30,"angle":30,"values":[{"latitude":44.39993766652866,"longitude":8.940000000000026},{"latitude":44.39943899637509,"longitude":8.939529641180883},{"latitude":44.399243863411584,"longitude":8.940075864332641},{"latitude":44.39940105390387,"longitude":8.940652433217394},{"latitude":44.39993766652866,"longitude":8.940000000000026}]},{"name":"PolyPath","offset":30,"angle":30,"values":[{"latitude":44.400197840578414,"longitude":8.938884794386013},{"latitude":44.3998292603344,"longitude":8.938770997897848},{"latitude":44.39974253523594,"longitude":8.93946894971478},{"latitude":44.40004065222675,"longitude":8.939734474846631},{"latitude":44.40018157973325,"longitude":8.939317221049606},{"latitude":44.400197840578414,"longitude":8.938884794386013}]},{"name":"PolyPath","offset":30,"angle":30,"values":[{"latitude":44.40006775369815,"longitude":8.940971063392823},{"latitude":44.39975879620314,"longitude":8.940743470416521},{"latitude":44.39962870834669,"longitude":8.941008995548373},{"latitude":44.39962870834669,"longitude":8.941600737299694},{"latitude":44.40000271014572,"longitude":8.941684188076295},{"latitude":44.40015989859932,"longitude":8.941426249345454},{"latitude":44.40006775369815,"longitude":8.940971063392823}]},{"name":"PointPath","values":[{"latitude":44.39909751327148,"longitude":8.938892380830026},{"latitude":44.398739766929,"longitude":8.939711715557621},{"latitude":44.39864219936646,"longitude":8.940887612637738},{"latitude":44.39894032196373,"longitude":8.941206242813166}]},{"name":"SecurityPoly","values":[{"latitude":44.40123310500795,"longitude":8.938285466212193},{"latitude":44.39862593808893,"longitude":8.938088218947428},{"latitude":44.39847416595517,"longitude":8.941631083032746},{"latitude":44.401292727013285,"longitude":8.941790398120446},{"latitude":44.40123310500795,"longitude":8.938285466212193}]}]}'
        var data = JSON.parse(jsondata)

        var i,j,lat,lon,p


        clearAll()
        for(i = 0; i < data.paths.length; i++){
            switch(data.paths[i].name){
            /*
            case "RectPath":
                map.removeMapItem(rect_list)
                if (currentState === generalState.empty){
                    currentState = generalState.rect
                    if (rect_cur)
                        rect_cur.end.disconnect(endRect)
                    rect_cur = rectComponent.createObject(map_component, {offset:data.paths[i].offset, angle:data.paths[i].angle, debug_c: overlay_canvas, editCircle: editCircle})
                    rect_list.push(rect_cur)
                    map.addMapItem(rect_cur)

                    rect_cur.line.color = "#33cc33"
                    for(j = 0; j < data.paths[i].values.length; j++){
                        lat = data.paths[i].values[j].latitude
                        lon = data.paths[i].values[j].longitude

                        p = QtPositioning.coordinate(lat,lon)
                        rect_cur.addCoordinate(p)
                    }

                    //rect_cur.generate_path()
                    //rect_cur.draw_path()
                    //rect_cur.generate_nurbs()
                    rect_cur.end()

                    rect_cur.end.connect(endRect)
                    rect_list.push(rect_cur)
                    endRect()
                }
                break;
                */
            case "PolyPath":
                if (currentState === generalState.empty){
                    currentState = generalState.poly
                    poly_cur = polyComponent.createObject(map_component,
                        {
                            offset: data.paths[i].offset,
                            angle:  data.paths[i].angle
                        })

                    poly_list.push(poly_cur)
                    map.addMapItem(poly_cur)
                    poly_cur.deserialize(data.paths[i].values)
                    poly_cur.draw_deferred()

                }
                break

            case "PointPath":

                map.removeMapItem(path_list)

                if (currentState === generalState.empty){
                    currentState = generalState.path
                    if (path_cur)
                        path_cur.end.disconnect(endPath)
                    path_cur = pathComponent.createObject(map_component)
                    path_list.push(path_cur)
                    map.addMapItem(path_cur)

                    path_cur.line.color = "#33cc33"
                    for(j = 0; j < data.paths[i].values.length; j++){
                        lat = data.paths[i].values[j].latitude
                        lon = data.paths[i].values[j].longitude

                        p = QtPositioning.coordinate(lat,lon)
                        path_cur.addCoordinate(p)
                    }

                    path_cur.end.connect(endPath)
                    endPath()
                    path_list.push(path_cur)
                }
                break;
            case "SecurityPoly":
                map.removeMapItem(polysec_cur)

                if (currentState === generalState.empty){
                    currentState = generalState.polysec
                    if (polysec_cur)
                        polysec_cur.end.disconnect(endPolySec)
                    polysec_cur = polysecComponent.createObject(map_component)
                    map.addMapItem(polysec_cur)

                    polysec_cur.line.color = "#161fc4"
                    for(j = 0; j < data.paths[i].values.length; j++){
                        lat = data.paths[i].values[j].latitude
                        lon = data.paths[i].values[j].longitude

                        p = QtPositioning.coordinate(lat,lon)
                        polysec_cur.addCoordinate(p)
                    }
                    polysec_cur.close_polygon()
                    polysec_cur.end.connect(endPolySec)
                    endPolySec()
                    polysec_cur.closed = true
                }
                break

            }
        }

    }

    //TODO -> save to a file
    function savePath(){
        var i
        var j

        var l = []

        var all_paths = {
            security_box: null,
            paths: []
        }
        /*


        var p_i
        var single_path = {};
        single_path.name = 'paths'
        single_path.values = []
        //Add all the rectangular paths
        for(i = 0; i < rect_list.length; i++){
            single_path = {}
            single_path.name = 'RectPath'
            single_path.offset = rect_list[i].offset
            single_path.angle = rect_list[i].angle
            single_path.values = []
            for(j = 0; j < rect_list[i].pathLength(); j++){
                p_i = rect_list[i].coordinateAt(j)
                l = {}
                l.latitude = p_i.latitude
                l.longitude = p_i.longitude
                single_path.values.push(l)
            }
            all_paths.paths.push(single_path)
        }
        */

        //Add all the polygonal paths
        for(i = 0; i < poly_list.length; i++){
            all_paths.paths.push(path_list[i].serialize())
        }

        /*
        //Add all the point paths
        for(i = 0; i < path_list.length; i++){
            all_paths.paths.push(path_list[i].serialize())
        }

        if(polysec_cur){
            single_path = {}
            single_path.name = 'SecurityPoly'
            single_path.values = []
            for(j = 0; j < polysec_cur.pathLength(); j++){
                p_i = polysec_cur.coordinateAt(j)
                l = {}
                l.latitude = p_i.latitude
                l.longitude = p_i.longitude
                single_path.values.push(l)

            }
            all_paths.paths.push(single_path)
        }
        */

        console.log(JSON.stringify(all_paths))

        //saveFile(path_file,JSON.stringify(all_paths))
    }

    compass.transform: [
        Rotation {
            origin.x: compass.width / 2
            origin.y: compass.height / 2
            angle: 180.0 - map.bearing
        },
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: map.tilt
            axis.x: 1
            axis.y: 0
            axis.z: 0
        }
    ]

    ulisseIcon.transform: [
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: fbkUpdater.ulisse_yaw_deg - map.bearing
        },
        Rotation {
            origin.x: ulisseIcon.sourceItem.width / 2
            origin.y: ulisseIcon.sourceItem.height / 2
            angle: map.tilt
            axis.x: 1
            axis.y: 0
            axis.z: 0
        }
    ]

    onCenterChanged: {
        ruler.rulerTimer.restart()
    }

    onZoomLevelChanged: {
        ruler.rulerTimer.restart()
    }

    onWidthChanged: {
        ruler.rulerTimer.restart()
    }

    onHeightChanged: {
        ruler.rulerTimer.restart()
    }

    markerIcon.onCoordinateChanged: {
        mapsidebar.markerText = "%1, %2".arg(marker_coords.latitude).arg(
                    marker_coords.longitude)
    }

    Timer {
        interval: 500
        running: true
        repeat: true
        onTriggered: {
            if (ulissePath.firstRun) {
                ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                ulissePath.firstRun = false
            }
            // To reduce the line density (and avoid to overload the gui)
            // we add a new point only every 1.0 meter
            var lastCoord = ulissePath.coordinateAt(
                        ulissePath.pathLength() - 1)
            var distToNext = lastCoord.distanceTo(fbkUpdater.ulisse_pos)
            if (distToNext > 1.0) {
                ulissePath.addCoordinate(fbkUpdater.ulisse_pos)
                if (ulissePath.pathLength() > ulissePath.traceSize) {
                    ulissePath.removeCoordinate(0)
                }
            }
        }
    }

    function clearUlisseTrace() {
        ulissePath.path = []
        ulissePath.firstRun = true

        //TODO -> remove
        //savePath()
        loadPath()
    }

    //TODO -> finish (delete also the lines/circles) and add to a button
    function clearAll(){
        //clearUlisseTrace()
        var i
        /*
        for(i = 0; i < rect_list.length; i++){
            map.removeMapItem(rect_list[i])
        }*/
        for(i = 0; i < poly_list.length; i++){
            poly_list[i].deregister_map_items()
            map.removeMapItem(poly_list[i])
            poly_list = []
            poly_cur = null

        }
        /*
        for(i = 0; i < path_list.length; i++){
            map.removeMapItem(path_list[i])
        }
        */
        polysec_cur.clear_path()
    }
}
