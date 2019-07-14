.pragma library

var QtPositioning
var QtLocation

function init_lib(qt_positioning){
    QtPositioning = qt_positioning
}

function coords_centroid(coords){
    var lat = 0
    var lon = 0
    for (var i=0; i<coords.length; i++){
        lat += coords[i].latitude
        lon += coords[i].longitude
    }
    return QtPositioning.coordinate(lat/coords.length, lon/coords.length)
}

function map_to_euclidean(coordinates, centroid, lam, lom){
    var points = []
    for (var i = 0; i<coordinates.length; i++){
        points.push(Qt.point((centroid.longitude - coordinates[i].longitude) * lom,
                             (centroid.latitude  - coordinates[i].latitude)  * lam))
    }
    return points
}

function point_euclidean_to_map(x, y, centroid, lam, lom){
    return QtPositioning.coordinate(centroid.latitude - y/lam, centroid.longitude - x/lom)
}

function euclidean_to_map(ii, centroid, lam, lom){
    var cc = []
    for (var i=0; i<ii.length; i++){
        cc.push([point_euclidean_to_map(ii[i][0].x, ii[i][0].y, centroid, lam ,lom),
                 point_euclidean_to_map(ii[i][1].x, ii[i][1].y, centroid, lam ,lom)])
    }
    return cc
}

function map_to_canvas(cc, map, canvas){
    var pp = []
    for (var i=0; i<cc.length; i++){
        var a = toCanvasCoordinates(map.fromCoordinate(cc[i][0], false), map, canvas)
        var b = toCanvasCoordinates(map.fromCoordinate(cc[i][1], false), map, canvas)
        pp.push([a,b])
    }
    return pp
}

function relative_zoom_pixel_ratio(map, zoom){
    //assuming tilt = 0, otherwise nonsense
    var z = map.zoomLevel
    map.zoomLevel = zoom
    var c0 = map.toCoordinate(Qt.point(1,1), false)
    var c1 = map.toCoordinate(Qt.point(2,1), false)
    map.zoomLevel = z
    var p0 = map.fromCoordinate(c0, false)
    var p1 = map.fromCoordinate(c1, false)
    return 1.0/(p1.x-p0.x)
}

function side_angle(side){
    var a = rad_to_deg(Math.atan(slope(side[0], side[1])))
    return (a>=0) ? a : a+90
}

function intersections(point, angle, sides){
    var l0 = to_homogeneous_line(point, deg_to_rad(angle))

    var ii = []
    for (var i=0; i<sides.length; i++){
        var s = sides[i]
        var li = to_homogeneous_line(s[0], Math.atan(slope(s[0], s[1])))

        var intersection = l0.crossProduct(li)
        if (intersection.z !== 0){/*not parallel*/
            var pi = from_homogeneous_point(intersection)
            if (point_in_box(pi, s[0], s[1]))
                ii.push(pi)
        }
    }
    return ii
}


function set_points_clockwise(points){
    if (three_point_direction(points[0],points[1],points[2]) === 1)
        points.reverse()
    return points
}

function find_interesting_features(points){
    var t = find_top(points)
    var pti = t[1]
    var pt = t[0]
    var pl = points[pti > 0 ? pti-1 : points.length-1]

    var sides = []
    for (var i=0, idx_a=pti-1, idx_b; i<points.length; i++){
        idx_a = add_and_wrap(idx_a, points.length)
        idx_b = add_and_wrap(idx_a, points.length)
        sides.push([points[idx_a], points[idx_b]])
    }
    var upper_side = sides[sides.length-1]
    return {
        point_top: pt,
        point_before: pl,
        upper_side: upper_side,
        sides: sides
    }
}

function init_canvas(canvas, map, width, height, lat, lon, px_multiplier){
    canvas.canvasWidth = width*px_multiplier
    canvas.canvasHeight = height*px_multiplier
    canvas.zoomLevel = map.maximumZoomLevel
    canvas.coordinate = QtPositioning.coordinate(lat, lon)
    canvas.canvasAngle = map.bearing
    canvas.multiplier = px_multiplier
    canvas.canvasCtx.lineWidth = 6
    canvas.canvasCtx.lineJoin = "bevel"
}

function draw_line(ctx,x1,y1,x2,y2){
    ctx.beginPath()
    ctx.moveTo(x1,y1)
    ctx.lineTo(x2,y2)
    ctx.closePath()
    ctx.stroke()
}

function draw_bounding_rect(map, canvas, w, h){
    var ctx = canvas.canvasCtx
    ctx.fillStyle = Qt.rgba(0, 0.4, 0, 0.2)
    ctx.fillRect(0, 0, w, h)
    canvas.requestPaint()
}

function find_intersections(angle, pt, pl, upper_side, sides, offset){
    var au_deg = 90-angle
    var at_deg = side_angle(upper_side)
    var offset_on_t = offset/Math.cos(deg_to_rad(90-au_deg))

    var times=0
    var ii1=[]
    while (true){
        ++times
        var p = interpolate(pt, pl, offset_on_t, times)
        var _i = intersections(p, at_deg-au_deg, sides) //NB two intersections always if a convex polygon
        if (_i.length < 2) break
        if (_i[0].x < _i[1].x)
            ii1.push([_i[0], _i[1]])
        else
            ii1.push([_i[1], _i[0]])
    }

    var pr = Qt.point(2*pt.x - pl.x,
                      2*pt.y - pl.y)
    times = -1
    var ii2=[]
    while (true){
        ++times

        var p = interpolate(pt, pr, offset_on_t, times)
        var _i = intersections(p, at_deg-au_deg, sides) //NB two intersections always if a convex polygon
        if (_i.length < 2) break
        if (_i[0].x < _i[1].x)
            ii2.push([_i[0], _i[1]])
        else
            ii2.push([_i[1], _i[0]])
    }

    return ii2.reverse().concat(ii1)
}

function rectify_dense_winding(ii){
    var a = 0, b = 1
    for (var i = 0; i < ii.length-1; i++){
        var rr = rectify_parallelogram_side(ii[i][a], ii[i][b], ii[i+1][b], ii[i+1][a])
        ii[i][b] = rr[1]
        ii[i+1][b] = rr[2]
        a = flip(a)
        b = flip(b)
    }
    return ii
}

function draw_path_lines(canvas, pp){
    var ctx = canvas.canvasCtx
    for (var i = 0; i < pp.length; i++){
        var p0 = pp[i][0]
        var p1 = pp[i][1]
        draw_line(ctx, p0.x, p0.y, p1.x, p1.y)
        canvas.requestPaint()
    }
}

function draw_path_semicircles(canvas, pp){
    var ctx = canvas.canvasCtx
    for (var i = 0; i < pp.length-1; i++){
        var dir = (i+1)%2
        var p0 = pp[i][dir]
        var p1 = pp[i+1][dir]
        ctx.beginPath()
        ctx.moveTo(p1.x, p1.y)
        ctx.arc((p0.x+p1.x)/2,
                (p0.y+p1.y)/2,
                distance(p0, p1)/2,
                Math.atan2(p1.y-p0.y, p1.x-p0.x),
                Math.atan2(p1.y-p0.y, p1.x-p0.x) + Math.PI,
                dir === 1)
        ctx.moveTo(p1.x, p1.y)
        ctx.closePath()
        ctx.stroke()
    }
    canvas.requestPaint()
}

function roundNumber(number, digits)
{
    var multiple = Math.pow(10, digits);
    return Math.round(number * multiple) / multiple;
}

function formatTime(sec)
{
    var value = sec
    var seconds = value % 60
    value /= 60
    value = (value > 1) ? Math.round(value) : 0
    var minutes = value % 60
    value /= 60
    value = (value > 1) ? Math.round(value) : 0
    var hours = value
    if (hours > 0) value = hours + "h:"+ minutes + "m"
    else value = minutes + "min"
    return value
}

function formatDistance(meters)
{
    var dist = Math.round(meters)
    if (dist > 1000 ){
        if (dist > 100000){
            dist = Math.round(dist / 1000)
        }
        else{
            dist = Math.round(dist / 100)
            dist = dist / 10
        }
        dist = dist + " km"
    }
    else{
        dist = dist + " m"
    }
    return dist
}

function project(p0,m,p1){
    var p2 = Qt.point(0,0);
    if ( (Math.abs(m) === 16331239353195370) || (Math.abs(m) === Infinity)){
        p2.x = p0.x
        p2.y = p1.y
    } else if (m === 0){
        p2.x = p1.x
        p2.y = p0.y
    } else {
        var m_perp = -1/m
        p2.x = (m*p0.x - m_perp*p1.x + p1.y - p0.y)/(m-m_perp)
        p2.y = m*(p2.x - p0.x) + p0.y
    }
    return p2
}

function distance(p1,p2){
    return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y,2))
}

function interpolate(from, to, offset, times){
    var a = Math.atan2(from.y - to.y, from.x - to.x)
    var ox = offset * Math.cos(a) * times
    var oy = offset * Math.sin(a) * times
    return Qt.point(from.x-ox, from.y-oy)
}

function point_in_box(p, p1, p2){
    return ( (p1.x <= p.x && p.x <= p2.x)
          || (p2.x <= p.x && p.x <= p1.x)
          || (p1.y <= p.y && p.y <= p2.y)
          || (p2.y <= p.y && p.y <= p1.y))
}

function to_homogeneous_line(point, angle){
    var m = Math.tan(angle)
    if ( (Math.abs(m) === 16331239353195370) || (Math.abs(m) === Infinity))
        return Qt.vector3d(1, 0, -point.x)
    if (m === 0)
        return Qt.vector3d(0, 1, -point.y)
    var q = point.y - m * point.x
    return Qt.vector3d(m, -1, q)
}

function from_homogeneous_point(p){
    return Qt.point(p.x/p.z, p.y/p.z)
}

function slope(p0, p1){
    return (p0.y - p1.y)/(p0.x - p1.x)
}

function rectify_parallelogram_side(p0,p1,p2,p3){
    //p0-p1, p2-p3 must be parallel!
    //either p1 or p2 will be translated in order to create 90 degrees angles between p1 and p2, p2 and p3
    var _a = p1.x-p0.x
    var _b = p1.y-p0.y
    var _c = p1.x-p2.x
    var _d = p1.y-p2.y
    var a = Math.acos((_a*_c+_b*_d)/(Math.sqrt(_a*_a+_b*_b)*Math.sqrt(_c*_c+_d*_d)))
    var m = slope(p0,p1)
    if (a > Math.PI/2)
        p1 = project(p0, m, p2)
    else
        p2 = project(p3, m, p1)
    return [p0, p1, p2, p3]
}

function find_top(points){
    var pt = points[0]
    var t = pt.y
    for (var i = 1; i<points.length; i++){
        if (points[i].y >= t){
            t = points[i].y;
            pt = points[i]
        }
    }
    var pti = points.indexOf(pt)
    if (pti === points.length-1 && pt.y === points[0].y && pt.x < points[0].x)
        return [points[0], 0]
    return [pt, pti]
}

function flip(x){
    return (x===0) ? 1 : 0
}

function add_and_wrap(x, mod){
    return (x+1 < mod) ? x+1 : 0
}

function rotate(p, a){
    return Qt.point(p.x*Math.cos(a)-p.y*Math.sin(a), p.x*Math.sin(a)+p.y*Math.cos(a))
}

function toCanvasCoordinates(point, map, canvas){
    var o = map.fromCoordinate(canvas.coordinate, false)
    return Qt.point((point.x-o.x)*canvas.multiplier, (point.y-o.y)*canvas.multiplier)
}

function fromCanvasCoordinates(point, map, canvas){
    var o = map.fromCoordinate(canvas.coordinate)
    return Qt.point(point.x+o.x, point.y+o.y)
}

function deg_to_rad(deg){
    return (Math.PI/180.0) * deg
}

function rad_to_deg(rad){
    return (180.0/Math.PI) * rad
}

function three_point_direction(p1,p2,p3){
    var a1 = rad_to_deg(Math.atan2(p1.y-p2.y, p1.x-p2.x))
    var a2 = rad_to_deg(Math.atan2(p2.y-p3.y, p2.x-p3.x))
    a1 = a1-360*Math.floor(a1/360)
    a2 = a2-360*Math.floor(a2/360)
    return ((a1<a2 && (a2-a1)<180) || (a1>a2 && (a1-a2)>180)) ? 1/*cc*/ : 2/*c*/
}

function lat_to_m_coeff(lat){
    lat = deg_to_rad(lat)
    return 111132.92 - 559.82*Math.cos(2*lat) + 1.175*Math.cos(4*lat) - 0.0023*Math.cos(6*lat)
}

function lon_to_m_coeff(lon){
    lon = deg_to_rad(lon)
    return 111412.84*Math.cos(lon) - 93.5*Math.cos(3*lon) + 0.118*Math.cos(5*lon)
}


function serialize(object, maxDepth) {
    function _processObject(object, maxDepth, level) {
        var output = []
        var pad = "  "
        if (maxDepth == undefined) {
            maxDepth = -1
        }
        if (level == undefined) {
            level = 0
        }
        var padding = new Array(level + 1).join(pad)

        output.push((Array.isArray(object) ? "[" : "{"))
        var fields = []
        for (var key in object) {
            var keyText = Array.isArray(object) ? "" : ("\"" + key + "\": ")
            if (typeof (object[key]) == "object" && key != "parent" && maxDepth != 0) {
                var res = _processObject(object[key], maxDepth > 0 ? maxDepth - 1 : -1, level + 1)
                fields.push(padding + pad + keyText + res)
            } else {
                fields.push(padding + pad + keyText + "\"" + object[key] + "\"")
            }
        }
        output.push(fields.join(",\n"))
        output.push(padding + (Array.isArray(object) ? "]" : "}"))

        return output.join("\n")
    }

    return _processObject(object, maxDepth)
}

function generate_nurb_circle(p0, p3, dir){
    var dist = distance(p0, p3)
    var angle = Math.tan(1/-slope(p0, p3))
    if (dir === 1){
        var p1 = Qt.point(p0.x + Math.cos(angle)*dist, p0.y + Math.sin(angle)*dist)
        var p2 = Qt.point(p3.x + Math.cos(angle)*dist, p3.y + Math.sin(angle)*dist)
    } else {
        var p1 = Qt.point(p0.x - Math.cos(angle)*dist, p0.y - Math.sin(angle)*dist)
        var p2 = Qt.point(p3.x - Math.cos(angle)*dist, p3.y - Math.sin(angle)*dist)
    }
    /*
    console.log(Helper.serialize({
        "discretization": 100,
        "degree": 3,
        "controlPoints": [p0.x, p0.y, 0, 1, p1.x, p1.y, 0, 1/3.0, p2.x, p2.y, 0, 1/3.0, p3.x, p3.y, 0, 1],
        "knots": [0, 0, 0, 0, 1, 1, 1, 1]
    }, 10))
    */
    return {
        degree: 3,
        points: [[p0.x, p0.y], [p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y]],
        weigths: [1, 1/3.0, 1/3.0, 1],
        knots: [0, 0, 0, 0, 1, 1, 1, 1]
    }
}

function generate_nurb_line(p0, p1){
    /*
    console.log(Helper.serialize({
        "discretization": 100,
        "degree": 1,
        "controlPoints": [p0.x, p0.y, 0, 1, p1.x, p1.y, 0, 1],
        "knots": [0, 0, 1, 1]
    }, 10))
    */
    return {
        degree: 1,
        points: [[p0.x, p0.y], [p1.x, p1.y]],
        weigths: [1, 1],
        knots: [0, 0, 1, 1]
    }
}

function shape_geo_limits(coords){
    var min_x = coords[0].longitude
    var max_x = coords[0].longitude
    var min_y = coords[0].latitude
    var max_y = coords[0].latitude
    for (var i=1; i<coords.length; i++){
        var p = coords[i]
        if (p.longitude<min_x) min_x=p.longitude
        else if (p.longitude>max_x) max_x=p.longitude
        if (p.latitude<min_y) min_y=p.latitude
        else if (p.latitude>max_y) max_y=p.latitude
    }
    return {
        min_lon: min_x,
        max_lon: max_x,
        min_lat: min_y,
        max_lat: max_y
    }
}

function shape_px_dimensions(limits, map){
    var p0 = map.fromCoordinate(QtPositioning.coordinate(limits.min_lat, limits.min_lon), false)
    var p1 = map.fromCoordinate(QtPositioning.coordinate(limits.min_lat, limits.max_lon), false)
    var p2 = map.fromCoordinate(QtPositioning.coordinate(limits.max_lat, limits.min_lon), false)
    var w = distance(p0, p1)
    var h = distance(p0, p2)
    return [w,h]
}
