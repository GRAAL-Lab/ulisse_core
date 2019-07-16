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

function point_screen2canvas(point, map, canvas){
    var o = map.fromCoordinate(canvas.coordinate, false)
    return rotate(Qt.point((point.x-o.x)*canvas.multiplier, (point.y-o.y)*canvas.multiplier), deg_to_rad(map.bearing))
}

function point_canvas2screen(point, map, canvas){
    var o = map.fromCoordinate(canvas.coordinate)
    return Qt.point(point.x/canvas.multiplier+o.x, point.y/canvas.multiplier+o.y)
}

function point_map2euclidean(latitude, longitude, centroid, lam, lom){
    return Qt.point((centroid.longitude - longitude) * lom,
                    (centroid.latitude  - latitude)  * lam)
}

function point_euclidean2map(x, y, centroid, lam, lom){
    return QtPositioning.coordinate(centroid.latitude - y/lam, centroid.longitude - x/lom)
}

function points_map2euclidean(coordinates, centroid, lam, lom){
    var points = []
    for (var i = 0; i<coordinates.length; i++)
        points.push(point_map2euclidean(coordinates[i].latitude, coordinates[i].longitude, centroid, lam, lom))
    return points
}

function points_euclidean2map(points, centroid, lam, lom){
    var coordinates = []
    for (var i = 0; i<points.length; i++)
        coordinates.push(point_euclidean2map(points[i].x, points[i].y, centroid, lam, lom))
    return coordinates
}

function point_couples_euclidean2map(ii, centroid, lam, lom){
    var cc = []
    for (var i=0; i<ii.length; i++)
        cc.push(points_euclidean2map(ii[i], centroid, lam ,lom))
    return cc
}

function point_couples_map2canvas(cc, map, canvas){
    var pp = []
    for (var i=0; i<cc.length; i++){
        var a = point_screen2canvas(map.fromCoordinate(cc[i][0], false), map, canvas)
        var b = point_screen2canvas(map.fromCoordinate(cc[i][1], false), map, canvas)
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

function path_bearing(c0, c1){
    var y = Math.sin(c1.longitude-c0.longitude) * Math.cos(c1.latitude);
    var x = Math.cos(c0.latitude)*Math.sin(c1.latitude) -
            Math.sin(c0.latitude)*Math.cos(c1.latitude)*Math.cos(c1.longitude-c0.longitude);
    return rad_to_deg(Math.atan2(y, x));
}


function nearest_geo_intersection(cp1, a1, cp2, a2){
    var f = function (i,r){
        if (i !== null && !isNaN(i.latitude) && !isNaN(i.longitude))
            r.push(i)
    }

    var m = function (r, cpc){
        if (r.length > 0){
            var min_d = r[0].distanceTo(cpc)
            var min_c = r[0]
            for (var i = 1; i<r.length; i++)
                if (r[i].distanceTo(cpc) < min_d)
                    min_c = r[i]
            return min_c
        } else return null
    }

    var r = []
    f(geo_intersection(cp1, a1, cp2, a2),r)
    f(geo_intersection(cp1, a1, cp2, a2+180),r)
    f(geo_intersection(cp1, a1+180, cp2, a2),r)
    f(geo_intersection(cp1, a1+180, cp2, a2+180),r)
    return m(r, cp2)
}

function geo_intersection(p1, brng1, p2, brng2) {
    // see www.edwilliams.org/avform.htm#Intersection

    var φ1 = deg_to_rad(p1.latitude), λ1 = deg_to_rad(p1.longitude);
    var φ2 = deg_to_rad(p2.latitude), λ2 = deg_to_rad(p2.longitude);
    var θ13 = deg_to_rad(brng1), θ23 = deg_to_rad(brng2);
    var Δφ = φ2 - φ1, Δλ = λ2 - λ1;

    // angular distance p1-p2
    var δ12 = 2 * Math.asin(Math.sqrt(Math.sin(Δφ/2) * Math.sin(Δφ/2)
        + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ/2) * Math.sin(Δλ/2)));
    if (Math.abs(δ12) < Number.EPSILON) return QtPositioning.coordinate(p1.latitude, p1.longitude); // coincident points

    // initial/final bearings between points
    var cosθa = (Math.sin(φ2) - Math.sin(φ1)*Math.cos(δ12)) / (Math.sin(δ12)*Math.cos(φ1));
    var cosθb = (Math.sin(φ1) - Math.sin(φ2)*Math.cos(δ12)) / (Math.sin(δ12)*Math.cos(φ2));
    var θa = Math.acos(Math.min(Math.max(cosθa, -1), 1)); // protect against rounding errors
    var θb = Math.acos(Math.min(Math.max(cosθb, -1), 1)); // protect against rounding errors

    var θ12 = Math.sin(λ2-λ1)>0 ? θa : 2*Math.PI-θa;
    var θ21 = Math.sin(λ2-λ1)>0 ? 2*Math.PI-θb : θb;

    var α1 = θ13 - θ12; // angle 2-1-3
    var α2 = θ21 - θ23; // angle 1-2-3

    if (Math.sin(α1) == 0 && Math.sin(α2) == 0) return null; // infinite intersections
    if (Math.sin(α1) * Math.sin(α2) < 0) return null;        // ambiguous intersection (antipodal?)

    var cosα3 = -Math.cos(α1)*Math.cos(α2) + Math.sin(α1)*Math.sin(α2)*Math.cos(δ12);

    var δ13 = Math.atan2(Math.sin(δ12)*Math.sin(α1)*Math.sin(α2), Math.cos(α2) + Math.cos(α1)*cosα3);

    var φ3 = Math.asin(Math.sin(φ1)*Math.cos(δ13) + Math.cos(φ1)*Math.sin(δ13)*Math.cos(θ13));

    var Δλ13 = Math.atan2(Math.sin(θ13)*Math.sin(δ13)*Math.cos(φ1), Math.cos(δ13) - Math.sin(φ1)*Math.sin(φ3));
    var λ3 = λ1 + Δλ13;

    var lat = rad_to_deg(φ3);
    var lon = rad_to_deg(λ3);

    return QtPositioning.coordinate(lat, lon);
}


function point_in_box(p, p1, p2){
    var o = 0.1
    return ( (p1.x-o <= p.x && p.x <= p2.x+o && p1.y-o <= p.y && p.y <= p2.y+o)
          || (p2.x-o <= p.x && p.x <= p1.x+o && p2.y-o <= p.y && p.y <= p1.y+o)
          || (p1.x-o <= p.x && p.x <= p2.x+o && p2.y-o <= p.y && p.y <= p1.y+o)
          || (p2.x-o <= p.x && p.x <= p1.x+o && p1.y-o <= p.y && p.y <= p2.y+o))
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
            //console.log(JSON.stringify(s[0]))
            //console.log(JSON.stringify(pi))
            //console.log(JSON.stringify(s[1]))
            if (point_in_box(pi, s[0], s[1]))
                ii.push(pi)
        }
    }
    return ii
}

function find_intersections(angle, points, sides, offset, lam, lom){
    var step_points = []

    var a_perp = Math.atan(-Math.pow(lam/lom,2)/Math.tan(deg_to_rad(angle)))
    var p_dir = Qt.point(points[1].x + Math.cos(a_perp)*offset,points[1].y + Math.sin(a_perp)*offset)
    var p_start = interpolate(points[1], p_dir, -offset/2, 1)


    var times=0
    var ii1=[]
    while (true){
        ++times
        var p = interpolate(p_start, p_dir, -offset, times)
        step_points.push(p)
        var _i = intersections(p, angle, sides) //NB two intersections always if a convex polygon
        if (_i.length < 2) break
        if (_i[0].x < _i[1].x || (_i[0].x === _i[1].x && _i[0].y < _i[1].y))
            ii1.push([_i[0], _i[1]])
        else
            ii1.push([_i[1], _i[0]])
    }

    times = -1
    var ii2=[]
    while (true){
        ++times
        var p = interpolate(p_start, p_dir, offset, times)
        step_points.push(p)
        var _i = intersections(p, angle, sides) //NB two intersections always if a convex polygon
        if (_i.length < 2) break
        if (_i[0].x < _i[1].x || (_i[0].x === _i[1].x && _i[0].y < _i[1].y))
            ii2.push([_i[0], _i[1]])
        else
            ii2.push([_i[1], _i[0]])
    }

    return {
        intersections: ii2.reverse().concat(ii1),
        step_points: step_points
    }
}


function set_points_clockwise(points){
    if (three_point_direction(points[0],points[1],points[2]) === 1)
        points.reverse()
    return points
}

function set_point_from_upper(points){
    var t = find_top(points)
    var pti = t[1]
    return points.slice(pti).concat(points.slice(0,pti))
}

function make_sides(points){
    var r = []
    for (var i=0; i<points.length-1; i++)
        r.push([points[i], points[i+1]])
    r.push([points[points.length-1], points[0]])
    return r
}

function find_interesting_features(points){
    var t = find_top(points)
    var pti = t[1]
    var pt = t[0]
    var pli = sub_and_wrap(pti, points.length)
    var pl = points[pli]

    var sides = []
    for (var i=0, idx_a=pti-1, idx_b; i<points.length; i++){
        idx_a = add_and_wrap(idx_a, points.length)
        idx_b = add_and_wrap(idx_a, points.length)
        sides.push([points[idx_a], points[idx_b]])
    }
    var upper_side = sides[0]
    return {
        point_top: pt,
        point_before: pl,
        upper_side: upper_side,
        sides: sides
    }
}



function rectify_parallelogram_side(p0,p1,p2,p3, ratio){
    //p0-p1, p2-p3 must be parallel!
    //either p1 or p2 will be translated in order to create 90 degrees angles between p1 and p2, p2 and p3
    var _a = p1.x-p0.x
    var _b = p1.y-p0.y
    var _c = p1.x-p2.x
    var _d = p1.y-p2.y
    var a = angle_between(_a,_b, _c, _d)
    var m = slope(p0,p1)
    if (a > Math.PI/2)
        p1 = project(p0, m, p2, ratio)
    else
        p2 = project(p3, m, p1, ratio)
    var result = [p0, p1, p2, p3]
    console.log(p0.x + "," + p0.y)
    console.log(p1.x + "," + p1.y)
    console.log(p2.x + "," + p2.y)
    console.log(p3.x + "," + p3.y)
    return result
}

function rectify_dense_winding(ii, ratio){
    var a = 0, b = 1
    for (var i = 0; i < ii.length-1; i++){
        var rr = rectify_parallelogram_side(ii[i][a], ii[i][b], ii[i+1][b], ii[i+1][a], ratio)
        ii[i][b] = rr[1]
        ii[i+1][b] = rr[2]
        a = flip(a)
        b = flip(b)
    }
    return ii
}

function intersect_two_lines(p0,m0,p1,m1){
    var l0 = to_homogeneous_line(p0, Math.atan(m0))
    var l1 = to_homogeneous_line(p1, Math.atan(m1))

    var intersection = l0.crossProduct(l1)
    return from_homogeneous_point(intersection)
}

function project(p0,m,p1,ratio){
    var p2 = Qt.point(0,0);
    if ( (Math.abs(m) === 16331239353195370) || (Math.abs(m) === Infinity)){
        p2.x = p0.x
        p2.y = p1.y
    } else if (m === 0){
        p2.x = p1.x
        p2.y = p0.y
    } else {
        var _m = m
        var _m_perp = -1/m
        var m = _m*ratio
        var m_perp = _m_perp*ratio
        p2.x = (m*p0.x - m_perp*p1.x + p1.y - p0.y)/(m-m_perp)
        p2.y = m*(p2.x - p0.x) + p0.y
    }
    return p2
}

function distance(p1,p2){
    return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y,2))
}

function interpolate(from, to, offset, times){
    var a = Math.atan2(to.y - from.y,to.x - from.x)
    var ox = offset * times * Math.cos(a)
    var oy = offset * times * Math.sin(a)
    return Qt.point(from.x+ox, from.y+oy)
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

function angle_between(/*vector 1*/_a,_b, /*vector 2*/_c,_d){
    return Math.acos((_a*_c+_b*_d)/(Math.sqrt(_a*_a+_b*_b)*Math.sqrt(_c*_c+_d*_d)))
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

function sub_and_wrap(x, mod){
    return (x-1 >= 0) ? x-1 : mod-1
}

function rotate(p, a){
    var x = p.x*Math.cos(a)-p.y*Math.sin(a)
    var y = p.x*Math.sin(a)+p.y*Math.cos(a)
    return Qt.point(x,y)
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
    return {
        degree: 3,
        points: [[p0.x, p0.y], [p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y]],
        weigths: [1, 1/3.0, 1/3.0, 1],
        knots: [0, 0, 0, 0, 1, 1, 1, 1]
    }
}

function generate_nurb_line(p0, p1){
    return {
        degree: 1,
        points: [[p0.x, p0.y], [p1.x, p1.y]],
        weigths: [1, 1],
        knots: [0, 0, 1, 1]
    }
}

function generate_nurb_broken_line(pp){
    var points = []
    var weights = []
    var knots = [0]
    for (var i = 0; i < pp.length; i++){
        points.push([pp[i].x, pp[i].y])
        knots.push((i+1)/pp.length)
        weights.push(1)
    }
    return {
        degree: 1,
        points: points,
        weigths: weights,
        knots: knots
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

function init_canvas(canvas, map, width, height, lat, lon, px_multiplier){
    canvas.canvasWidth = width*px_multiplier
    canvas.canvasHeight = height*px_multiplier
    canvas.zoomLevel = map.maximumZoomLevel
    canvas.coordinate = QtPositioning.coordinate(lat, lon)
    canvas.canvasAngle = 0
    canvas.multiplier = px_multiplier
    canvas.canvasCtx.lineWidth = 6
    canvas.canvasCtx.lineJoin = "bevel"
}

function draw_line(ctx,x1,y1,x2,y2){
    ctx.strokeStyle = "#ff0000"
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

function draw_path_lines(canvas, pp){
    var ctx = canvas.canvasCtx
    for (var i = 0; i < pp.length; i++){
        var p0 = pp[i][0]
        var p1 = pp[i][1]
        draw_line(ctx, p0.x, p0.y, p1.x, p1.y)
        canvas.requestPaint()
    }
}

function draw_semicircle(ctx, p0, p1, dir){
    ctx.strokeStyle = "#0000ff"
    ctx.beginPath()
    ctx.moveTo(p1.x, p1.y)
    ctx.arc((p0.x+p1.x)/2,
            (p0.y+p1.y)/2,
            distance(p0, p1)/2,
            Math.atan2(p1.y-p0.y, p1.x-p0.x),
            Math.atan2(p1.y-p0.y, p1.x-p0.x) + Math.PI,
            dir === 0)
    ctx.moveTo(p1.x, p1.y)
    ctx.closePath()
    ctx.stroke()
}

function draw_circle(ctx, p0, r, color){
    ctx.strokeStyle = color
    ctx.beginPath()
    ctx.moveTo(p0.x, p0.y)
    ctx.arc(p0.x,
            p0.y,
            r,
            0,
            2*Math.PI)
    ctx.moveTo(p0.x, p0.y)
    ctx.closePath()
    ctx.stroke()
}

function draw_path_semicircles(canvas, pp){
    var ctx = canvas.canvasCtx
    for (var i = 0; i < pp.length-1; i++){
        var dir = (i+1)%2
        var p0 = pp[i][dir]
        var p1 = pp[i+1][dir]
        draw_semicircle(ctx,p0,p1,dir)
    }
    canvas.requestPaint()
}


function roundNumber(number, digits){
    var multiple = Math.pow(10, digits);
    return Math.round(number * multiple) / multiple;
}

function formatTime(sec){
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

function formatDistance(meters){
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
