
var QtPositioning
var QtLocation

function init_lib(qt_positioning) {
    QtPositioning = qt_positioning
}

function coordinate_deep_copy(c){
    return QtPositioning.coordinate(c.latitude, c.longitude)
}

function set_bulk_opacity(bulk, val){
    for (var i in bulk)
        bulk[i].opacity = val
}

function coords_centroid(coords) {
    var lat = 0
    var lon = 0
    for (var i = 0; i < coords.length; i++) {
        lat += coords[i].latitude
        lon += coords[i].longitude
    }
    return QtPositioning.coordinate(lat / coords.length, lon / coords.length)
}

function coord_inside_polygon(coord, coords) {
    var sum = 0
    for (var i = 0; i < coords.length - 1; i++) {
        var a2 = geo_bearing(coord, coords[i + 1])
        var a1 = geo_bearing(coord, coords[i])
        var d = a2 - a1
        if (d < -180)
            d += 360
        else if (d > 180)
            d -= 360
        sum -= d
    }
    return Math.abs(sum) > 180
}

function point_map2canvas(coord, map, canvas) {
    return point_screen2canvas(map.fromCoordinate(coord, false), map, canvas)
}

function point_canvas2map(point, map, canvas) {
    return map.toCoordinate(point_canvas2screen(point, map, canvas))
}

function point_screen2canvas(point, map, canvas) {
    var o = map.fromCoordinate(canvas.coordinate, false)
    return rotate(Qt.point((point.x - o.x) * canvas.multiplier,
                           (point.y - o.y) * canvas.multiplier),
                  deg_to_rad(map.bearing))
}

function point_canvas2screen(point, map, canvas) {
    var o = map.fromCoordinate(canvas.coordinate)
    return Qt.point(point.x / canvas.multiplier + o.x,
                    point.y / canvas.multiplier + o.y)
}

function point_map2euclidean(latitude, longitude, centroid, lam, lom) {
    return Qt.point((centroid.longitude - longitude) * lom,
                    (centroid.latitude - latitude) * lam)
}

function point_euclidean2map(x, y, centroid, lam, lom) {
    return QtPositioning.coordinate(centroid.latitude - y / lam,
                                    centroid.longitude - x / lom)
}

function points_map2euclidean(coordinates, centroid, lam, lom) {
    var points = []
    for (var i = 0; i < coordinates.length; i++)
        points.push(point_map2euclidean(coordinates[i].latitude,
                                        coordinates[i].longitude, centroid,
                                        lam, lom))
    return points
}

function points_euclidean2map(points, centroid, lam, lom) {
    var coordinates = []
    for (var i = 0; i < points.length; i++)
        coordinates.push(point_euclidean2map(points[i].x, points[i].y,
                                             centroid, lam, lom))
    return coordinates
}

function segments_euclidean2map(ii, centroid, lam, lom) {
    var cc = []
    for (var i = 0; i < ii.length; i++)
        cc.push(points_euclidean2map(ii[i], centroid, lam, lom))
    return cc
}

function segments_map2canvas(cc, map, canvas) {
    var pp = []
    for (var i = 0; i < cc.length; i++) {
        var a = point_map2canvas(cc[i][0], map, canvas)
        var b = point_map2canvas(cc[i][1], map, canvas)
        pp.push([a, b])
    }
    return pp
}

function relative_zoom_pixel_ratio(map, zoom) {
    var t = map.tilt
    var z = map.zoomLevel
    map.tilt = 0
    map.zoomLevel = zoom
    var c0 = map.toCoordinate(Qt.point(1, 1), false)
    var c1 = map.toCoordinate(Qt.point(2, 1), false)
    map.zoomLevel = z
    var p0 = map.fromCoordinate(c0, false)
    var p1 = map.fromCoordinate(c1, false)
    map.tilt = t
    return 1.0 / (p1.x - p0.x)
}

function point_in_box(p, p1, p2) {
    var o = 0.01
    return ((p1.x - o <= p.x && p.x <= p2.x + o && p1.y - o <= p.y
             && p.y <= p2.y + o) || (p2.x - o <= p.x && p.x <= p1.x + o && p2.y - o <= p.y
                                     && p.y <= p1.y + o) || (p1.x - o <= p.x && p.x <= p2.x
                                                             + o && p2.y - o <= p.y
                                                             && p.y <= p1.y + o) || (p2.x - o <= p.x && p.x <= p1.x + o && p1.y - o <= p.y && p.y <= p2.y + o))
}

function polylines_disjoint(po1, po2) {
    for (var i = 0; i < po1.length - 1; i++)
        for (var j = 0; j < po2.length - 1; j++) {
            var inter = segments_interserction_1(po1[i], po1[i + 1], po2[j],
                                                 po2[j + 1])
            if (inter !== null && distance(inter, po1[i]) > 0.001 && distance(
                        inter, po1[i + 1]) > 0.001 && distance(inter,
                                                               po2[i]) > 0.001
                    && distance(inter, po2[i + 1]) > 0.001)
                return false
        }
    return true
}

function segments_interserction(s1, s2) {
    return segments_interserction_1(s1[0], s1[1], s2[0], s2[1])
}

function segments_interserction_1(s10, s11, s20, s21) {
    var angle_0 = Math.atan(slope(s10, s11))
    var l0 = to_homogeneous_line(s10, angle_0)
    var angle_i = Math.atan(slope(s20, s21))
    var li = to_homogeneous_line(s20, angle_i)
    var intersection = cross_product(l0, li)
    if (intersection[2] !== 0) {
        //not parallel
        var pi = from_homogeneous_point(intersection)
        if (point_in_box(pi, s10, s11) && point_in_box(pi, s20, s21))
            return pi
    }
    return null
}

function intersections_with_segments(point, angle, sides) {
    var l0 = to_homogeneous_line(point, deg_to_rad(angle))

    var ii = []
    for (var i = 0; i < sides.length; i++) {
        var s = sides[i]
        var li = to_homogeneous_line(s[0], Math.atan(slope(s[0], s[1])))

        var intersection = cross_product(l0, li)
        if (intersection[2] !== 0) {
            //not parallel
            var pi = from_homogeneous_point(intersection)
            if (point_in_box(pi, s[0], s[1]))
                ii.push(pi)
        }
    }
    return ii
}

function convex_polygon_parallel_slices_intersections(angle, offset, sides, ratio) {
    var a_perp = Math.atan(-Math.pow(ratio, 2) / Math.tan(deg_to_rad(angle)))
    var p_dir = Qt.point(Math.cos(a_perp) * offset, Math.sin(a_perp) * offset)

    var times = 0
    var ii1 = []
    while (true) {
        ++times
        var p = interpolate(Qt.point(0, 0), p_dir, -offset, times)
        var _i = intersections_with_segments(
                    p, angle,
                    sides) //NB two intersections always if a convex polygon
        if (_i.length < 2)
            break
        if (_i[0].x < _i[1].x || (_i[0].x === _i[1].x && _i[0].y < _i[1].y))
            ii1.push(_i.slice(0, 2))
        else
            ii1.push(_i.slice(0, 2).reverse())
    }

    times = -1
    var ii2 = []
    while (true) {
        ++times
        var p = interpolate(Qt.point(0, 0), p_dir, offset, times)
        var _i = intersections_with_segments(
                    p, angle,
                    sides) //NB two intersections always if a convex polygon
        if (_i.length < 2)
            break
        if (_i[0].x < _i[1].x || (_i[0].x === _i[1].x && _i[0].y < _i[1].y))
            ii2.push(_i.slice(0, 2))
        else
            ii2.push(_i.slice(0, 2).reverse())
    }

    return ii2.reverse().concat(ii1)
}

function rectify_parallelogram_side(p0, p1, p2, p3, ratio) {
    //p0-p1, p2-p3 must be parallel!
    //either p1 or p2 will be translated in order to create 90 degrees angles between p1 and p2, p2 and p3
    var _a = p1.x - p0.x
    var _b = p1.y - p0.y
    var _c = p1.x - p2.x
    var _d = p1.y - p2.y
    var a = angle_between(_a, _b, _c, _d)
    var m = slope(p0, p1)
    if (a > Math.PI / 2)
        p1 = project(p0, m, p2, ratio)
    else
        p2 = project(p3, m, p1, ratio)
    return [p0, p1, p2, p3]
}

function rectify_dense_winding(ii, ratio) {
    var a = 0, b = 1
    for (var i = 0; i < ii.length - 1; i++) {
        var rr = rectify_parallelogram_side(ii[i][a], ii[i][b], ii[i + 1][b],
                                            ii[i + 1][a], ratio)
        ii[i][b] = rr[1]
        ii[i + 1][b] = rr[2]
        a = flip(a)
        b = flip(b)
    }
    return ii
}

function set_points_clockwise(points) {
    if (three_point_direction(points[0], points[1], points[2]) === 1)
        points.reverse()
    return points
}

function make_sides(points) {
    var r = []
    for (var i = 0; i < points.length - 1; i++)
        r.push([points[i], points[i + 1]])
    r.push([points[points.length - 1], points[0]])
    return r
}

function intersect_two_lines(p0, m0, p1, m1) {
    var l0 = to_homogeneous_line(p0, Math.atan(m0))
    var l1 = to_homogeneous_line(p1, Math.atan(m1))

    var intersection = cross_product(l0, l1)
    return from_homogeneous_point(intersection)
}

function project(p0, m, p1, ratio) {
    return intersect_two_lines(p0, m, p1, -Math.pow(ratio, 2) / m)
}

function distance(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2))
}

function interpolate(from, to, offset, times) {
    var a = Math.atan2(to.y - from.y, to.x - from.x)
    var ox = offset * times * Math.cos(a)
    var oy = offset * times * Math.sin(a)
    return Qt.point(from.x + ox, from.y + oy)
}

function cross_product(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

function to_homogeneous_line(point, angle) {
    var m = Math.tan(angle)
    if ((Math.abs(m) === 16331239353195370) || (Math.abs(m) === Infinity))
        return [1, 0, -point.x]
    if (m === 0)
        return [0, 1, -point.y]
    var q = point.y - m * point.x
    return [m, -1, q]
}

function from_homogeneous_point(p) {
    return Qt.point(p[0] / p[2], p[1] / p[2])
}

function slope(p0, p1) {
    return (p0.y - p1.y) / (p0.x - p1.x)
}

function angle_between(/*vector 1*/ _a, _b, /*vector 2*/ _c, _d) {
    return Math.acos(
                (_a * _c + _b * _d) / (Math.sqrt(_a * _a + _b * _b) * Math.sqrt(
                                           _c * _c + _d * _d)))
}

function flip(x) {
    return (x === 0) ? 1 : 0
}

function add_and_wrap(x, mod, quota) {
    if (quota == null)
        quota = 1
    return (x + quota < mod) ? x + quota : x + quota - mod
}

function sub_and_wrap(x, mod, quota) {
    if (quota == null)
        quota = 1
    return (x - quota >= 0) ? x - quota : x + mod - quota
}

function rotate(p, a) {
    var x = p.x * Math.cos(a) - p.y * Math.sin(a)
    var y = p.x * Math.sin(a) + p.y * Math.cos(a)
    return Qt.point(x, y)
}

function deg_to_rad(deg) {
    return (Math.PI / 180.0) * deg
}

function rad_to_deg(rad) {
    return (180.0 / Math.PI) * rad
}

function three_point_direction(p1, p2, p3) {
    var a1 = rad_to_deg(Math.atan2(p1.y - p2.y, p1.x - p2.x))
    var a2 = rad_to_deg(Math.atan2(p2.y - p3.y, p2.x - p3.x))
    a1 = a1 - 360 * Math.floor(a1 / 360)
    a2 = a2 - 360 * Math.floor(a2 / 360)
    return ((a1 < a2 && (a2 - a1) < 180)
            || (a1 > a2 && (a1 - a2) > 180)) ? 1 /*cc*/ : 2 /*c*/
}

function coherent_points_direction(pts, dir){
    if (dir !== null && dir !== undefined){
        for (var i = 0; i+2<=pts.length-1; i++)
            if ( dir !== three_point_direction(pts[i], pts[i+1], pts[i+2]))
                return false
    } else {
        dir = three_point_direction(pts[0], pts[1], pts[2])
        for (var j = 1; j+2<=pts.length-1; j++)
            if ( dir !== three_point_direction(pts[j], pts[j+1], pts[j+2]))
                return false
    }
    return true
}

function lat_to_m_coeff(lat) {
    lat = deg_to_rad(lat)
    return 111132.92 - 559.82 * Math.cos(2 * lat) + 1.175 * Math.cos(
                4 * lat) - 0.0023 * Math.cos(6 * lat)
}

function lon_to_m_coeff(lon) {
    lon = deg_to_rad(lon)
    return 111412.84 * Math.cos(lon) - 93.5 * Math.cos(
                3 * lon) + 0.118 * Math.cos(5 * lon)
}

function generate_nurb_circle(p0, p3, dir) {
    var dist = distance(p0, p3)
    var angle = Math.tan(1 / -slope(p0, p3))
    if (dir === 1) {
        var p1 = Qt.point(p0.x + Math.cos(angle) * dist,
                          p0.y + Math.sin(angle) * dist)
        var p2 = Qt.point(p3.x + Math.cos(angle) * dist,
                          p3.y + Math.sin(angle) * dist)
    } else {
        var p1 = Qt.point(p0.x - Math.cos(angle) * dist,
                          p0.y - Math.sin(angle) * dist)
        var p2 = Qt.point(p3.x - Math.cos(angle) * dist,
                          p3.y - Math.sin(angle) * dist)
    }
    return {
        degree: 3,
        points: [[p0.x, p0.y], [p1.x, p1.y], [p2.x, p2.y], [p3.x, p3.y]],
        weigths: [1, 1 / 3.0, 1 / 3.0, 1],
        knots: [0, 0, 0, 0, 1, 1, 1, 1]
    }
}

function generate_nurb_line(p0, p1) {
    return {
        degree: 1,
        points: [[p0.latitude, p0.longitude], [p1.latitude, p1.longitude]],
        weigths: [1, 1],
        knots: [0, 0, 1, 1]
    }
}

function generate_nurb_broken_line(pp) {
    var points = []
    var weights = []
    var knots = [0]
    for (var i = 0; i < pp.length; i++) {
        points.push([pp[i].x, pp[i].y])
        knots.push((i + 1) / pp.length)
        weights.push(1)
    }
    return {
        degree: 1,
        points: points,
        weigths: weights,
        knots: knots
    }
}

function shape_geo_limits(coords) {
    var min_x = coords[0].longitude
    var max_x = coords[0].longitude
    var min_y = coords[0].latitude
    var max_y = coords[0].latitude
    for (var i = 1; i < coords.length; i++) {
        var p = coords[i]
        if (p.longitude < min_x)
            min_x = p.longitude
        else if (p.longitude > max_x)
            max_x = p.longitude
        if (p.latitude < min_y)
            min_y = p.latitude
        else if (p.latitude > max_y)
            max_y = p.latitude
    }
    return {
        min_lon: min_x,
        max_lon: max_x,
        min_lat: min_y,
        max_lat: max_y
    }
}

function shape_px_dimensions(limits, map) {
    var p0 = map.fromCoordinate(QtPositioning.coordinate(limits.min_lat,
                                                         limits.min_lon), false)
    var p1 = map.fromCoordinate(QtPositioning.coordinate(limits.min_lat,
                                                         limits.max_lon), false)
    var p2 = map.fromCoordinate(QtPositioning.coordinate(limits.max_lat,
                                                         limits.min_lon), false)
    var w = distance(p0, p1)
    var h = distance(p0, p2)
    return [w, h]
}

function init_canvas(canvas, map, width, height, lat, lon, px_multiplier) {
    canvas.canvasWidth = width * px_multiplier
    canvas.canvasHeight = height * px_multiplier
    canvas.zoomLevel = map.maximumZoomLevel
    canvas.coordinate = QtPositioning.coordinate(lat, lon)
    canvas.canvasAngle = 0
    canvas.multiplier = px_multiplier
    canvas.canvasCtx.lineWidth = 6
    canvas.canvasCtx.lineJoin = "bevel"
}

function draw_line(ctx, x1, y1, x2, y2) {
    ctx.strokeStyle = "#ff0000"
    ctx.beginPath()
    ctx.moveTo(x1, y1)
    ctx.lineTo(x2, y2)
    ctx.stroke()
}

function draw_bounding_rect(map, canvas, w, h) {
    var ctx = canvas.canvasCtx
    ctx.fillStyle = Qt.rgba(0, 0.4, 0, 0.2)
    ctx.fillRect(0, 0, w, h)
    canvas.requestPaint()
}

function ctx_draw_path_lines(ctx, pp) {
    for (var i = 0; i < pp.length; i++) {
        var p0 = pp[i][0]
        var p1 = pp[i][1]
        draw_line(ctx, p0.x, p0.y, p1.x, p1.y)
    }
}

function draw_path_lines(canvas, pp) {
    var ctx = canvas.canvasCtx
    ctx_draw_path_lines(ctx, pp)
    canvas.requestPaint()
}

function ctx_draw_manouvre_simple(ctx, pp) {
    for (var i = 0; i < pp.length - 1; i++) {
        var dir = (i + 1) % 2
        var p0 = pp[i][dir]
        var p1 = pp[i + 1][dir]
        draw_line(ctx, p0.x, p0.y, p1.x, p1.y)
    }
}

function draw_manouvre_simple(canvas, pp) {
    var ctx = canvas.canvasCtx
    ctx_draw_manouvre_simple(ctx, pp)
    canvas.requestPaint()
}

function draw_manouvre_single_winding(canvas, pp) {
    for (var i = 0, a = 0, b = 1; i < pp.length - 1; i++) {
        a = flip(a)
        b = flip(b)
        var p0 = pp[i][b]
        var p1 = pp[i][a]
        var p2 = pp[i + 1][a]
        var p3 = pp[i + 1][b]
        draw_semicircle(canvas.canvasCtx, p0, p1, p2, p3)
    }
}

function draw_semicircle(ctx, p0, p1, p2, p3) {
    var l1 = distance(p0, p1)
    var l2 = distance(p3, p2)
    var d = distance(p1, p2)
    var c = 0.55191502449 * d / 2
    var pc1a = interpolate(p0, p1, l1 + c, 1)
    var pc1b = interpolate(p3, p2, l2 + c, 1)
    var _a = interpolate(p0, p1, l1 + d / 2, 1)
    var _b = interpolate(p3, p2, l2 + d / 2, 1)
    var pc3 = interpolate(_a, _b, d / 2, 1)
    var pc2a = interpolate(pc3, _a, c, 1)
    var pc2b = interpolate(pc3, _b, c, 1)
    draw_cubic_bezier(ctx, p1, pc1a, pc2a, pc3)
    draw_cubic_bezier(ctx, pc3, pc2b, pc1b, p2)
}

function draw_cubic_bezier(ctx, c0, c1, c2, c3) {
    ctx.strokeStyle = "#0000ff"
    ctx.beginPath()
    ctx.moveTo(c0.x, c0.y)
    ctx.bezierCurveTo(c1.x, c1.y, c2.x, c2.y, c3.x, c3.y)
    ctx.stroke()
}

function draw_circle(ctx, p0, p1, p2, p3, color) {
    ctx.strokeStyle = color
    ctx.beginPath()
    ctx.moveTo(p0.x, p0.y)
    ctx.arc(p0.x, p0.y, r, 0, 2 * Math.PI)
    ctx.moveTo(p0.x, p0.y)
    ctx.stroke()
}

function roundNumber(number, digits) {
    var multiple = Math.pow(10, digits)
    return Math.round(number * multiple) / multiple
}

function formatTime(sec) {
    var value = sec
    var seconds = value % 60
    value /= 60
    value = (value > 1) ? Math.round(value) : 0
    var minutes = value % 60
    value /= 60
    value = (value > 1) ? Math.round(value) : 0
    var hours = value
    if (hours > 0)
        value = hours + "h:" + minutes + "m"
    else
        value = minutes + "min"
    return value
}

function formatDistance(meters) {
    var dist = Math.round(meters)
    if (dist > 1000) {
        if (dist > 100000) {
            dist = Math.round(dist / 1000)
        } else {
            dist = Math.round(dist / 100)
            dist = dist / 10
        }
        dist = dist + " km"
    } else {
        dist = dist + " m"
    }
    return dist
}

//Taken from https://www.movable-type.co.uk/scripts/latlong.html
function geo_midpoint(c1, c2) {
    var φ1 = deg_to_rad(c1.latitude)
    var λ1 = deg_to_rad(c1.longitude)
    var φ2 = deg_to_rad(c2.latitude)
    var λ2 = deg_to_rad(c2.longitude)

    var Bx = Math.cos(φ2) * Math.cos(λ2 - λ1)
    var By = Math.cos(φ2) * Math.sin(λ2 - λ1)
    var φ3 = Math.atan2(
                Math.sin(φ1) + Math.sin(φ2),
                Math.sqrt((Math.cos(φ1) + Bx) * (Math.cos(φ1) + Bx) + By * By))
    var λ3 = λ1 + Math.atan2(By, Math.cos(φ1) + Bx)
    return QtPositioning.coordinate(rad_to_deg(φ3),
                                    rad_to_deg((λ3 + 540) % 360 - 180))
}

function geo_bearing(c1, c2) {
    var φ1 = deg_to_rad(c1.latitude)
    var λ1 = deg_to_rad(c1.longitude)
    var φ2 = deg_to_rad(c2.latitude)
    var λ2 = deg_to_rad(c2.longitude)
    var y = Math.sin(λ2 - λ1) * Math.cos(φ2)
    var x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(
                φ2) * Math.cos(λ2 - λ1)
    var r = rad_to_deg(Math.atan2(y, x))
    return (r > 0) ? r : r + 360
}

function geo_intermediate(c1, c2, fraction) {
    var φ1 = deg_to_rad(c1.latitude)
    var λ1 = deg_to_rad(c1.longitude)
    var φ2 = deg_to_rad(c2.latitude)
    var λ2 = deg_to_rad(c2.longitude)

    var Δφ = φ2 - φ1
    var Δλ = λ2 - λ1
    var a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) + Math.cos(φ1) * Math.cos(
                φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2)
    var δ = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))

    var A = Math.sin((1 - fraction) * δ) / Math.sin(δ)
    var B = Math.sin(fraction * δ) / Math.sin(δ)

    var x = A * Math.cos(φ1) * Math.cos(λ1) + B * Math.cos(φ2) * Math.cos(λ2)
    var y = A * Math.cos(φ1) * Math.sin(λ1) + B * Math.cos(φ2) * Math.sin(λ2)
    var z = A * Math.sin(φ1) + B * Math.sin(φ2)

    var φ3 = Math.atan2(z, Math.sqrt(x * x + y * y))
    var λ3 = Math.atan2(y, x)

    var lat = rad_to_deg(φ3)
    var lon = rad_to_deg(λ3)

    return QtPositioning.coordinate(lat, lon)
}
