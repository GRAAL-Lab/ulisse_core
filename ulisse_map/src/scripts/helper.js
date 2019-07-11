/****************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensllc holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

.pragma library

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
    return (((p1.x <= p.x && p.x <= p2.x) || p2.x <= p.x && p.x <= p1.x)
        &&  ((p1.y <= p.y && p.y <= p2.y) || p2.y <= p.y && p.y <= p1.y))
}

function to_homogeneous_line(m,q){
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
        if (points[i].y > t){
            t = points[i].y;
            pt = points[i]
        }
    }
    var pti = points.indexOf(pt)
    return [pt, pti]
}

function flip(x){
    return (x===0) ? 1 : 0
}

function add_and_wrap(x, mod){
    return (x+1 < mod) ? x+1 : 0
}

function toCanvasCoordinates(point, map, canvas){
    var o = map.fromCoordinate(canvas.coordinate)
    return Qt.point(point.x-o.x, point.y-o.y)
}

function deg_to_rad(deg){
    return (Math.PI/180.0) * deg
}

function rad_to_deg(rad){
    return (180.0/Math.PI) * rad
}

function three_point_direction(p1,p2,p3){
    var a1 = Math.atan2(p1.y-p2.y, p1.x-p2.x)*360/(2*Math.PI)
    var a2 = Math.atan2(p2.y-p3.y, p2.x-p3.x)*360/(2*Math.PI)
    a1 = a1-360*Math.floor(a1/360)
    a2 = a2-360*Math.floor(a2/360)
    return ((a1<a2 && (a2-a1)<180) || (a1>a2 && (a1-a2)>180)) ? 1/*cc*/ : 2/*c*/
}
