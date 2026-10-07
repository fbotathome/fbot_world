#!/usr/bin/env python3
"""
Shared RViz marker builders for fbot_world.

Used by both the runtime pose plugin (pose.py) and the interactive room
annotator (room_writer.py), so the live annotation preview is drawn identically
to the markers published from the saved YAML.
"""

import numpy as np

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def make_color(rgba: tuple) -> ColorRGBA:
    '''@brief Build a ColorRGBA from an (r, g, b, a) tuple.'''
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = (float(v) for v in rgba)
    return c


def lifetime_forever() -> Duration:
    '''@brief Duration(sec=0, nanosec=0) means the marker lives forever.'''
    return Duration(sec=0, nanosec=0)


def centroid(polygon: list) -> tuple:
    '''@brief Return the (x, y) centroid of a polygon given as [[x, y], ...].'''
    arr = np.array(polygon, dtype=np.float32)
    return float(arr[:, 0].mean()), float(arr[:, 1].mean())


def polygon_line_strip(
    polygon: list,
    marker_id: int,
    ns: str,
    color: tuple,
    z: float = 0.0,
    line_width: float = 0.05,
    frame: str = 'map',
    closed: bool = True,
) -> Marker:
    '''
    @brief Creates a LINE_STRIP marker that traces a polygon.
    @param closed: if True, repeat the first vertex to close the loop.
    '''
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = line_width
    m.color = make_color(color)
    m.lifetime = lifetime_forever()
    m.pose.orientation.w = 1.0

    for v in polygon:
        p = Point()
        p.x = float(v[0])
        p.y = float(v[1])
        p.z = z
        m.points.append(p)

    if closed and polygon:
        p = Point()
        p.x = float(polygon[0][0])
        p.y = float(polygon[0][1])
        p.z = z
        m.points.append(p)

    return m


def polygon_fill(
    polygon: list,
    marker_id: int,
    ns: str,
    color: tuple,
    z: float = 0.0,
    frame: str = 'map',
) -> Marker:
    '''
    @brief Creates a TRIANGLE_LIST marker that fills a polygon via a centroid fan.
    Works well for convex/mildly concave shapes.
    '''
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns + '_fill'
    m.id = marker_id
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    # Make fill slightly more transparent than the outline.
    fill_color = (color[0], color[1], color[2], color[3] * 0.6)
    m.color = make_color(fill_color)
    m.lifetime = lifetime_forever()
    m.pose.orientation.w = 1.0

    arr = np.array(polygon, dtype=np.float32)
    cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
    n = len(arr)
    for i in range(n):
        j = (i + 1) % n
        for vx, vy in [(cx, cy), (arr[i][0], arr[i][1]), (arr[j][0], arr[j][1])]:
            p = Point()
            p.x = float(vx)
            p.y = float(vy)
            p.z = z
            m.points.append(p)

    return m


def text_marker(
    text: str,
    position: tuple,
    marker_id: int,
    ns: str,
    z: float = 0.3,
    text_size: float = 0.25,
    frame: str = 'map',
) -> Marker:
    '''
    @brief Creates a TEXT_VIEW_FACING marker at a given position. Always white for
    readability.
    '''
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns + '_text'
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = z
    m.pose.orientation.w = 1.0
    m.scale.z = text_size
    m.color = make_color((1.0, 1.0, 1.0, 1.0))
    m.lifetime = lifetime_forever()
    m.text = text
    return m


def point_spheres(
    points: list,
    marker_id: int,
    ns: str,
    color: tuple,
    scale: float = 0.12,
    z: float = 0.05,
    frame: str = 'map',
) -> Marker:
    '''
    @brief Creates a SPHERE_LIST marker, one sphere per point. Used to show the
    individual vertices clicked while a polygon is being drawn.
    '''
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns + '_points'
    m.id = marker_id
    m.type = Marker.SPHERE_LIST
    m.action = Marker.ADD
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    m.color = make_color(color)
    m.lifetime = lifetime_forever()
    m.pose.orientation.w = 1.0
    for v in points:
        p = Point()
        p.x = float(v[0])
        p.y = float(v[1])
        p.z = z
        m.points.append(p)
    return m


def delete_all_marker(frame: str = 'map') -> Marker:
    '''@brief A DELETEALL marker; prepend to a MarkerArray to clear stale markers.'''
    m = Marker()
    m.header.frame_id = frame
    m.action = Marker.DELETEALL
    return m
