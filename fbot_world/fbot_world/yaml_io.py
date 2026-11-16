#!/usr/bin/env python3
"""
Shared ordered-YAML helpers for fbot_world.

The pose writers and the room annotator all need to round-trip YAML while
preserving key order, so the saved config files stay diff-friendly and match
the hand-authored layout. This centralises the OrderedDict dumper/loader that
was previously duplicated across the writer nodes.
"""

import os

from collections import OrderedDict

import yaml


class OrderedDumper(yaml.SafeDumper):
    '''@brief YAML dumper that serialises OrderedDict as a plain mapping.'''


def _represent_ordereddict(dumper, data):
    return dumper.represent_dict(data.items())


OrderedDumper.add_representer(OrderedDict, _represent_ordereddict)


class OrderedLoader(yaml.SafeLoader):
    '''@brief YAML loader that builds OrderedDict to preserve key order.'''


def _construct_ordered_dict(loader, node):
    return OrderedDict(loader.construct_pairs(node))


OrderedLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_ordered_dict,
)


def load_ordered(path: str) -> OrderedDict:
    '''
    @brief Load a YAML file as an OrderedDict.
    @return The parsed mapping, or an empty OrderedDict if the file is missing
    or empty.
    @raise yaml.YAMLError if the file cannot be parsed. Writers must not
    overwrite such a file: it would erase its poses/rooms.
    '''
    if not os.path.exists(path):
        return OrderedDict()
    with open(path, 'r') as f:
        return yaml.load(f, Loader=OrderedLoader) or OrderedDict()


def merge_poses(data: OrderedDict, group: str, new_poses: dict) -> OrderedDict:
    '''
    @brief Merge poses into a config mapping using the layout read by pose.py:
        poses: {<group>: {<name>: {px, py, pz, ox, oy, oz, ow}}}
        rooms: {<room>: {...}}
    'rooms' and any other top-level key are kept. Older layouts are moved under
    'poses': groups at the top level (file without 'poses') and groups under
    'places' (written by previous versions of the pose writers).
    @param data: The mapping loaded from the YAML file (modified in place).
    @param group: The pose group to write, e.g. 'targets'.
    @param new_poses: {<name>: {px, ...}} to add or replace in that group.
    @return The same mapping.
    '''
    if not isinstance(data.get('poses'), dict):
        legacy_groups = [k for k, v in data.items()
                         if k not in ('poses', 'rooms', 'places') and isinstance(v, dict)]
        data['poses'] = OrderedDict((k, data.pop(k)) for k in legacy_groups)
    poses = data['poses']

    places = data.pop('places', None)
    if isinstance(places, dict):
        for legacy_group, legacy_poses in places.items():
            if not isinstance(legacy_poses, dict):
                continue
            if not isinstance(poses.get(legacy_group), dict):
                poses[legacy_group] = OrderedDict()
            poses[legacy_group].update(legacy_poses)

    if not isinstance(poses.get(group), dict):
        poses[group] = OrderedDict()
    poses[group].update(new_poses)

    if not isinstance(data.get('rooms'), dict):
        data['rooms'] = OrderedDict()
    return data


def dump_ordered(data, path: str) -> None:
    '''@brief Write a mapping to a YAML file in block style, preserving order.'''
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, Dumper=OrderedDumper)
