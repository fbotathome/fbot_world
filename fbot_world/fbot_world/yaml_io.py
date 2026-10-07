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
    @return The parsed mapping, or an empty OrderedDict if the file is missing,
    empty or unparseable.
    '''
    if not os.path.exists(path):
        return OrderedDict()
    with open(path, 'r') as f:
        try:
            return yaml.load(f, Loader=OrderedLoader) or OrderedDict()
        except yaml.YAMLError:
            return OrderedDict()


def dump_ordered(data, path: str) -> None:
    '''@brief Write a mapping to a YAML file in block style, preserving order.'''
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, Dumper=OrderedDumper)
