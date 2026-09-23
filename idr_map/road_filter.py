"""
idr_map.road_filter
===================
Deterministic road classification, access filtering, and tagging rules
for OpenStreetMap offline road extraction in ISRO IDR.
"""

from typing import Dict, Any, Optional, Tuple

# Supported drivable road classes and numeric enum identifiers
ROAD_CLASS_TO_ID = {
    "motorway": 1,
    "trunk": 2,
    "primary": 3,
    "secondary": 4,
    "tertiary": 5,
    "residential": 6,
    "service": 7,
    "unclassified": 8,
    "motorway_link": 9,
    "trunk_link": 9,
    "primary_link": 9,
    "secondary_link": 9,
    "tertiary_link": 9,
    "living_street": 10,
}

ID_TO_ROAD_CLASS = {v: k for k, v in ROAD_CLASS_TO_ID.items()}
ID_TO_ROAD_CLASS[9] = "link"

# Non-drivable highway types to explicitly reject
NON_DRIVABLE_HIGHWAYS = {
    "footway", "cycleway", "pedestrian", "steps", "path",
    "track", "bridleway", "corridor", "elevator", "proposed",
    "construction", "raceway", "abandoned", "escape", "bus_guideway"
}

# Access tags indicating closed / private access to motor vehicles
PROHIBITED_ACCESS_VALUES = {"no", "private", "restricted", "agricultural", "forestry", "delivery", "emergency"}
ALLOWED_ACCESS_VALUES = {"yes", "permissive", "designated", "destination", "customers"}


def filter_osm_way(tags: Dict[str, str]) -> Tuple[bool, Optional[int], int, int, int, str]:
    """
    Evaluates OSM way tags for drivable road inclusion.
    
    Returns:
        (is_drivable, road_class_id, oneway_direction, layer, flags, name)
        where:
            oneway_direction: 0 = bidirectional, 1 = forward only, -1 = reverse only
            layer: integer between -5 and +5 (default 0)
            flags: bitmask (bit 0: bridge, bit 1: tunnel, bit 2: roundabout, bit 3: paved)
    """
    highway = tags.get("highway", "").strip().lower()
    if not highway or highway in NON_DRIVABLE_HIGHWAYS:
        return False, None, 0, 0, 0, ""

    if highway not in ROAD_CLASS_TO_ID:
        return False, None, 0, 0, 0, ""

    road_class_id = ROAD_CLASS_TO_ID[highway]

    # Access filtering (hierarchical: motorcar -> motor_vehicle -> vehicle -> access)
    access = tags.get("access", "").strip().lower()
    vehicle = tags.get("vehicle", "").strip().lower()
    motor_vehicle = tags.get("motor_vehicle", "").strip().lower()
    motorcar = tags.get("motorcar", "").strip().lower()

    if motorcar in PROHIBITED_ACCESS_VALUES:
        return False, None, 0, 0, 0, ""
    elif not motorcar and motor_vehicle in PROHIBITED_ACCESS_VALUES:
        return False, None, 0, 0, 0, ""
    elif not motorcar and not motor_vehicle and vehicle in PROHIBITED_ACCESS_VALUES:
        return False, None, 0, 0, 0, ""
    elif not motorcar and not motor_vehicle and not vehicle and access in PROHIBITED_ACCESS_VALUES:
        return False, None, 0, 0, 0, ""

    # Directionality
    oneway_tag = tags.get("oneway", "").strip().lower()
    junction_tag = tags.get("junction", "").strip().lower()

    oneway_dir = 0
    if oneway_tag in ("yes", "1", "true"):
        oneway_dir = 1
    elif oneway_tag in ("-1", "reverse"):
        oneway_dir = -1
    elif oneway_tag in ("no", "0", "false"):
        oneway_dir = 0
    elif junction_tag in ("roundabout", "circular"):
        oneway_dir = 1
    elif highway in ("motorway", "motorway_link"):
        oneway_dir = 1  # Motorways are oneway by default unless specified

    # Vertical Layer & 3D Info
    layer_str = tags.get("layer", "0").strip()
    try:
        layer = int(layer_str)
        layer = max(-5, min(5, layer))
    except ValueError:
        layer = 0

    # Flags bitmask
    flags = 0
    bridge_tag = tags.get("bridge", "").strip().lower()
    if bridge_tag in ("yes", "viaduct", "aqueduct", "cantilever", "suspension"):
        flags |= 0x01
        if layer == 0:
            layer = 1  # Default bridge layer to +1 if unspecified

    tunnel_tag = tags.get("tunnel", "").strip().lower()
    if tunnel_tag in ("yes", "building_passage", "culvert"):
        flags |= 0x02
        if layer == 0:
            layer = -1  # Default tunnel layer to -1 if unspecified

    if junction_tag in ("roundabout", "circular"):
        flags |= 0x04

    surface_tag = tags.get("surface", "").strip().lower()
    if surface_tag in ("paved", "asphalt", "concrete", "paving_stones", "sett"):
        flags |= 0x08

    name = tags.get("name", "").strip()

    return True, road_class_id, oneway_dir, layer, flags, name
