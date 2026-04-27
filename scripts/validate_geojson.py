#!/usr/bin/env python3
"""
GeoJSON Graph Validator
Checks if your graph file is valid GeoJSON that route_server can parse
"""

import json
import sys
import os

def validate_geojson(filepath):
    """Validate GeoJSON graph file"""
    
    print("="*60)
    print("GeoJSON Graph Validator")
    print("="*60)
    
    # Check file exists
    if not os.path.exists(filepath):
        print(f"❌ ERROR: File not found: {filepath}")
        return False
    
    print(f"✅ File exists: {filepath}")
    
    # Try to parse as JSON
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        print("✅ Valid JSON format")
    except json.JSONDecodeError as e:
        print(f"❌ ERROR: Invalid JSON format")
        print(f"   Error: {e}")
        return False
    
    # Check if it's a FeatureCollection
    if data.get('type') != 'FeatureCollection':
        print(f"❌ ERROR: Root type must be 'FeatureCollection', got '{data.get('type')}'")
        return False
    print("✅ Valid FeatureCollection")
    
    # Check features exist
    features = data.get('features', [])
    if not features:
        print("❌ ERROR: No features found in FeatureCollection")
        return False
    print(f"✅ Found {len(features)} features")
    
    # Analyze features
    points = []
    lines = []
    
    for i, feature in enumerate(features):
        ftype = feature.get('type')
        if ftype != 'Feature':
            print(f"⚠️  WARNING: Feature {i} has type '{ftype}', expected 'Feature'")
            continue
        
        geom = feature.get('geometry', {})
        geom_type = geom.get('type')
        feature_id = feature.get('id', f'feature_{i}')
        
        if geom_type == 'Point':
            coords = geom.get('coordinates', [])
            points.append({
                'id': feature_id,
                'coords': coords,
                'properties': feature.get('properties', {})
            })
        elif geom_type == 'LineString':
            coords = geom.get('coordinates', [])
            lines.append({
                'id': feature_id,
                'coords': coords,
                'properties': feature.get('properties', {})
            })
        else:
            print(f"⚠️  WARNING: Unknown geometry type '{geom_type}' in feature {feature_id}")
    
    print(f"\n📍 Points (Nodes): {len(points)}")
    for p in points:
        coords = p['coords']
        print(f"   - {p['id']}: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f})")
    
    print(f"\n📏 LineStrings (Edges): {len(lines)}")
    for line in lines:
        props = line['properties']
        from_node = props.get('from', 'unknown')
        to_node = props.get('to', 'unknown')
        bidir = props.get('bidirectional', False)
        arrow = '<->' if bidir else '->'
        print(f"   - {line['id']}: {from_node} {arrow} {to_node}")
    
    # Validation checks
    print("\n🔍 Validation Checks:")
    
    if len(points) < 2:
        print(f"❌ ERROR: Need at least 2 points (nodes), found {len(points)}")
        return False
    print(f"✅ Sufficient points: {len(points)}")
    
    if len(lines) == 0:
        print(f"⚠️  WARNING: No edges defined - graph is disconnected")
    else:
        print(f"✅ Edges defined: {len(lines)}")
    
    # Check edge references
    point_ids = {p['id'] for p in points}
    invalid_edges = []
    
    for line in lines:
        props = line['properties']
        from_node = props.get('from')
        to_node = props.get('to')
        
        if from_node not in point_ids:
            invalid_edges.append(f"{line['id']}: 'from' node '{from_node}' not found")
        if to_node not in point_ids:
            invalid_edges.append(f"{line['id']}: 'to' node '{to_node}' not found")
    
    if invalid_edges:
        print(f"❌ ERROR: Invalid edge references:")
        for err in invalid_edges:
            print(f"   - {err}")
        return False
    else:
        print(f"✅ All edge references are valid")
    
    print("\n" + "="*60)
    print("✅ VALIDATION PASSED - Graph file is valid!")
    print("="*60)
    return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 validate_geojson.py <path_to_graph.geojson>")
        sys.exit(1)
    
    filepath = sys.argv[1]
    success = validate_geojson(filepath)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
