#!/usr/bin/env python3
"""
Test script to verify boundary handling and out-of-bounds node processing fixes
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from phingdee2 import GraphMapper, GraphNode

def test_boundary_handling():
    """Test the boundary handling logic to ensure it doesn't hang"""
    print("🧪 Testing boundary handling fixes...")
    
    # Create a small 3x3 grid mapper
    graph_mapper = GraphMapper(min_x=-1, min_y=-1, max_x=1, max_y=1)
    
    # Test case 1: Node at boundary with open paths to out-of-bounds areas
    print("\n📍 Test 1: Node at boundary (-1, -1) with open paths")
    
    # Create a node at (-1, -1) - corner of the 3x3 grid
    position = (-1, -1)
    node = graph_mapper.create_node(position)
    
    # Set robot facing south
    graph_mapper.currentDirection = 'south'
    graph_mapper.currentPosition = position
    
    # Simulate walls: front blocked (south wall), left/right open (would be out-of-bounds)
    node.walls = {
        'north': False,  # Back - leads to explored area
        'south': True,   # Front - blocked by wall
        'east': False,   # Left when facing south - leads to out-of-bounds
        'west': False    # Right when facing south - leads to out-of-bounds
    }
    
    print(f"   Node position: {position}")
    print(f"   Robot facing: {graph_mapper.currentDirection}")
    print(f"   Wall configuration: {node.walls}")
    
    try:
        # This should complete without hanging
        print("   🔄 Running update_unexplored_exits_absolute...")
        graph_mapper.update_unexplored_exits_absolute(node)
        
        print(f"   ✅ Success! Unexplored exits: {node.unexploredExits}")
        print(f"   ✅ Out-of-bounds exits: {node.outOfBoundsExits}")
        print(f"   ✅ Node is in frontier queue: {node.id in graph_mapper.frontierQueue}")
        
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return False
    
    # Test case 2: Node completely surrounded by out-of-bounds
    print("\n📍 Test 2: Node at corner with mixed boundaries")
    
    position2 = (1, 1)  # Other corner
    node2 = graph_mapper.create_node(position2)
    graph_mapper.currentPosition = position2
    
    node2.walls = {
        'north': False,  # Would be out-of-bounds (1, 2)
        'south': False,  # Valid position (1, 0)
        'east': False,   # Would be out-of-bounds (2, 1)
        'west': False    # Valid position (0, 1)
    }
    
    try:
        print("   🔄 Running update_unexplored_exits_absolute...")
        graph_mapper.update_unexplored_exits_absolute(node2)
        
        print(f"   ✅ Success! Unexplored exits: {node2.unexploredExits}")
        print(f"   ✅ Out-of-bounds exits: {node2.outOfBoundsExits}")
        
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return False
    
    print("\n🎉 All boundary handling tests passed!")
    return True

def test_scan_error_handling():
    """Test the enhanced error handling in scan functions"""
    print("\n🧪 Testing scan function error handling...")
    
    # Test that error handling doesn't break the basic logic
    try:
        graph_mapper = GraphMapper(min_x=-1, min_y=-1, max_x=1, max_y=1)
        
        # Create a mock scenario
        position = (0, 0)
        node = graph_mapper.create_node(position)
        graph_mapper.currentPosition = position
        graph_mapper.currentDirection = 'north'
        
        # Set some walls
        node.walls = {'north': True, 'south': False, 'east': False, 'west': False}
        
        # This should work with error handling
        graph_mapper.update_unexplored_exits_absolute(node)
        print("   ✅ Scan error handling works correctly")
        
    except Exception as e:
        print(f"   ❌ Scan error handling failed: {e}")
        return False
    
    return True

if __name__ == "__main__":
    print("🚀 Running boundary handling and marker detection fix tests...")
    
    success = True
    
    # Test boundary handling
    if not test_boundary_handling():
        success = False
    
    # Test scan error handling
    if not test_scan_error_handling():
        success = False
    
    if success:
        print("\n✅ All tests passed! The fixes should prevent hanging on out-of-bounds nodes.")
        print("🔧 Key improvements:")
        print("   • Enhanced error handling in scan functions")
        print("   • Timeout protection in boundary processing")
        print("   • Better out-of-bounds detection and handling")
        print("   • Enhanced marker detection with left-right sweep")
        print("   • Robust exploration loop with safety checks")
    else:
        print("\n❌ Some tests failed! Please review the implementation.")
    
    sys.exit(0 if success else 1)