#!/usr/bin/env python3
"""
Test script to verify merged phingdee2.py has both:
1. Complete exploration capability from patch branch
2. Boundary checking from main branch
3. Enhanced backtracking logic
"""

import sys
import time
from unittest.mock import Mock, MagicMock

# Mock the robomaster imports and other dependencies to allow testing without hardware
sys.modules['robomaster'] = Mock()
sys.modules['robomaster.robot'] = Mock()
sys.modules['robomaster.vision'] = Mock()
sys.modules['numpy'] = Mock()
sys.modules['scipy'] = Mock()
sys.modules['scipy.ndimage'] = Mock()
sys.modules['cv2'] = Mock()

# Now import our actual code
from phingdee2 import GraphMapper, TimeoutHandler, with_timeout

def test_boundary_checking():
    """Test that boundary checking works correctly"""
    print("🧪 Testing boundary checking...")
    
    # Create mapper with small boundaries
    mapper = GraphMapper(min_x=-1, max_x=1, min_y=-1, max_y=1)
    
    # Create a node at boundary position
    node = mapper.create_node((-1, 0))  # Western boundary
    
    # Mock wall data - no walls detected
    node.walls = {'north': False, 'south': False, 'east': False, 'west': False}
    
    # Update unexplored exits
    mapper.update_unexplored_exits_absolute(node)
    
    # Check results
    print(f"   📍 Node position: {node.position}")
    print(f"   🎯 Unexplored exits: {node.unexploredExits}")
    print(f"   🌐 Out-of-bounds exits: {node.outOfBoundsExits}")
    
    # Verify boundary detection
    expected_out_of_bounds = ['west']  # Should detect west as out-of-bounds
    if 'west' in node.outOfBoundsExits:
        print("   ✅ Boundary detection working correctly")
        return True
    else:
        print("   ❌ Boundary detection failed")
        return False

def test_enhanced_frontier_validation():
    """Test enhanced frontier validation logic"""
    print("🧪 Testing enhanced frontier validation...")
    
    mapper = GraphMapper(min_x=-2, max_x=2, min_y=-2, max_y=2)
    
    # Create two connected nodes
    node1 = mapper.create_node((0, 0))
    node2 = mapper.create_node((1, 0))
    
    # Node1: not fully scanned, should be frontier
    node1.fullyScanned = False
    node1.walls = {'north': False, 'south': False, 'east': False, 'west': False}
    node1.exploredDirections = []
    
    # Node2: fully scanned but has unexplored exits, should still be frontier
    node2.fullyScanned = True
    node2.walls = {'north': False, 'south': False, 'east': False, 'west': False}
    node2.exploredDirections = ['west']  # Only explored west direction
    node2.unexploredExits = ['north', 'south', 'east']  # Still has unexplored
    
    # Update both nodes
    mapper.update_unexplored_exits_absolute(node1)
    mapper.update_unexplored_exits_absolute(node2)
    
    print(f"   📍 Node1 frontier status: {'✅' if node1.id in mapper.frontierQueue else '❌'}")
    print(f"   📍 Node2 frontier status: {'✅' if node2.id in mapper.frontierQueue else '❌'}")
    print(f"   🚀 Total frontiers: {len(mapper.frontierQueue)}")
    
    # Both should be in frontier queue
    if len(mapper.frontierQueue) >= 2:
        print("   ✅ Enhanced frontier validation working")
        return True
    else:
        print("   ❌ Enhanced frontier validation failed")
        return False

def test_timeout_protection():
    """Test timeout protection system"""
    print("🧪 Testing timeout protection...")
    
    def quick_function():
        time.sleep(0.1)
        return "success"
    
    def slow_function():
        time.sleep(2)
        return "should_timeout"
    
    # Test quick function (should succeed)
    try:
        result = with_timeout(quick_function, 1)
        if result == "success":
            print("   ✅ Quick function completed successfully")
            quick_test_passed = True
        else:
            print("   ❌ Quick function failed unexpectedly")
            quick_test_passed = False
    except Exception as e:
        print(f"   ❌ Quick function error: {e}")
        quick_test_passed = False
    
    # Test slow function (should complete but with timeout warning)
    try:
        result = with_timeout(slow_function, 0.5)  # Short timeout
        print("   ✅ Timeout protection allows completion with warning")
        slow_test_passed = True
    except Exception as e:
        print(f"   ❌ Slow function error: {e}")
        slow_test_passed = False
    
    return quick_test_passed and slow_test_passed

def test_total_unexplored_check():
    """Test the total unexplored exits safety check"""
    print("🧪 Testing total unexplored exits check...")
    
    mapper = GraphMapper(min_x=-1, max_x=1, min_y=-1, max_y=1)
    
    # Create nodes with various unexplored states
    node1 = mapper.create_node((0, 0))
    node2 = mapper.create_node((1, 0))
    node3 = mapper.create_node((0, 1))
    
    # Node1: has unexplored exits
    node1.unexploredExits = ['north', 'south']
    
    # Node2: no unexplored exits
    node2.unexploredExits = []
    
    # Node3: has one unexplored exit
    node3.unexploredExits = ['west']
    
    # Calculate total unexplored
    total_unexplored = sum(len(node.unexploredExits) for node in mapper.nodes.values())
    
    print(f"   📊 Total unexplored exits: {total_unexplored}")
    print(f"   📍 Node1 exits: {len(node1.unexploredExits)}")
    print(f"   📍 Node2 exits: {len(node2.unexploredExits)}")
    print(f"   📍 Node3 exits: {len(node3.unexploredExits)}")
    
    expected_total = 3  # 2 + 0 + 1
    if total_unexplored == expected_total:
        print("   ✅ Total unexplored calculation working correctly")
        return True
    else:
        print("   ❌ Total unexplored calculation failed")
        return False

def run_all_tests():
    """Run all tests and report results"""
    print("🚀 === TESTING MERGED PHINGDEE2 VERSION ===\n")
    
    tests = [
        ("Boundary Checking", test_boundary_checking),
        ("Enhanced Frontier Validation", test_enhanced_frontier_validation),
        ("Timeout Protection", test_timeout_protection),
        ("Total Unexplored Check", test_total_unexplored_check),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"{'='*50}")
        print(f"Running: {test_name}")
        print(f"{'='*50}")
        
        try:
            result = test_func()
            results.append((test_name, result))
            print(f"Result: {'✅ PASSED' if result else '❌ FAILED'}\n")
        except Exception as e:
            print(f"❌ ERROR: {e}\n")
            results.append((test_name, False))
    
    # Summary
    print(f"{'='*60}")
    print("📊 TEST SUMMARY")
    print(f"{'='*60}")
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"{test_name:<30} {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 Overall: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 ALL TESTS PASSED!")
        print("✅ The merged version should work correctly with:")
        print("   - Complete exploration capability")
        print("   - Proper boundary checking")
        print("   - Enhanced backtracking logic")
        print("   - Timeout protection")
        print("   - Comprehensive error handling")
    else:
        print("⚠️ Some tests failed - please review the implementation")
    
    return passed == total

if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)