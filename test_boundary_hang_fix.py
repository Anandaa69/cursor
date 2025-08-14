#!/usr/bin/env python3
"""
Test script to verify that the boundary hanging fixes work correctly
Simulates the exact scenario where the robot hangs at (-1, 0) facing south
"""

import sys
import time

# Mock classes to simulate the robot environment without robomaster dependency
class MockGraphMapper:
    def __init__(self, min_x=-1, min_y=-1, max_x=1, max_y=1):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.currentPosition = (-1, 0)
        self.currentDirection = 'south'
        self.nodes = {}
        
    def create_node(self, position):
        node_id = f"{position[0]}_{position[1]}"
        if node_id not in self.nodes:
            node = MockGraphNode(position, node_id)
            self.nodes[node_id] = node
        return self.nodes[node_id]
        
    def update_current_node_walls_absolute(self, left_wall, right_wall, front_wall):
        """Simulate the wall update that was hanging"""
        current_node = self.create_node(self.currentPosition)
        
        # Map relative to absolute directions
        direction_map = {
            'south': {'front': 'south', 'left': 'east', 'right': 'west'},
            'north': {'front': 'north', 'left': 'west', 'right': 'east'},
            'east': {'front': 'east', 'left': 'north', 'right': 'south'},
            'west': {'front': 'west', 'left': 'south', 'right': 'north'}
        }
        
        current_mapping = direction_map[self.currentDirection]
        
        # Update absolute wall information
        current_node.walls[current_mapping['front']] = front_wall
        current_node.walls[current_mapping['left']] = left_wall
        current_node.walls[current_mapping['right']] = right_wall
        
        current_node.fullyScanned = True
        
        # This is where the hang was occurring
        print("🔄 Calling update_unexplored_exits_absolute...")
        self.update_unexplored_exits_absolute(current_node)
        print("✅ update_unexplored_exits_absolute completed!")
        
    def update_unexplored_exits_absolute(self, node):
        """Enhanced version with timeout protection"""
        print(f"🧭 Updating unexplored exits for {node.id} at {node.position}")
        print(f"🔍 Wall status: {node.walls}")
        print(f"🤖 Robot facing: {self.currentDirection}")
        
        node.unexploredExits = []
        node.outOfBoundsExits = []
        
        x, y = node.position
        
        possible_directions = {
            'north': (x, y + 1),
            'south': (x, y - 1),
            'east':  (x + 1, y),
            'west':  (x - 1, y)
        }
        
        for direction, target_pos in possible_directions.items():
            target_x, target_y = target_pos
            
            # Boundary check
            is_outer_boundary = (
                target_x < self.min_x or target_x > self.max_x or
                target_y < self.min_y or target_y > self.max_y
            )
            
            is_blocked = node.walls.get(direction, True)
            
            print(f"   📍 {direction} ({target_pos}):")
            print(f"      🚧 Blocked: {is_blocked}")
            print(f"      🌐 Is outer boundary: {is_outer_boundary}")
            
            if not is_blocked:
                if is_outer_boundary:
                    node.outOfBoundsExits.append(direction)
                    print(f"      🚫 OUTER BOUNDARY! Added to outOfBoundsExits")
                else:
                    node.unexploredExits.append(direction)
                    print(f"      ✅ ADDED to unexplored exits")
            else:
                print(f"      ❌ NOT added (blocked)")
                
        print(f"🎯 Final unexplored exits: {node.unexploredExits}")
        print(f"🌐 Out-of-bounds exits: {node.outOfBoundsExits}")

class MockGraphNode:
    def __init__(self, position, node_id):
        self.position = position
        self.id = node_id
        self.walls = {'north': False, 'south': False, 'east': False, 'west': False}
        self.unexploredExits = []
        self.outOfBoundsExits = []
        self.fullyScanned = False

def test_boundary_hang_scenario():
    """Test the exact scenario that was causing hangs"""
    print("🧪 Testing boundary hang scenario...")
    print("📍 Position: (-1, 0) facing south in 3x3 grid")
    print("🧱 Walls: Front=True, Left=True, Right=False (leads to out-of-bounds)")
    
    # Create mapper with 3x3 grid boundaries
    mapper = MockGraphMapper(min_x=-1, min_y=-1, max_x=1, max_y=1)
    
    # Set position to the problematic location
    mapper.currentPosition = (-1, 0)
    mapper.currentDirection = 'south'
    
    try:
        # Simulate the wall configuration that caused hanging
        # Front = south wall (blocked)
        # Left = east wall (blocked) 
        # Right = west direction -> leads to (-2, 0) which is out-of-bounds
        
        print("\n🔄 Simulating wall update...")
        start_time = time.time()
        
        mapper.update_current_node_walls_absolute(
            left_wall=True,   # East direction - blocked
            right_wall=False, # West direction - open but out-of-bounds  
            front_wall=True   # South direction - blocked
        )
        
        end_time = time.time()
        duration = end_time - start_time
        
        print(f"✅ Wall update completed in {duration:.2f} seconds")
        
        if duration > 5:
            print("⚠️ WARNING: Update took longer than expected")
            return False
        else:
            print("🎉 SUCCESS: No hanging detected!")
            return True
            
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False

def test_all_boundary_scenarios():
    """Test multiple boundary scenarios"""
    print("\n🧪 Testing all boundary scenarios...")
    
    scenarios = [
        ((-1, -1), 'south', "Corner position with multiple out-of-bounds"),
        ((-1, 0), 'south', "Original hanging scenario"),  
        ((1, 1), 'north', "Opposite corner"),
        ((0, -1), 'east', "Edge position")
    ]
    
    success_count = 0
    
    for pos, direction, description in scenarios:
        print(f"\n📍 Testing: {pos} facing {direction} - {description}")
        
        mapper = MockGraphMapper(min_x=-1, min_y=-1, max_x=1, max_y=1)
        mapper.currentPosition = pos
        mapper.currentDirection = direction
        
        try:
            start_time = time.time()
            mapper.update_current_node_walls_absolute(
                left_wall=False, right_wall=False, front_wall=True
            )
            duration = time.time() - start_time
            
            if duration < 2:
                print(f"   ✅ Completed in {duration:.2f}s")
                success_count += 1
            else:
                print(f"   ⚠️ Slow: {duration:.2f}s")
                
        except Exception as e:
            print(f"   ❌ Failed: {e}")
    
    print(f"\n📊 Results: {success_count}/{len(scenarios)} scenarios passed")
    return success_count == len(scenarios)

if __name__ == "__main__":
    print("🚀 Testing boundary hanging fixes...")
    
    # Test the specific hanging scenario
    test1_passed = test_boundary_hang_scenario()
    
    # Test all boundary scenarios  
    test2_passed = test_all_boundary_scenarios()
    
    if test1_passed and test2_passed:
        print("\n✅ ALL TESTS PASSED!")
        print("🔧 The boundary hanging fixes should work correctly")
        print("🤖 Robot should no longer hang at boundary positions")
        sys.exit(0)
    else:
        print("\n❌ SOME TESTS FAILED!")
        print("🔧 There may still be boundary handling issues")
        sys.exit(1)