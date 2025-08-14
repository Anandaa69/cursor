# Boundary Handling and Marker Detection Fixes

## Issue Description

The robot was hanging when exploring nodes at the boundary of the map, specifically at position (-1, -1) when encountering open paths that lead to out-of-bounds areas. The Thai comment in the log indicated: "พอมันมองไปที่โหนดที่อยู่นอกแมพ เพราะตรงนั้นไม่มีกำแพง มันค้างไปเลยที่บรรทัดสุดท้าย" (When it looks at nodes outside the map, because there are no walls there, it hangs at the last line).

## Root Cause Analysis

1. **Boundary Processing Hang**: The `update_unexplored_exits_absolute` function could hang when processing out-of-bounds nodes due to lack of error handling.

2. **Scan Function Vulnerability**: The main scan function lacked timeout protection and proper error recovery mechanisms.

3. **Missing Enhanced Marker Detection**: The logs showed enhanced marker scanning with left-right sweep that wasn't implemented.

## Implemented Fixes

### 1. Enhanced Error Handling in Scan Function (`phingdee2.py`, `claude.py`)

```python
# Added comprehensive try-catch blocks around:
- Video stream operations
- Marker scanning loops  
- Gimbal movements
- Node wall updates
```

**Benefits:**
- Prevents hanging on camera/hardware errors
- Graceful degradation when components fail
- Continues exploration even if individual scans fail

### 2. Robust Boundary Processing (`update_unexplored_exits_absolute`)

```python
# Enhanced with:
- Try-catch around entire function
- Per-direction error handling 
- Safe defaults on critical errors
- Frontier queue error protection
```

**Benefits:**
- No more hanging on out-of-bounds nodes
- Proper classification of boundary vs valid exits
- Robust frontier management

### 3. Enhanced Marker Detection with Left-Right Sweep

```python
# Added enhanced scanning logic:
- Center position scanning
- Left offset (-15°) scanning  
- Right offset (+15°) scanning
- Combined unique marker collection
```

**Benefits:**
- Better marker detection accuracy
- Matches the enhanced scanning shown in logs
- More comprehensive marker coverage

### 4. Exploration Loop Safety Mechanisms

```python
# Added protection for:
- Backtracking timeout
- Frontier rebuild limits
- Total unexplored exit counting
- Error recovery in final checks
```

**Benefits:**
- Prevents infinite loops
- Better exploration completion detection
- Graceful termination on errors

## Key Code Changes

### Video Stream Safety
```python
# Before:
try:
    ep_camera.stop_video_stream()
    print("📹 Video stream stopped")
except:
    pass

# After:  
try:
    ep_camera.stop_video_stream()
    print("📹 Video stream stopped")
except Exception as e:
    print(f"⚠️ Error stopping video stream: {e}")
```

### Boundary Processing Safety
```python
# Before: Could hang on boundary conditions
def update_unexplored_exits_absolute(self, node):
    node.unexploredExits = []
    # ... processing without error handling

# After: Comprehensive error protection
def update_unexplored_exits_absolute(self, node):
    try:
        node.unexploredExits = []
        # ... processing with per-step error handling
    except Exception as e:
        print(f"❌ Critical error: {e}")
        # Safe defaults
```

### Enhanced Marker Scanning
```python
# Added complete left-right sweep implementation
if detected and marker_handler.markers:
    marker_ids = [m.id for m in marker_handler.markers]
    print(f"   🔄 Using enhanced marker scanning with left-right sweep...")
    # Center + Left (-15°) + Right (+15°) scanning
    # Combine unique markers from all angles
```

## Expected Results

1. **No More Hanging**: Robot should not hang when encountering out-of-bounds nodes
2. **Better Error Recovery**: Graceful handling of hardware/software errors
3. **Enhanced Marker Detection**: More accurate marker detection with sweep scanning
4. **Robust Exploration**: Complete exploration even with boundary conditions

## Files Modified

- `phingdee2.py`: Main implementation with all fixes
- `claude.py`: Consistency fixes for boundary handling
- `test_boundary_fix.py`: Test script for verification

## Testing Scenarios

The fixes address these specific scenarios:
1. Node at (-1, -1) with open east/west paths (out-of-bounds)
2. Camera/hardware errors during scanning
3. Marker detection at various angles
4. Exploration completion with boundary constraints

## Verification

To verify the fixes work:
1. Run exploration on a 3x3 grid (`min_x=-1, min_y=-1, max_x=1, max_y=1`)
2. Monitor for hanging at boundary nodes
3. Check that out-of-bounds exits are properly classified
4. Verify enhanced marker detection performs left-right sweeps

The robot should now complete exploration without hanging, even when encountering nodes with open paths to out-of-bounds areas.