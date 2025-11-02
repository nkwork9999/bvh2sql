# bvh2sql

BVH (BioVision Hierarchy) motion capture files parser for DuckDB.

## Overview

BVH files store skeletal animation data in a hierarchical structure with relative coordinates. This extension parses BVH files and calculates absolute world positions for all joints, making motion capture data easily queryable in DuckDB.

## Installation

```sql
INSTALL bvh2sql FROM community;
LOAD bvh2sql;
```

## Usage

### Basic Query

```sql
-- Read BVH file and get absolute positions
SELECT frame_id, time, joint_name, world_x, world_y, world_z
FROM bvh_absolute_positions('motion.bvh')
LIMIT 10;
```

### Analyze Joint Movement

```sql
-- Calculate Hips movement distance between frames
SELECT
  time,
  world_x AS "Hips.x",
  world_z AS "Hips.z",
  SQRT(
    POWER(world_x - LAG(world_x) OVER (ORDER BY time), 2) +
    POWER(world_z - LAG(world_z) OVER (ORDER BY time), 2)
  ) as distance_moved
FROM bvh_absolute_positions('motion.bvh')
WHERE joint_name = 'Hips'
LIMIT 20;
```

### Export to CSV

```sql
-- Export all positions to CSV
COPY (
  SELECT * FROM bvh_absolute_positions('motion.bvh')
) TO 'output_pos.csv' (HEADER, DELIMITER ',');
```

## BVH File Format

### HIERARCHY Section

Defines the skeletal structure:

- **ROOT**: Root segment name (usually "Hips")
- **OFFSET**: X, Y, Z offset from parent joint
- **CHANNELS**: Number and types of channels
  - Root typically has 6 channels (3 position + 3 rotation)
  - Other joints have 3 channels (rotation only)
- **JOINT**: Child joint definition
- **End Site**: Terminal node (end of bone chain)

### MOTION Section

Contains frame-by-frame animation data:

- **Frames**: Total number of frames
- **Frame Time**: Time interval between frames (seconds)
- Frame data rows with channel values

## Output Columns

| Column     | Type    | Description                        |
| ---------- | ------- | ---------------------------------- |
| frame_id   | INTEGER | Frame number (0-indexed)           |
| time       | DOUBLE  | Time in seconds                    |
| joint_name | VARCHAR | Name of the joint                  |
| world_x    | DOUBLE  | Absolute X position in world space |
| world_y    | DOUBLE  | Absolute Y position in world space |
| world_z    | DOUBLE  | Absolute Z position in world space |
| rot_x      | DOUBLE  | X-axis rotation (degrees)          |
| rot_y      | DOUBLE  | Y-axis rotation (degrees)          |
| rot_z      | DOUBLE  | Z-axis rotation (degrees)          |

## How it Works

BVH files store joint positions as **relative offsets** from parent joints. This extension:

1. Parses the hierarchical structure
2. Reads motion data frame by frame
3. Calculates **absolute world positions** using 4x4 transformation matrices
4. Applies translations and rotations (Z → X → Y order)
5. Returns data as a standard SQL table

## License

MIT License

```
``
```
